//-----------------------------------------------------------------------------
//
// FujiNet link: an 8N1 UART, transmit and receive, between the AdamNet master
// in cv_adamnet and the HPS UART. On the MiSTer the far end is FujiNet-PC built
// for the ADAM, reading /dev/ttyS1; in the simulator it is a TCP socket.
//
// AdamNet proper is a 62.5 kbaud half-duplex single wire (ADAM Technical
// Manual 2.1.6). This link is two wires and full duplex, so there is no echo
// of our own bytes to drop, and the rate is whatever Main reports in
// uart_speed, made with the same phase accumulator as cv_printer_uart. The
// receiver oversamples at 16x and takes each bit in its middle.
//
// Received bytes queue in a small FIFO. The master pops them one at a time,
// and can flush the queue: that is how it drops the surplus ACKs a slow
// device sends in reply to the master's re-polls before asking for data
// (see cv_adamnet_master).
//
//-----------------------------------------------------------------------------

module cv_fuji_uart
  #(
    parameter CLK_HZ = 42_666_000,
    parameter RX_AW  = 4               // 16 byte receive FIFO
    )
  (
   input               clk,
   input               reset,
   input               enable,
   input [31:0]        baud,

   // transmit: one byte per tx_wr, only while tx_busy is low
   input [7:0]         tx_data,
   input               tx_wr,
   output              tx_busy,

   // receive FIFO
   output [7:0]        rx_data,
   output              rx_avail,
   input               rx_rd,          // pop rx_data
   input               rx_flush,       // drop everything queued
   output logic        rx_err,         // one clock per framing error

   output logic        txd,
   input               rxd
   );

  logic active;
  assign active = enable && (baud != 0);

  // 16x bit clock: accumulate 16*baud every clock, tick each time CLK_HZ is crossed
  logic [31:0] acc;
  logic        tick;
  wire  [31:0] step = baud << 4;

  always_ff @(posedge clk) begin
    tick <= '0;
    if (!active) begin
      acc <= '0;
    end else if (acc >= CLK_HZ - step) begin
      acc  <= acc + step - CLK_HZ;
      tick <= '1;
    end else begin
      acc  <= acc + step;
    end
  end

  //--------------------------------------------------------------------------
  // Transmitter: start, 8 data bits LSB first, stop. Each bit lasts 16 ticks.
  //--------------------------------------------------------------------------
  logic [9:0] tx_shift;
  logic [3:0] tx_bits;        // bits still to put on the pin
  logic [3:0] tx_cnt;         // ticks left in the current bit
  logic       tx_busy_r;

  // Busy from the clock the byte is accepted, so a second tx_wr the very next
  // clock is refused rather than overwriting the shift register.
  assign tx_busy = tx_busy_r | tx_wr;

  always_ff @(posedge clk) begin
    if (tx_wr && !tx_busy_r) begin
      tx_shift  <= {1'b1, tx_data, 1'b0};  // stop, data, start
      tx_bits   <= 4'd10;
      tx_cnt    <= '0;
      tx_busy_r <= '1;
    end else if (tick && tx_busy_r) begin
      if (tx_cnt != 0) begin
        tx_cnt <= tx_cnt - 1'b1;
      end else if (tx_bits != 0) begin
        txd      <= tx_shift[0];
        tx_shift <= {1'b1, tx_shift[9:1]};
        tx_bits  <= tx_bits - 1'b1;
        tx_cnt   <= 4'd15;
      end else begin
        tx_busy_r <= '0;                     // the stop bit has had its full time
      end
    end

    if (reset || !active) begin
      tx_busy_r <= '0;
      tx_bits   <= '0;
      txd       <= 1'b1;
    end
  end

  //--------------------------------------------------------------------------
  // Receiver
  //--------------------------------------------------------------------------
  logic [1:0] rx_sync;
  wire        rx_in = rx_sync[1];

  typedef enum logic [1:0] { RX_IDLE, RX_START, RX_DATA, RX_STOP } rx_state_t;
  rx_state_t  rx_state;
  logic [3:0] rx_cnt;
  logic [2:0] rx_bit;
  logic [7:0] rx_shift;

  logic [7:0]        rx_fifo[2**RX_AW];
  logic [RX_AW-1:0]  wp, rp;
  logic              push;
  logic [7:0]        push_data;

  assign rx_avail = (wp != rp);
  assign rx_data  = rx_fifo[rp];

  always_ff @(posedge clk) begin
    rx_sync <= {rx_sync[0], rxd};
    rx_err  <= '0;
    push    <= '0;

    if (tick) begin
      case (rx_state)
        RX_IDLE: begin
          if (!rx_in) begin                  // a start bit's leading edge
            rx_state <= RX_START;
            rx_cnt   <= 4'd6;                // to the middle of the start bit
          end
        end
        RX_START: begin
          if (rx_cnt != 0) rx_cnt <= rx_cnt - 1'b1;
          else if (rx_in) rx_state <= RX_IDLE; // a glitch, not a start bit
          else begin
            rx_state <= RX_DATA;
            rx_cnt   <= 4'd15;
            rx_bit   <= '0;
          end
        end
        RX_DATA: begin
          if (rx_cnt != 0) rx_cnt <= rx_cnt - 1'b1;
          else begin
            rx_shift <= {rx_in, rx_shift[7:1]};
            rx_cnt   <= 4'd15;
            if (rx_bit == 3'd7) rx_state <= RX_STOP;
            rx_bit <= rx_bit + 1'b1;
          end
        end
        RX_STOP: begin
          if (rx_cnt != 0) rx_cnt <= rx_cnt - 1'b1;
          else begin
            if (rx_in) begin
              push      <= '1;
              push_data <= rx_shift;
            end else begin
              rx_err <= '1;                  // no stop bit: drop the byte
            end
            rx_state <= RX_IDLE;
          end
        end
      endcase
    end

    // FIFO. A full FIFO drops the newest byte; the master drains far faster
    // than any byte can arrive, so that only happens while it is idle, and
    // whatever lands then is a stray it would have flushed anyway.
    if (push && ((wp + 1'b1) != rp)) begin
      rx_fifo[wp] <= push_data;
      wp          <= wp + 1'b1;
    end
    if (rx_flush)      rp <= (push && ((wp + 1'b1) != rp)) ? wp + 1'b1 : wp;
    else if (rx_rd && rx_avail) rp <= rp + 1'b1;

    if (reset || !active) begin
      rx_state <= RX_IDLE;
      wp       <= '0;
      rp       <= '0;
    end
  end

  initial begin
    acc       = '0;
    txd       = 1'b1;
    tx_busy_r = '0;
    tx_bits   = '0;
    rx_state  = RX_IDLE;
    rx_sync   = 2'b11;
    wp        = '0;
    rp        = '0;
  end

endmodule
