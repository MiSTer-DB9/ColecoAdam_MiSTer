//-----------------------------------------------------------------------------
//
// ADAM printer output: a FIFO in front of an 8N1 serial transmitter.
//
// cv_adamnet hands over one printer byte per prn_strobe and only fetches the
// next while ready is high, so a full FIFO holds EOS off the way a busy printer
// does. The serial side goes to the HPS UART, where mister_printerd turns the
// stream into a PDF when the OSD's UART connection is set to Printer.
//
// The bit rate is whatever Main reports in uart_speed. It is made with a
// phase accumulator rather than a divider, so any rate Main offers (110 to
// 115200 baud) comes out right on average with at most one clock of jitter.
//
// While the port is not in use (enable low, or no speed yet) bytes are thrown
// away and ready stays high, so printing never hangs the ADAM with no printer
// attached, and TXD sits at the idle mark level.
//
//-----------------------------------------------------------------------------

module cv_printer_uart
  #(
    parameter CLK_HZ  = 42_666_000,
    parameter FIFO_AW = 9             // 512 bytes
    )
  (
   input               clk,
   input               reset,
   input               enable,
   input [31:0]        baud,

   input [7:0]         din,
   input               wr,
   output              ready,

   output logic        txd
   );

  logic               active;
  assign active = enable && (baud != 0);

  // FIFO
  logic [7:0]         fifo[2**FIFO_AW];
  logic [FIFO_AW-1:0] wp, rp;
  logic               empty, full;

  logic [7:0]         fifo_q;
  logic               avail;              // fifo_q holds the byte at rp

  assign empty = (wp == rp);
  assign full  = ((wp + 1'b1) == rp);
  assign ready = ~active | ~full;

  // Registered read, so the FIFO fits a block RAM. avail lags wp and rp by the
  // same clock as fifo_q, so a byte is only taken once fifo_q has caught up.
  always_ff @(posedge clk) begin
    if (wr && active && !full) fifo[wp] <= din;
    fifo_q <= fifo[rp];
  end

  // Bit clock: accumulate baud every clock, tick each time CLK_HZ is crossed
  logic [31:0]        acc;
  logic               tick;

  always_ff @(posedge clk) begin
    tick <= '0;
    if (!active) begin
      acc <= '0;
    end else if (acc >= CLK_HZ - baud) begin
      acc  <= acc + baud - CLK_HZ;
      tick <= '1;
    end else begin
      acc  <= acc + baud;
    end
  end

  // Transmitter: start bit, 8 data bits LSB first, stop bit
  logic [8:0]         shift;
  logic [3:0]         bits_left;

  always_ff @(posedge clk) begin
    avail <= !empty;
    if (wr && active && !full) wp <= wp + 1'b1;

    if (tick) begin
      if (bits_left != 0) begin
        txd       <= shift[0];
        shift     <= {1'b1, shift[8:1]};
        bits_left <= bits_left - 1'b1;
      end else if (avail && !empty) begin
        txd       <= 1'b0;                  // start bit
        shift     <= {1'b1, fifo_q};        // data then the stop bit
        bits_left <= 4'd9;                  // 8 data + stop
        rp        <= rp + 1'b1;
      end else begin
        txd <= 1'b1;
      end
    end

    if (reset || !active) begin
      wp        <= '0;
      rp        <= '0;
      bits_left <= '0;
      avail     <= '0;
      txd       <= 1'b1;
    end
  end

  initial begin
    wp        = '0;
    rp        = '0;
    bits_left = '0;
    txd       = 1'b1;
    acc       = '0;
  end

endmodule
