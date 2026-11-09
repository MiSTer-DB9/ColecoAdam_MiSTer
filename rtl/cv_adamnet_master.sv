//-----------------------------------------------------------------------------
//
// AdamNet bus master for the FujiNet link.
//
// On an ADAM the master 6801 turns each Device Control Block command into a
// conversation on the AdamNet wire with the addressed device. cv_adamnet
// normally plays both the master and every device itself, from the DCBs, as
// ColEm does. For devices served by FujiNet-PC on the HPS it has to speak the
// wire protocol for real, and this module is that master: one transaction per
// request, run byte by byte over cv_fuji_uart.
//
// The protocol, byte for byte (FujiNet lib/bus/adamnet/adamnet.h and
// FujiAdamPacket.cpp, and "Connecting an Emulator to FujiNet-PC", 2026, ch. 5
// and 8). Every byte is a command or response nibble over a device nibble.
//
//   status      MN_STATUS -> [8|dev][len lo][len hi][devtype][status][cksum]
//   block read  MN_SEND [00 05][blk LE32, 00][cksum] -> ACK
//               MN_RECEIVE -> ACK, re-polled every 5 ms while the device is
//               fetching (it answers nothing until it has the block)
//               MN_CLR -> [B|dev][04 00][1024 bytes][cksum]
//   block write MN_SEND block# -> ACK;  MN_SEND [04 00][1024 bytes][cksum] -> ACK
//   char write  MN_SEND [len BE][bytes][cksum] -> ACK or NAK
//   char read   MN_RECEIVE -> ACK (NAK = nothing to read); MN_CLR -> [B|dev][len BE][bytes][cksum]
//   reset       MN_RESET, no reply
//   boot CONFIG MN_SEND to F [00 02][D6 00][D6] -> ACK: the Fuji device's
//               SET_BOOT_MODE 0, which puts its CONFIG disk back into slot 1
//
// The checksum is the XOR of the payload bytes. Lengths in data packets are
// big-endian; the one inside the status packet is little-endian.
//
// Timing is the emulator bridge's, not the real bus's. A real device must
// answer within 300 us; FujiNet-PC on Linux takes milliseconds, so the ACK
// wait is 300 ms, and a block read rides out a slow TNFS fetch by re-polling
// RECEIVE for up to 20 s while EOS sits on the busy DCB. Before each command
// the master waits 1 ms and drops anything still queued: FujiNet-PC discards
// its input for 180 us after every command it processes, and a slow device
// answers every re-poll it was sent once the block arrives, so stray ACKs
// would otherwise be read as the data header (guide ch. 13).
//
// The 1024 byte payload buffer lives here. cv_adamnet fills or empties it
// through buf_* while the master is idle, and copies between it and ADAM RAM
// in a short burst, so the Z80 is never held off its bus for a wire transfer.
//
//-----------------------------------------------------------------------------

import cv_fuji_pkg::*;

module cv_adamnet_master
  #(
    parameter CLK_HZ = 42_666_000
    )
  (
   input               clk,
   input               reset,         // abort anything in flight
   input               enable,

   // request, taken when busy is low
   input               req,
   input [2:0]         req_op,
   input [3:0]         req_dev,
   input [31:0]        req_block,
   input [15:0]        req_len,       // char write: bytes to send; reads: buffer capacity
   input               req_patient,   // status: keep retrying for a while (daemon still starting)
   output logic        busy,
   output logic        done,          // one clock
   output logic [7:0]  result,        // FRES_*
   output logic [15:0] rsp_len,       // bytes delivered by a read; block size from a status
   output logic [7:0]  rsp_devtype,   // status packet fields
   output logic [7:0]  rsp_status,

   // payload buffer, cv_adamnet's port: only while busy is low
   input [9:0]         buf_addr,
   input               buf_wr,
   input [7:0]         buf_din,
   output logic [7:0]  buf_q,         // the byte at buf_addr, one clock later

   // cv_fuji_uart
   output logic [7:0]  tx_data,
   output logic        tx_wr,
   input               tx_busy,
   input [7:0]         rx_data,
   input               rx_avail,
   output logic        rx_rd,
   output logic        rx_flush
   );

  localparam [31:0] CLK_PER_MS = CLK_HZ / 1000;
  // FujiNet-PC clears its input after every command it answers and then drops
  // whatever arrives in the next 180 us, and a Linux tty hands it bytes in
  // chunks several ms apart. Two commands of ours that land in one chunk are
  // therefore one command to it: the second is thrown away with the first's
  // leftovers. A command sent after its reply has arrived is safe, so T_GAP
  // can stay short; the two places that send again without a reply in between,
  // the RECEIVE re-poll and the reset broadcast, wait longer than that latency.
  localparam [31:0] T_GAP      = CLK_PER_MS;          // before a command that follows a reply
  localparam [31:0] T_RESET    = CLK_PER_MS * 20;     // between the reset broadcast's bytes
  localparam [31:0] T_ACK      = CLK_PER_MS * 300;    // ACK or status reply
  localparam [31:0] T_ACK_W    = CLK_PER_MS * 1000;   // ACK after a 1K payload
  localparam [31:0] T_POLL     = CLK_PER_MS * 50;     // one RECEIVE re-poll (an ACK to the last
                                                      // one seen on a MiSTer took over 5 ms)
  localparam [31:0] T_BYTE     = CLK_PER_MS * 100;    // between bytes of a packet
  localparam [31:0] T_QUIET    = CLK_PER_MS * 5;      // line quiet this long before CLR
  localparam [31:0] T_HDR      = CLK_PER_MS * 500;    // data header after CLR: the device
                                                      // ACKed RECEIVE, so it has the data
  localparam [31:0] T_TOTAL    = CLK_PER_MS * 20000;  // re-poll budget, block devices (a TNFS fetch, retried)
  localparam [31:0] T_CHAR     = CLK_PER_MS * 5000;   // re-poll budget, character devices: a network
                                                      // device with nothing to read answers nothing
  localparam [31:0] T_PATIENT  = CLK_PER_MS * 20000;  // status retries when req_patient: Main
                                                      // restarts the daemon on every core load, and
                                                      // on a MiSTer it took 7 s to be listening again

  localparam [15:0] BLOCK_SIZE = 16'd1024;

  typedef enum logic [5:0] {
    S_IDLE, S_GAP, S_TXSEQ, S_TXWAIT, S_RX, S_OK, S_FAIL, S_DONE,
    ST_SEND, ST_WAIT, ST_BYTE, ST_TMO,
    RS_SEND,
    RA_SEND, RA_NEXT,
    BC_SEND,
    BN_HDR, BN_ACK, BN_RSP,
    RR_POLL, RR_WAITACK, RR_POLLRSP, RR_POLLTMO, RR_DRAIN, RR_CLR, RR_HDRWAIT, RR_HDRTMO, RR_HDR,
    RR_LENHI, RR_LENLO, RR_DATACHK, RR_DATA, RR_CKSUM,
    WR_HDR, WR_FETCH, WR_FETCH2, WR_BYTE, WR_NEXT, WR_CKSUM, WR_ACKWAIT, WR_ACK
  } state_t;

  state_t      state;
  state_t      gap_next, rx_next, rx_tmo_next, seq_next, txw_next;

  logic [31:0] timer;        // the current wait
  logic [31:0] total;        // since the request was taken
  logic [2:0]  op;
  logic [3:0]  dev;
  logic [31:0] blk;
  logic [15:0] len_req;      // capped at the buffer
  logic        patient;
  logic        is_block;
  logic [7:0]  rx_byte;
  logic [7:0]  cksum;
  logic [15:0] idx;
  logic [15:0] pkt_len;
  logic [15:0] pay_len;
  logic [2:0]  st_idx;
  logic [3:0]  ra_idx;

  // a short byte sequence to send
  logic [7:0]  seq[9];
  logic [3:0]  seq_len, seq_idx;

  // The devices FujiNet-PC registers on an ADAM: printer, clock, four disks,
  // two network adapters and the Fuji gateway (lib/device/adamnet, src/main.cpp)
  function automatic logic [3:0] reset_list(input logic [3:0] i);
    case (i)
      4'd0: reset_list = 4'h2;
      4'd1: reset_list = 4'h3;
      4'd2: reset_list = 4'h4;
      4'd3: reset_list = 4'h5;
      4'd4: reset_list = 4'h6;
      4'd5: reset_list = 4'h7;
      4'd6: reset_list = 4'h9;
      4'd7: reset_list = 4'hA;
      default: reset_list = 4'hF;
    endcase
  endfunction

  //--------------------------------------------------------------------------
  // Payload buffer: the master's while busy, cv_adamnet's otherwise
  //--------------------------------------------------------------------------
  logic [7:0] fbuf[1024];
  logic [9:0] m_addr;
  logic       m_wr;
  logic [7:0] m_din;
  logic [7:0] fb_q;

  wire [9:0] fb_addr = busy ? m_addr : buf_addr;
  wire       fb_wr   = busy ? m_wr   : buf_wr;
  wire [7:0] fb_din  = busy ? m_din  : buf_din;

  always_ff @(posedge clk) begin
    if (fb_wr) fbuf[fb_addr] <= fb_din;
    fb_q <= fbuf[fb_addr];
  end
  assign buf_q = fb_q;

  //--------------------------------------------------------------------------
  // The master
  //--------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    done     <= '0;
    tx_wr    <= '0;
    rx_rd    <= '0;
    rx_flush <= '0;
    m_wr     <= '0;
    if (busy) total <= total + 1'b1;

    case (state)
      S_IDLE: begin
        if (req && enable) begin
          op       <= req_op;
          dev      <= req_dev;
          blk      <= req_block;
          len_req  <= (req_len > BLOCK_SIZE) ? BLOCK_SIZE : req_len;
          patient  <= req_patient;
          is_block <= (req_op == FOP_READ_BLOCK) || (req_op == FOP_WRITE_BLOCK);
          total    <= '0;
          result   <= FRES_TIMEOUT;
          rsp_len  <= '0;
          ra_idx   <= '0;
          busy     <= '1;
          timer    <= T_GAP;
          state    <= S_GAP;
          case (req_op)
            FOP_STATUS:      gap_next <= ST_SEND;
            FOP_RESET:       gap_next <= RS_SEND;
            FOP_RESET_ALL:   gap_next <= RA_SEND;
            FOP_READ_BLOCK,
            FOP_WRITE_BLOCK: gap_next <= BN_HDR;
            FOP_CHAR_WRITE:  gap_next <= WR_HDR;
            FOP_CHAR_READ:   gap_next <= RR_POLL;
            FOP_BOOT_CONFIG: gap_next <= BC_SEND;
            default:         gap_next <= S_FAIL;
          endcase
        end
      end

      // Wait, dropping whatever the device still had to say
      S_GAP: begin
        rx_flush <= '1;
        if (timer == 0) state <= gap_next;
        else timer <= timer - 1'b1;
      end

      // Send seq[0..seq_len-1]
      S_TXSEQ: begin
        if (seq_idx == seq_len) state <= seq_next;
        else if (!tx_busy) begin
          tx_data <= seq[seq_idx];
          tx_wr   <= '1;
          seq_idx <= seq_idx + 1'b1;
        end
      end

      // Wait for the transmitter to finish
      S_TXWAIT: begin
        if (!tx_busy) state <= txw_next;
      end

      // Wait for one byte, or give up after timer
      S_RX: begin
        if (rx_avail) begin
          rx_byte <= rx_data;
          rx_rd   <= '1;
          state   <= rx_next;
        end else if (timer == 0) begin
          state <= rx_tmo_next;
        end else begin
          timer <= timer - 1'b1;
        end
      end

      // The failing state has set result to the 6801's code for what went wrong
      S_OK:   begin result <= FRES_OK; state <= S_DONE; end
      S_FAIL: state <= S_DONE;
      S_DONE: begin
        done  <= '1;
        busy  <= '0;
        state <= S_IDLE;
      end

      //------------------------------------------------------------------
      // Status
      //------------------------------------------------------------------
      ST_SEND: begin
        seq[0]   <= {MN_STATUS, dev};
        seq_len  <= 4'd1;
        seq_idx  <= '0;
        seq_next <= ST_WAIT;
        state    <= S_TXSEQ;
      end
      ST_WAIT: begin
        st_idx      <= '0;
        cksum       <= '0;
        result      <= FRES_STAT_TIMEOUT;
        timer       <= T_ACK;
        rx_next     <= ST_BYTE;
        rx_tmo_next <= ST_TMO;
        state       <= S_RX;
      end
      ST_BYTE: begin
        timer   <= T_BYTE;
        rx_next <= ST_BYTE;
        state   <= S_RX;
        case (st_idx)
          3'd0: begin
            // anything but our status header is a stray; keep waiting for it
            if (rx_byte == {NM_STATUS, dev}) st_idx <= 3'd1;
            else timer <= T_ACK;
          end
          3'd1: begin rsp_len[7:0]  <= rx_byte; cksum <= cksum ^ rx_byte; st_idx <= 3'd2; end
          3'd2: begin rsp_len[15:8] <= rx_byte; cksum <= cksum ^ rx_byte; st_idx <= 3'd3; end
          3'd3: begin rsp_devtype   <= rx_byte; cksum <= cksum ^ rx_byte; st_idx <= 3'd4; end
          3'd4: begin rsp_status    <= rx_byte; cksum <= cksum ^ rx_byte; st_idx <= 3'd5; end
          default: begin
            if (rx_byte == cksum) state <= S_OK;
            else begin
              result <= FRES_TIMEOUT;
              state  <= S_FAIL;
            end
          end
        endcase
      end
      ST_TMO: begin
        // Nothing answered. While the daemon may still be starting, try again.
        if (patient && total < T_PATIENT) begin
          timer    <= T_GAP;
          gap_next <= ST_SEND;
          state    <= S_GAP;
        end else begin
          state <= S_FAIL;
        end
      end

      //------------------------------------------------------------------
      // Reset: one device, or every FujiNet device in turn
      //------------------------------------------------------------------
      RS_SEND: begin
        seq[0]   <= {MN_RESET, dev};
        seq_len  <= 4'd1;
        seq_idx  <= '0;
        seq_next <= S_TXWAIT;
        txw_next <= S_OK;
        state    <= S_TXSEQ;
      end
      RA_SEND: begin
        seq[0]   <= {MN_RESET, reset_list(ra_idx)};
        seq_len  <= 4'd1;
        seq_idx  <= '0;
        seq_next <= S_TXWAIT;
        txw_next <= RA_NEXT;
        state    <= S_TXSEQ;
      end
      RA_NEXT: begin
        timer    <= T_RESET;
        gap_next <= (ra_idx == 4'd8) ? S_OK : RA_SEND;
        ra_idx   <= ra_idx + 1'b1;
        state    <= S_GAP;
      end

      //------------------------------------------------------------------
      // Boot CONFIG: a two byte SEND to the Fuji device, then its ACK
      //------------------------------------------------------------------
      BC_SEND: begin
        seq[0]   <= {MN_SEND, dev};
        seq[1]   <= 8'h00;
        seq[2]   <= 8'h02;
        seq[3]   <= 8'hD6;                 // FUJI_SET_BOOT_MODE
        seq[4]   <= 8'h00;                 // 0: the CONFIG disk
        seq[5]   <= 8'hD6;                 // checksum: D6 ^ 00
        seq_len  <= 4'd6;
        seq_idx  <= '0;
        seq_next <= WR_ACKWAIT;
        state    <= S_TXSEQ;
      end

      //------------------------------------------------------------------
      // Block number: MN_SEND, length 5, block little-endian plus a zero
      //------------------------------------------------------------------
      BN_HDR: begin
        seq[0]   <= {MN_SEND, dev};
        seq[1]   <= 8'h00;
        seq[2]   <= 8'h05;
        seq[3]   <= blk[7:0];
        seq[4]   <= blk[15:8];
        seq[5]   <= blk[23:16];
        seq[6]   <= blk[31:24];
        seq[7]   <= 8'h00;
        seq[8]   <= blk[7:0] ^ blk[15:8] ^ blk[23:16] ^ blk[31:24];
        seq_len  <= 4'd9;
        seq_idx  <= '0;
        seq_next <= BN_ACK;
        state    <= S_TXSEQ;
      end
      BN_ACK: begin
        result      <= FRES_SEND_TIMEOUT;
        timer       <= T_ACK;
        rx_next     <= BN_RSP;
        rx_tmo_next <= S_FAIL;
        state       <= S_RX;
      end
      BN_RSP: begin
        if (rx_byte == {NM_ACK, dev}) begin
          timer    <= T_GAP;
          gap_next <= (op == FOP_READ_BLOCK) ? RR_POLL : WR_HDR;
          state    <= S_GAP;
        end else if (rx_byte == {NM_NAK, dev}) begin
          result <= FRES_SEND_NACK;
          state  <= S_DONE;
        end else begin
          state <= BN_ACK;                 // a stray; wait for the real answer
        end
      end

      //------------------------------------------------------------------
      // Read: RECEIVE until the device ACKs, then CLR for the data packet
      //------------------------------------------------------------------
      RR_POLL: begin
        seq[0]   <= {MN_RECEIVE, dev};
        seq_len  <= 4'd1;
        seq_idx  <= '0;
        seq_next <= RR_WAITACK;
        state    <= S_TXSEQ;
      end
      RR_WAITACK: begin
        timer       <= T_POLL;
        rx_next     <= RR_POLLRSP;
        rx_tmo_next <= RR_POLLTMO;
        state       <= S_RX;
      end
      RR_POLLRSP: begin
        if (rx_byte == {NM_ACK, dev}) begin
          timer <= T_QUIET;
          state <= RR_DRAIN;
        end else if (rx_byte == {NM_NAK, dev}) begin
          result <= FRES_RECV_NACK;        // nothing to read
          state  <= S_DONE;
        end else begin
          state <= RR_WAITACK;
        end
      end
      RR_POLLTMO: begin
        // Silence: the device is still fetching. Ask again, within the budget.
        if (total >= (is_block ? T_TOTAL : T_CHAR)) begin
          result <= FRES_RECV_TIMEOUT;
          state  <= S_DONE;
        end else begin
          timer    <= T_GAP;
          gap_next <= RR_POLL;
          state    <= S_GAP;
        end
      end
      // Drop the ACKs to earlier re-polls until the line has been quiet for T_QUIET. A
      // device that was fetching answers every re-poll queued meanwhile, one by one, and
      // FujiNet-PC throws away input that arrives while it is still busy with them: a
      // CLR sent into that is lost, and the device never sends its data.
      RR_DRAIN: begin
        if (rx_avail) begin
          rx_rd <= '1;
          timer <= T_QUIET;
        end else if (timer == 0) begin
          state <= RR_CLR;
        end else begin
          timer <= timer - 1'b1;
        end
      end
      RR_CLR: begin
        seq[0]   <= {MN_CLR, dev};
        seq_len  <= 4'd1;
        seq_idx  <= '0;
        seq_next <= RR_HDRWAIT;
        state    <= S_TXSEQ;
      end
      RR_HDRWAIT: begin
        result      <= FRES_CLR_TIMEOUT;   // also for a packet that stops part way
        timer       <= T_HDR;
        rx_next     <= RR_HDR;
        rx_tmo_next <= RR_HDRTMO;
        state       <= S_RX;
      end
      RR_HDRTMO: begin
        // The CLR went unanswered, so it was lost: the device still has the data, and
        // answers another RECEIVE with an ACK. Ask again, within the budget, rather
        // than leave an error in the DCB, which ADAM CP/M's BIOS never gets past.
        if (total >= (is_block ? T_TOTAL : T_CHAR)) begin
          state <= S_FAIL;
        end else begin
          timer    <= T_GAP;
          gap_next <= RR_POLL;
          state    <= S_GAP;
        end
      end
      RR_HDR: begin
        if (rx_byte == {NM_SEND, dev}) begin
          timer   <= T_BYTE;
          rx_next <= RR_LENHI;
          state   <= S_RX;
        end else if (rx_byte == {NM_ACK, dev}) begin
          state <= RR_HDRWAIT;             // a late ACK that slipped past the gap
        end else if (rx_byte == {NM_NAK, dev}) begin
          result <= FRES_RECV_NACK;
          state  <= S_DONE;
        end else begin
          result <= FRES_TIMEOUT;
          state  <= S_FAIL;
        end
      end
      RR_LENHI: begin
        pkt_len[15:8] <= rx_byte;
        timer   <= T_BYTE;
        rx_next <= RR_LENLO;
        state   <= S_RX;
      end
      RR_LENLO: begin
        pkt_len[7:0] <= rx_byte;
        idx   <= '0;
        cksum <= '0;
        state <= RR_DATACHK;
      end
      RR_DATACHK: begin
        timer <= T_BYTE;
        if (pkt_len > BLOCK_SIZE) begin             // not a packet we can hold
          result <= FRES_TIMEOUT;
          state  <= S_FAIL;
        end
        else if (idx == pkt_len) begin
          rx_next <= RR_CKSUM;
          state   <= S_RX;
        end else begin
          rx_next <= RR_DATA;
          state   <= S_RX;
        end
      end
      RR_DATA: begin
        m_addr <= idx[9:0];
        m_din  <= rx_byte;
        m_wr   <= '1;
        cksum  <= cksum ^ rx_byte;
        idx    <= idx + 1'b1;
        state  <= RR_DATACHK;
      end
      RR_CKSUM: begin
        rsp_len <= (pkt_len > len_req) ? len_req : pkt_len;
        if (rx_byte == cksum) state <= S_OK;
        else begin
          result <= FRES_TIMEOUT;          // a bad packet: let EOS retry
          state  <= S_FAIL;
        end
      end

      //------------------------------------------------------------------
      // Write: MN_SEND, big-endian length, the buffer, checksum; expect ACK
      //------------------------------------------------------------------
      WR_HDR: begin
        pay_len  <= is_block ? BLOCK_SIZE : len_req;
        seq[0]   <= {MN_SEND, dev};
        seq[1]   <= is_block ? 8'h04 : len_req[15:8];   // 1024, big-endian
        seq[2]   <= is_block ? 8'h00 : len_req[7:0];
        seq_len  <= 4'd3;
        seq_idx  <= '0;
        seq_next <= WR_FETCH;
        idx      <= '0;
        cksum    <= '0;
        state    <= S_TXSEQ;
      end
      WR_FETCH: begin
        if (idx == pay_len) state <= WR_CKSUM;
        else begin
          m_addr <= idx[9:0];
          state  <= WR_FETCH2;
        end
      end
      WR_FETCH2: state <= WR_BYTE;       // fb_q catches up with m_addr
      WR_BYTE: begin
        if (!tx_busy) begin
          tx_data <= fb_q;
          tx_wr   <= '1;
          cksum   <= cksum ^ fb_q;
          idx     <= idx + 1'b1;
          state   <= WR_FETCH;
        end
      end
      WR_CKSUM: begin
        if (!tx_busy) begin
          tx_data <= cksum;
          tx_wr   <= '1;
          state   <= WR_ACKWAIT;
        end
      end
      WR_ACKWAIT: begin
        result      <= FRES_SEND_TIMEOUT;
        timer       <= T_ACK_W;
        rx_next     <= WR_ACK;
        rx_tmo_next <= S_FAIL;
        state       <= S_RX;
      end
      WR_ACK: begin
        if (rx_byte == {NM_ACK, dev}) state <= S_OK;
        else if (rx_byte == {NM_NAK, dev}) begin
          result <= FRES_SEND_NACK;
          state  <= S_DONE;
        end else state <= WR_ACKWAIT;
      end

      default: state <= S_IDLE;
    endcase

    if (reset || !enable) begin
      if (busy) begin
        done   <= '1;
        result <= FRES_TIMEOUT;
      end
      busy  <= '0;
      state <= S_IDLE;
    end
  end

  initial begin
    state = S_IDLE;
    busy  = '0;
    done  = '0;
    tx_wr = '0;
    rx_rd = '0;
    rx_flush = '0;
    m_wr  = '0;
  end

endmodule
