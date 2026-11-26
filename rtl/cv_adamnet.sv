//-----------------------------------------------------------------------------
//
// FPGA Colecovision AdamNet
//
// Adamnet implementation for Disk, Tape and Printer
//
// Based upon:
// ColEm: portable Coleco emulator
//
//                       AdamNet.c
//
// This file contains implementation for the AdamNet I/O
// interface found in Coleco Adam home computer.
//
// Copyright (C) Marat Fayzullin 1994-2021
//
// References:
//
//   * Dan Boris' schematics of the Colecovision board
//     http://www.atarihq.com/danb/files/colecovision.pdf
//
//   * Schematics of the Colecovision controller, same source
//     http://www.atarihq.com/danb/files/ColecoController.pdf
//
//   * Technical information, same source
//     http://www.atarihq.com/danb/files/CV-Tech.txt
//
//-----------------------------------------------------------------------------
//
// Copyright (c) 2022, Frank Bruno (fbruno@asicsolutions.com)
//
// All rights reserved
//
// Redistribution and use in source and synthezised forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// Redistributions in synthesized form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// Neither the name of the author nor the names of other contributors may
// be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please report bugs to the author, but before you do so, please
// make sure that this is not a derivative work and that
// you have the latest version of this file.
//
//-----------------------------------------------------------------------------

import cv_fuji_pkg::*;

module cv_adamnet
  #
  (
   parameter NUM_DISKS = 1,
   parameter NUM_TAPES = 1,
   parameter USE_REQ = 1,
   parameter CLK_HZ = 42_666_000,   // for the FujiNet master's timeouts
   parameter TOT_DISKS = NUM_DISKS + NUM_TAPES
   )
  (
   input                        clk_i,
   input                        adam_reset_pcb_n_i /*verilator public_flat*/,
   input                        z80_wr/*verilator public_flat*/,
   input                        z80_rd/*verilator public_flat*/,
   input                        z80_rd_lvl,
   input [15:0]                 z80_addr/*verilator public_flat*/,
   input [7:0]                  z80_data_wr/*verilator public_flat*/,
   input [7:0]                  z80_data_rd/*verilator public_flat*/,

   // Dual port or mux into ADAM system memory
   output logic [15:0]          ramb_addr,
   output logic                 ramb_wr,
   output logic                 ramb_rd,
   output logic [7:0]           ramb_dout,
   input  [7:0]                 ramb_din,
   input                        ramb_wr_ack,
   input                        ramb_rd_ack,

   // Keyboard interface. Not sure how we should do this
   output logic [7:0]           kbd_status,
   output logic                 kbd_status_upd,
   output logic                 lastkey_in_valid,
   input logic [7:0]            lastkey_in,
   output logic                 lastkey_in_ack,
   output logic [7:0]           lastkey_out,
   input logic [10:0]           ps2_key,

   // Disk interface
   input logic [TOT_DISKS-1:0]  disk_present,
   output logic [31:0]          disk_sector, // sector
   output logic [TOT_DISKS-1:0] disk_load, // load the 512 byte sector
   input logic [TOT_DISKS-1:0]  disk_sector_loaded, // set high when sector ready
   output logic [8:0]           disk_addr, // Byte to read or write from sector
   output logic [TOT_DISKS-1:0] disk_wr, // Write data into sector (read when low)
   output logic [TOT_DISKS-1:0] disk_flush, // sector access done, so flush (hint)
   input logic [TOT_DISKS-1:0]  disk_flushed, // sector access done, so flush (hint)
   input logic [TOT_DISKS-1:0]  disk_error, // out of bounds (?)
   input logic [7:0]            disk_data[TOT_DISKS],
   output logic [7:0]           disk_din,

   // Printer (AdamNet device 2). One byte per prn_strobe, taken from the DCB
   // buffer in order. A byte is only fetched while prn_ready is high, so a
   // FIFO behind this can hold the Z80 off by filling up.
   output logic [7:0]           prn_data,
   output logic                 prn_strobe,
   input logic                  prn_ready,

   output logic                 adamnet_req_n,
   input logic                  adamnet_ack_n,
   output logic                 adamnet_wait_n,
   output logic                 adamnet_sel,
   output logic [7:0]           adamnet_dout,

   // FujiNet. While fuji_en is high the devices FujiNet-PC serves are not
   // emulated here but reached over the AdamNet wire, byte by byte through
   // cv_fuji_uart (see cv_adamnet_master and the is_fuji rule below).
   input logic                  fuji_en,
   // OSD "FujiNet: boot CONFIG": held high across the ADAM reset that goes with
   // it, until fuji_boot_cfg_ack says the daemon has CONFIG in slot 1 again
   input logic                  fuji_boot_cfg,
   output logic                 fuji_boot_cfg_ack,
   output logic [7:0]           fuji_tx_data,
   output logic                 fuji_tx_wr,
   input logic                  fuji_tx_busy,
   input logic [7:0]            fuji_rx_data,
   input logic                  fuji_rx_avail,
   output logic                 fuji_rx_rd,
   output logic                 fuji_rx_flush
   );

  //assign adamnet_dout = ramb_dout;

  localparam PCB_BASE_INIT = 16'hFEC0; // PCB base address on reset

  /** PCB Field Offsets ****************************************/
  localparam PCB_CMD_STAT   = 0;
  localparam PCB_BA_LO      = 1;
  localparam PCB_BA_HI      = 2;
  localparam PCB_MAX_DCB    = 3;
  localparam PCB_SIZE       = 4;

  typedef struct {
    logic [7:0]  pcb_cmd_stat;
    logic [7:0]  pcb_ba_lo;
    logic [7:0]  pcb_ba_hi;
    logic [7:0]  pcb_max_dcb;
  } pcb_table_t;

  pcb_table_t pcb_table;

  /** DCB Field Offsets ****************************************/
  localparam DCB_CMD_STAT   = 0;
  localparam DCB_BA_LO      = 1;
  localparam DCB_BA_HI      = 2;
  localparam DCB_BUF_LEN_LO = 3;
  localparam DCB_BUF_LEN_HI = 4;
  localparam DCB_SEC_NUM_0  = 5;
  localparam DCB_SEC_NUM_1  = 6;
  localparam DCB_SEC_NUM_2  = 7;
  localparam DCB_SEC_NUM_3  = 8;
  localparam DCB_DEV_NUM    = 9;
  localparam DCB_RETRY_LO   = 14;
  localparam DCB_RETRY_HI   = 15;
  localparam DCB_ADD_CODE   = 16;
  localparam DCB_MAXL_LO    = 17;
  localparam DCB_MAXL_HI    = 18;
  localparam DCB_DEV_TYPE   = 19;
  localparam DCB_NODE_TYPE  = 20;
  localparam DCB_SIZE       = 21;

  typedef struct {
    logic [7:0]  dcb_cmd_stat;
    logic [7:0]  dcb_ba_lo;
    logic [7:0]  dcb_ba_hi;
    logic [7:0]  dcb_buf_len_lo;
    logic [7:0]  dcb_buf_len_hi;
    logic [7:0]  dcb_sec_num_0;
    logic [7:0]  dcb_sec_num_1;
    logic [7:0]  dcb_sec_num_2;
    logic [7:0]  dcb_sec_num_3;
    logic [7:0]  dcb_dev_num;
    logic [7:0]  dcb_retry_lo;
    logic [7:0]  dcb_retry_hi;
    logic [7:0]  dcb_add_code;
    logic [7:0]  dcb_maxl_lo;
    logic [7:0]  dcb_maxl_hi;
    logic [7:0]  dcb_dev_type;
    logic [7:0]  dcb_node_type;
  } dcb_table_t;

  dcb_table_t dcb_table[15];

  /** PCB Commands *********************************************/
  localparam CMD_PCB_IDLE   = 8'h00;
  localparam CMD_PCB_SYNC1  = 8'h01;
  localparam CMD_PCB_SYNC2  = 8'h02;
  localparam CMD_PCB_SNA    = 8'h03;
  localparam CMD_PCB_RESET  = 8'h04;
  localparam CMD_PCB_WAIT   = 8'h05;

  /** DCB Commands *********************************************/
  localparam CMD_RESET      = 8'h00;
  localparam CMD_STATUS     = 8'h01;
  localparam CMD_ACK        = 8'h02;
  localparam CMD_CLEAR      = 8'h03;
  localparam CMD_RECEIVE    = 8'h04;
  localparam CMD_CANCEL     = 8'h05;
  localparam CMD_SEND       = 8'h06; /* + SIZE_HI + SIZE_LO + DATA + CRC */
  localparam CMD_NACK       = 8'h07;

  localparam CMD_SOFT_RESET = 8'h02;
  localparam CMD_WRITE      = 8'h03;
  localparam CMD_READ       = 8'h04;

  /** Response Codes *******************************************/
  localparam RSP_STATUS     = 8'h80; /* + SIZE_HI + SIZE_LO + TXCODE + STATUS + CRC */
  localparam RSP_ACK        = 8'h90;
  localparam RSP_CANCEL     = 8'hA0;
  localparam RSP_SEND       = 8'hB0; /* + SIZE_HI + SIZE_LO + DATA + CRC */
  localparam RSP_NACK       = 8'hC0;
  localparam RSP_BUSY       = 8'h9B;

  localparam CB_RANGE = PCB_SIZE + (15*DCB_SIZE);

  logic         relocating;   // PCB relocate (03h) in progress: MOVE_PCB keeps the DCB count
  logic [15:0]  pcb_base; /* Base of the PCB/ DCB in memory
                           * The size of the memory segment is
                           * pcb_base + PCB_SIZE + (15*DCB_SIZE) - 1
                           */
  logic [15:0]  pcb_addr;
  logic [15:0]  dcb_base; // precompute the dcb_base
  logic [15:0]  dcb_counter, data_counter;
  logic [3:0]   max_dcb;
  logic [2:0]   diskid;
  logic [1:0]   tapeid;
  logic [15:0]  dcb_base_cmd[15];
  logic [14:0]  dcb_cmd_hit;
  logic [3:0]   dcb_cmd_dev;
  logic         dcb_dev_hit_any;
  logic [14:0]  dcb_dev_hit;
  logic [3:0]   dcb_dev;
  // Disk access
  logic [15:0]  buffer;
  logic [15:0]  len;
  logic [31:0]  sec;
  logic         disk_req;   // Request a disk acccess
  logic         disk_rd;    // Read or ~write
  logic         disk_done;  // Disk transfer complete
  logic [3:0]   disk_dev;   // Device that is done
  logic         tape_req;
  logic [3:0]   done_dev;   // Device that is done
  logic [31:0]  disk_lastblock[4]; // Last block accessed by the disk
  logic         set_lastblock;
  logic [31:0]  tape_lastblock[4]; // Last block accessed by the disk
  logic         set_lastblock_tape;

  //
  logic         kbd_req;
  logic         kbd_done;
  logic         kbd_partial;  // completed a CMD_READ with fewer keys than asked
  logic [3:0]   kbd_dev;

  // Printer
  logic         prn_req;    // CMD_WRITE to the printer: start sending the buffer
  logic         prn_busy;   // that write has not finished yet
  logic         prn_done;   // the last byte has gone out
  logic [15:0]  prn_buf;
  logic [15:0]  prn_len;
  logic [3:0]   prn_dev;

  assign max_dcb = pcb_table.pcb_max_dcb;
  typedef enum bit
               {
                MOVE_PCB,
                IDLE
                } adam_state_t;

  adam_state_t adam_state, return_state, next_state;

  function logic [2:0] InterleaveTable(input logic [2:0] sector);
    case (sector)
      0: InterleaveTable = 0;
      1: InterleaveTable = 5;
      2: InterleaveTable = 2;
      3: InterleaveTable = 7;
      4: InterleaveTable = 4;
      5: InterleaveTable = 1;
      6: InterleaveTable = 6;
      7: InterleaveTable = 3;
    endcase // case (sector)
  endfunction

  bit [14:0] is_dsk[2];
  bit [14:0] is_kbd;
  bit [14:0] is_prn;
  bit [14:0] is_tap;
  bit [14:0] is_fuji;

  //--------------------------------------------------------------------------
  // FujiNet forwarding
  //
  // A DCB whose device FujiNet serves is not emulated here while fuji_en is
  // high: its command is queued for the AdamNet master, which runs the wire
  // transaction with FujiNet-PC, and the result lands back in the DCB the way
  // the master 6801 would write it. FujiNet-PC registers the printer (2), the
  // clock (3), disks 1-4 (4-7), two network adapters (9, A) and the Fuji
  // gateway (F) on an ADAM. A disk with an image mounted in the OSD stays
  // local, so FujiNet gets the empty slots. The keyboard and tapes stay here:
  // FujiNet has no tape device. (ADAMEm's bridge forwards the same set.)
  //
  // The job state machine below copies a write's bytes from ADAM RAM into
  // the master's buffer first, or a read's bytes out of it afterwards, in a
  // burst with the Z80 held off its bus, as the disk engine does. The DCB
  // status stays below 80h meanwhile, and EOS polls, as on an ADAM.
  //--------------------------------------------------------------------------
  logic [14:0] fuji_pending;      // DCBs with a command waiting for the wire
  logic [7:0]  fuji_cmd[15];      // and what it was
  logic [31:0] fuji_lastblock[4]; // FujiNet disks 1-4: the block last answered busy

  // The DCB the Z80 is writing: is it a FujiNet disk (4-7), and which block
  wire        fj_dcb_is_disk = ({dcb_table[dcb_dev].dcb_dev_num[3:0], dcb_table[dcb_dev].dcb_add_code[3:2]} == 6'b000001);
  wire [31:0] fj_dcb_block   = {dcb_table[dcb_dev].dcb_sec_num_3, dcb_table[dcb_dev].dcb_sec_num_2,
                                dcb_table[dcb_dev].dcb_sec_num_1, dcb_table[dcb_dev].dcb_sec_num_0};

  // job state machine -> the DCB block
  logic        fj_clear;          // drop fuji_pending[fj_dev]
  logic        fj_finish;         // write the result into DCB fj_dev
  logic [3:0]  fj_dev;            // the DCB being served
  logic [7:0]  fj_result;
  logic        fj_set_status;     // a STATUS answered: block size, type, node type
  logic [15:0] fj_maxl;
  logic [7:0]  fj_devtype;
  logic [7:0]  fj_node;
  logic        fj_set_len;        // a character read: how many bytes arrived
  logic [15:0] fj_len;

  // job state machine -> the RAM port
  logic        fj_bus_req;        // hold the Z80 off its bus
  logic        fj_ram_active;     // and drive the RAM port
  logic [15:0] fj_ram_addr;
  logic        fj_ram_wr;
  logic        fj_ram_rd;
  logic [7:0]  fj_wdata;
  logic        fuji_sel;

  // the master
  logic        fm_req;
  logic [2:0]  fm_op;
  logic [3:0]  fm_dev;
  logic [31:0] fm_block;
  logic [15:0] fm_len;
  logic        fm_patient;
  logic        fm_busy;
  logic        fm_done;
  logic [7:0]  fm_result;
  logic [15:0] fm_rsp_len;
  logic [7:0]  fm_rsp_devtype;
  logic [7:0]  fm_rsp_status;
  logic [9:0]  fm_buf_addr;
  logic        fm_buf_wr;
  logic [7:0]  fm_buf_din;
  logic [7:0]  fm_buf_q;

  always_ff @(posedge clk_i) begin
    dcb_cmd_hit     <= '0;
    dcb_cmd_dev     <= '0;
    dcb_dev         <= '0;
    dcb_dev_hit     <= '0;
    dcb_dev_hit_any <= '0;
    is_dsk          <= '{default: '0};
    is_kbd          <= '0;
    is_prn          <= '0;
    is_tap          <= '0;
    is_fuji         <= '0;
    diskid          <= '0;
    tapeid          <= '0;

    for (int i = 0; i < 15; i++) begin
      if ((z80_addr >= dcb_base_cmd[i]) &&
          (z80_addr < (dcb_base_cmd[i] + DCB_SIZE))  &&
          (i < max_dcb)) begin
        dcb_dev_hit_any <= '1;
        dcb_dev_hit[i]  <= '1;
        dcb_dev         <= i;
      end
      if ((z80_addr == dcb_base_cmd[i]) && (i < max_dcb)) begin
        dcb_cmd_hit[i] <= '1;
        dcb_cmd_dev    <= i;
        diskid         <= {dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} - 4;
        //tapeid         <= {dcb_table[i].dcb_dev_num[0], dcb_table[i].dcb_add_code[0]};
        tapeid         <= {dcb_table[i].dcb_add_code[0], dcb_table[i].dcb_dev_num[0]};
      end
      //if (i < max_dcb) begin
        is_dsk[0][i] <= ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h4) |
                       ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h5) |
                       ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h6) |
                       ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h7) & (i < max_dcb);
        is_dsk[1][i] <= ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h52) & (i < max_dcb);
        is_kbd[i] <= ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h1) & (i < max_dcb);
        is_prn[i] <= ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h2) & (i < max_dcb);
        is_tap[i] <= ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h8) |
                    ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h9) |
                    ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h18) |
                    ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h19) & (i < max_dcb);
        // FujiNet's devices: 2, 3, 9, A, F, and a disk 4-7 with no image mounted here
        is_fuji[i] <= fuji_en & (i < max_dcb) &
                      (({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h02) |
                       ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h03) |
                       ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h09) |
                       ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h0A) |
                       ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h0F) |
                       (({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:2]} == 6'b000001) &
                        ~disk_present[dcb_table[i].dcb_add_code[1:0]]));
      //end // if (i < pcb_table.pcb_max_dcb)
    end
  end

  logic [15:0] dcb_reg;

  assign pcb_base = {pcb_table.pcb_ba_hi, pcb_table.pcb_ba_lo};
  assign dcb_reg = z80_addr - dcb_base_cmd[dcb_dev];
  logic [7:0]  temp_reg;
  assign temp_reg = {dcb_table[0].dcb_dev_num[3:0], dcb_table[0].dcb_add_code[3:0]};
  always_ff @(posedge clk_i) begin
    // defaults
    adamnet_wait_n <= '0;
    kbd_status_upd <= '0;
    disk_req       <= '0;
    tape_req       <= '0;
    kbd_req        <= '0;
    prn_req        <= '0;

    for (int i = 0; i < 15; i++) begin
      dcb_base_cmd[i]    <= (pcb_base + PCB_SIZE) + i * DCB_SIZE;
    end

    case (adam_state)
      /** MovePCB() ************************************************/
      /** Move PCB and related DCBs to a new address.             **/
      /*************************************************************/
      // static void MovePCB(word NewAddr,byte MaxDCB)
      MOVE_PCB: begin
        $display("Adamnet (HDL): MovePCB Address: %x", pcb_addr);
        // A relocation keeps the DCB count and acknowledges; a reset starts with 15 DCBs
        pcb_table.pcb_cmd_stat <= relocating ? (RSP_STATUS | CMD_PCB_SNA) : CMD_PCB_IDLE;
        pcb_table.pcb_ba_lo    <= pcb_addr[7:0];
        pcb_table.pcb_ba_hi    <= pcb_addr[15:8];
        if (!relocating) pcb_table.pcb_max_dcb <= 15;
        relocating             <= '0;

        for (int i = 0; i < 15; i++) begin
          dcb_table[i].dcb_cmd_stat <= '0;
          dcb_table[i].dcb_dev_num  <= '0;
          dcb_table[i].dcb_add_code <= i;
        end
        for (int i = 0; i < 4; i++) begin
          disk_lastblock[i] <= '1;
          tape_lastblock[i] <= '1;
          fuji_lastblock[i] <= '1;
        end
        adam_state   <= IDLE;
      end // case: MOVE_PCB

      IDLE: begin
        //$display("IDLE");
        adamnet_wait_n       <= '1;
        if ((USE_REQ == 1) ? adamnet_ack_n : adamnet_wait_n) begin
          // Snoop PCB/ DCB Writes
          if ((z80_addr - pcb_base) < PCB_SIZE) begin
            case (z80_addr - pcb_base)
              PCB_CMD_STAT: begin
                if (z80_wr) begin
                  case (z80_data_wr)
                    CMD_PCB_SYNC1: begin
                      $display("Adamnet (HDL): SyncZ80: %x, %x",PCB_CMD_STAT,RSP_STATUS|CMD_PCB_SYNC1);
                      pcb_table.pcb_cmd_stat <= z80_data_wr | RSP_STATUS;
                    end
                    CMD_PCB_SYNC2: begin
                      $display("Adamnet (HDL): Sync6801: %x, %x",PCB_CMD_STAT,RSP_STATUS|CMD_PCB_SYNC2);
                      pcb_table.pcb_cmd_stat <= z80_data_wr | RSP_STATUS;
                    end
                    CMD_PCB_SNA: begin
                      // Relocate the PCB to the address just written and reset its DCBs, as
                      // ColEm's WritePCB does through MovePCB. This used to be a $finish stub.
                      $display("Adamnet (HDL): Relocate PCB to %x", pcb_addr);
                      relocating <= '1;
                      adam_state <= MOVE_PCB;
                    end
                    default: begin
                      // Software writes 00h and 80h+ here as plain status values; ColEm ignores
                      // them quietly. The byte is still kept, since it reads back like RAM.
                      if (z80_data_wr != 8'h00 && !z80_data_wr[7])
                        $display("Adamnet (HDL): Unimplemented PCB Operation %02x", z80_data_wr);
                      pcb_table.pcb_cmd_stat <= z80_data_wr;
                    end
                  endcase
                end
              end
              PCB_BA_LO: begin
                //if (z80_wr) pcb_table.pcb_ba_lo <= z80_data_wr;
                if (z80_wr) pcb_addr[7:0] <= z80_data_wr;
              end
              PCB_BA_HI: begin
                //if (z80_wr) pcb_table.pcb_ba_hi <= z80_data_wr;
                if (z80_wr) pcb_addr[15:8] <= z80_data_wr;
              end
              PCB_MAX_DCB: begin
                if (z80_wr) pcb_table.pcb_max_dcb <= z80_data_wr;
              end
              default: begin
              end
            endcase // case (z80_addr - pcb_base)
          end
          if (dcb_dev_hit_any) begin
            case (dcb_reg)// (pcb_base + PCB_SIZE + DCB_SIZE * dcb_dev))
              DCB_CMD_STAT: begin
                // A write that isn't a command (00h, or 80h+) must still read back, since the
                // DCB is RAM on an ADAM (ColEm keeps it in RAM too). Software clears a finished
                // command by writing 00h and waits for that before issuing the next one: the
                // Tape-Disk Verification cartridge's keyboard read never started without this.
                if (z80_wr && (z80_data_wr == 8'h00 || z80_data_wr[7]))
                  dcb_table[dcb_dev].dcb_cmd_stat <= z80_data_wr;
                if (|z80_data_wr[6:0] && ~z80_data_wr[7]) begin
                  if (is_fuji[dcb_dev]) begin
                    // FujiNet's device: queue the command for the wire and leave the
                    // DCB busy. The job state machine finishes it.
                    if (z80_wr) begin
                      case (z80_data_wr)
                        CMD_READ: begin
                          // A disk answers a block it has not been asked for just before
                          // with 9Bh, busy, and moves nothing; asking again gets the data.
                          // The local disks do the same (disk_lastblock, below), EOS
                          // retries on 9Bh, and ADAM CP/M's boot loader depends on it: it
                          // reads every block twice and fires a read-ahead into its
                          // buffer at EE3Bh after the last BIOS block, which would land
                          // on the BIOS it has just loaded there.
                          if (fj_dcb_is_disk &&
                              fj_dcb_block != fuji_lastblock[dcb_table[dcb_dev].dcb_add_code[1:0]]) begin
                            dcb_table[dcb_dev].dcb_cmd_stat <= RSP_BUSY;
                            fuji_lastblock[dcb_table[dcb_dev].dcb_add_code[1:0]] <= fj_dcb_block;
                          end else begin
                            dcb_table[dcb_dev].dcb_cmd_stat <= '0;
                            fuji_pending[dcb_dev]           <= '1;
                            fuji_cmd[dcb_dev]               <= z80_data_wr;
                            if (fj_dcb_is_disk)
                              fuji_lastblock[dcb_table[dcb_dev].dcb_add_code[1:0]] <= '1;
                          end
                        end
                        CMD_STATUS, CMD_SOFT_RESET, CMD_WRITE: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= '0;
                          fuji_pending[dcb_dev]           <= '1;
                          fuji_cmd[dcb_dev]               <= z80_data_wr;
                        end
                        default: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                        end
                      endcase
                    end
                  end else if (is_kbd[dcb_dev]) begin
`ifdef ADAMNET_TRACE
                    if (z80_wr) $display("Adamnet trace: KBD dcb %0d cmd %02x buf %02x%02x len %02x%02x",
                                         dcb_dev, z80_data_wr,
                                         dcb_table[dcb_dev].dcb_ba_hi, dcb_table[dcb_dev].dcb_ba_lo,
                                         dcb_table[dcb_dev].dcb_buf_len_hi, dcb_table[dcb_dev].dcb_buf_len_lo);
`endif
                    if (z80_rd_lvl) begin
                      kbd_status <= RSP_STATUS;
                      kbd_status_upd <= '1;
                      lastkey_out    <= '0;
                    end else if (z80_wr) begin
                      case (z80_data_wr)
                        CMD_STATUS, CMD_SOFT_RESET: begin
                          kbd_status     <= RSP_STATUS;
                          kbd_status_upd <= '1;
                          lastkey_out    <= '0;
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                          dcb_table[dcb_dev].dcb_maxl_lo  <= 8'h01;
                          dcb_table[dcb_dev].dcb_maxl_hi  <= 8'h00;
                          dcb_table[dcb_dev].dcb_dev_type <= '0;
                        end
                        CMD_WRITE: begin
                          kbd_status     <= RSP_STATUS;
                          kbd_status_upd <= '1;
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_BUSY; // RSP_ACK + 8'h0B;
                        end
                        CMD_READ: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= '0;
                          //A = GetDCBBase(Dev);
                          //N = GetDCBLen(Dev);
                          buffer <= {dcb_table[dcb_dev].dcb_ba_hi, dcb_table[dcb_dev].dcb_ba_lo};
                          len    <= {dcb_table[dcb_dev].dcb_buf_len_hi, dcb_table[dcb_dev].dcb_buf_len_lo};
                          kbd_req <= '1;
                          // FIXME!!!!!
                          //for(J=0 ; (J<N) && (V=GetKBD()) ; ++J, A=(A+1)&0xFFFF)
                          //  RAM(A) = V;
                          //KBDStatus = RSP_STATUS+(J<N? 0x0C:0x00);
                        end // case: CMD_READ
                        default: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                        end
                      endcase
                    end // if (z80_wr)
                  end else if (is_prn[dcb_dev]) begin
                    if (z80_wr) begin
                      case (z80_data_wr)
                        CMD_STATUS, CMD_SOFT_RESET: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                          dcb_table[dcb_dev].dcb_maxl_lo  <= 8'h01;
                          dcb_table[dcb_dev].dcb_maxl_hi  <= 8'h00;
                          dcb_table[dcb_dev].dcb_dev_type <= '0;
                        end
                        CMD_READ: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_BUSY; // RSP_ACK + 8'h0B;
                        end
                        CMD_WRITE: begin
                          // ColEm's UpdatePRN sends DCB length bytes from the DCB buffer. Here
                          // the status stays 00h (in progress) until the last byte has been
                          // taken, the way a disk transfer reports, so a full printer FIFO
                          // holds EOS off instead of losing characters.
                          dcb_table[dcb_dev].dcb_cmd_stat <= '0;
                          prn_req  <= '1;
                          prn_busy <= '1;
                          prn_buf  <= {dcb_table[dcb_dev].dcb_ba_hi, dcb_table[dcb_dev].dcb_ba_lo};
                          prn_len  <= {dcb_table[dcb_dev].dcb_buf_len_hi, dcb_table[dcb_dev].dcb_buf_len_lo};
                          prn_dev  <= dcb_dev;
`ifdef ADAMNET_TRACE
                          $display("Adamnet trace: PRN dcb %0d write buf %02x%02x len %02x%02x",
                                   dcb_dev, dcb_table[dcb_dev].dcb_ba_hi, dcb_table[dcb_dev].dcb_ba_lo,
                                   dcb_table[dcb_dev].dcb_buf_len_hi, dcb_table[dcb_dev].dcb_buf_len_lo);
`endif
                        end
                        default: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                        end
                      endcase
                    end else if (z80_rd_lvl && !prn_busy) begin // if (z80_wr)
                      dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                    end
                  end else if (is_dsk[0][dcb_dev]) begin
                    $display("Adamnet (HDL): UpdateDSK N %x Dev %x V %x",diskid,dcb_dev,z80_data_wr);
                    if (z80_rd_lvl && dcb_cmd_hit[dcb_dev]) begin
                      //dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                    end else if (z80_wr) begin
                      dcb_table[dcb_dev].dcb_node_type[3:0] <= disk_present[diskid] ? '0 : 4'h3;
                      case (z80_data_wr)
                        CMD_RESET: begin
                          disk_lastblock[diskid]          <= '1;
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                        end
                        CMD_STATUS: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                          dcb_table[dcb_dev].dcb_maxl_lo  <= 8'h00;
                          dcb_table[dcb_dev].dcb_maxl_hi  <= 8'h04;
                          dcb_table[dcb_dev].dcb_dev_type <= 8'h01;
                        end
                        CMD_SOFT_RESET: begin
                          $display("Adamnet (HDL): Disk %s: Soft reset",diskid+65);
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                        end
                        CMD_WRITE, CMD_READ: begin
                          buffer <= {dcb_table[dcb_dev].dcb_ba_hi, dcb_table[dcb_dev].dcb_ba_lo};
                          len    <= {dcb_table[dcb_dev].dcb_buf_len_hi, dcb_table[dcb_dev].dcb_buf_len_lo} < 16'h400 ?
                                    16'h400 : {dcb_table[dcb_dev].dcb_buf_len_hi, dcb_table[dcb_dev].dcb_buf_len_lo};
                          sec <= {dcb_table[dcb_dev].dcb_sec_num_3, dcb_table[dcb_dev].dcb_sec_num_2,
                                  dcb_table[dcb_dev].dcb_sec_num_1, dcb_table[dcb_dev].dcb_sec_num_0};
                          if (disk_present[diskid] &&
                              (({dcb_table[dcb_dev].dcb_sec_num_3, dcb_table[dcb_dev].dcb_sec_num_2,
                                  dcb_table[dcb_dev].dcb_sec_num_1, dcb_table[dcb_dev].dcb_sec_num_0} == disk_lastblock[diskid]) ||
                               (z80_data_wr == CMD_WRITE))) begin
                            dcb_table[dcb_dev].dcb_cmd_stat <= '0;
                            $display("Adamnet (HDL): Disk %s: %s %d bytes, sector 0x%X, memory 0x%04X",
                                     dcb_dev+65,z80_data_wr==CMD_READ? "Reading":"Writing",len,sec<<1,buffer);
                            disk_req                        <= '1;
                            disk_rd                         <= z80_data_wr == CMD_READ;
                            dcb_table[dcb_dev].dcb_cmd_stat <= '0;
                            disk_lastblock[diskid]          <= '1;
                          end else begin // if (disk_present[diskid] &&...
                            dcb_table[dcb_dev].dcb_cmd_stat <= RSP_BUSY;
                            disk_lastblock[diskid]          <= {dcb_table[dcb_dev].dcb_sec_num_3, dcb_table[dcb_dev].dcb_sec_num_2,
                                                                dcb_table[dcb_dev].dcb_sec_num_1, dcb_table[dcb_dev].dcb_sec_num_0};
                          end
                        end
                        default: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                        end
                      endcase
                    end
                  end else if (is_dsk[1][dcb_dev]) begin
                    $display("Adamnet (HDL): UpdateDSK N %x Dev %x V %x",4,dcb_dev,z80_data_wr);
                    if (z80_rd_lvl && dcb_cmd_hit[dcb_dev]) begin
                      dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                    end
                  end else if (is_tap[dcb_dev]) begin
                    $display("Adamnet (HDL): UpdateTAP N %x Dev %x V %x",tapeid,dcb_dev,z80_rd_lvl ? -1 : z80_data_wr);
                    if (z80_rd_lvl && dcb_cmd_hit[dcb_dev]) begin
                      //dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                    end else if (z80_wr) begin
                      if (tapeid < 2) begin
                        $display("SetDCB A ff56 Value %4h%4h, tape_present %h",
                                 (disk_present[NUM_DISKS+1] ? 4'h0 : 4'h3),
                                 (disk_present[NUM_DISKS+0] ? 4'h0 : 4'h3), disk_present);
                        dcb_table[dcb_dev].dcb_node_type[3:0] <= disk_present[NUM_DISKS+0] ? 4'h0 : 4'h3;
                        dcb_table[dcb_dev].dcb_node_type[7:4] <= disk_present[NUM_DISKS+1] ? 4'h0 : 4'h3;
                      end else begin
                        $display("SetDCB A ff56 Value %4h%4h, tape_present %h",
                                 (disk_present[NUM_DISKS+2] ? 4'h0 : 4'h3),
                                 (disk_present[NUM_DISKS+3] ? 4'h0 : 4'h3), disk_present);
                        dcb_table[dcb_dev].dcb_node_type[3:0] <= disk_present[NUM_DISKS+2] ? 4'h0 : 4'h3;
                        dcb_table[dcb_dev].dcb_node_type[7:4] <= disk_present[NUM_DISKS+3] ? 4'h0 : 4'h3;
                      end
                      case (z80_data_wr)
                        CMD_RESET: begin
                          tape_lastblock[tapeid]          <= '1;
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                        end
                        CMD_STATUS: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                          dcb_table[dcb_dev].dcb_maxl_lo  <= 8'h00;
                          dcb_table[dcb_dev].dcb_maxl_hi  <= 8'h04;
                          dcb_table[dcb_dev].dcb_dev_type <= 8'h01;
                        end
                        CMD_SOFT_RESET: begin
                          $display("Adamnet (HDL): Tape %s: Soft reset",tapeid+65);
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                        end
                        CMD_WRITE, CMD_READ: begin
                          buffer <= {dcb_table[dcb_dev].dcb_ba_hi, dcb_table[dcb_dev].dcb_ba_lo};
                          len    <= {dcb_table[dcb_dev].dcb_buf_len_hi, dcb_table[dcb_dev].dcb_buf_len_lo} < 16'h400 ?
                                    16'h400 : {dcb_table[dcb_dev].dcb_buf_len_hi, dcb_table[dcb_dev].dcb_buf_len_lo};
                          sec <= {dcb_table[dcb_dev].dcb_sec_num_3, dcb_table[dcb_dev].dcb_sec_num_2,
                                  dcb_table[dcb_dev].dcb_sec_num_1, dcb_table[dcb_dev].dcb_sec_num_0};
                          if (disk_present[NUM_DISKS+tapeid] &&
                              (({dcb_table[dcb_dev].dcb_sec_num_3, dcb_table[dcb_dev].dcb_sec_num_2,
                                  dcb_table[dcb_dev].dcb_sec_num_1, dcb_table[dcb_dev].dcb_sec_num_0} == tape_lastblock[tapeid]) ||
                               (z80_data_wr == CMD_WRITE))) begin
                            dcb_table[dcb_dev].dcb_cmd_stat <= '0;
                            $display("Adamnet (HDL): Tape %s: %s %d bytes, sector 0x%X, memory 0x%04X",
                                     dcb_dev+65,z80_data_wr==CMD_READ? "Reading":"Writing",len,sec<<1,buffer);
                            tape_req                        <= '1;
                            disk_rd                         <= z80_data_wr == CMD_READ;
                            tape_lastblock[tapeid]          <= '1;
                          end else begin // if (disk_present[diskid] &&...
                            dcb_table[dcb_dev].dcb_cmd_stat <= RSP_BUSY;
                            tape_lastblock[tapeid]          <= {dcb_table[dcb_dev].dcb_sec_num_3, dcb_table[dcb_dev].dcb_sec_num_2,
                                                                dcb_table[dcb_dev].dcb_sec_num_1, dcb_table[dcb_dev].dcb_sec_num_0};
                          end
                        end // case: CMD_WRITE, CMD_READ
                        default: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                        end
                      endcase
                    end // if (z80_wr)
                  end else if (dcb_cmd_hit[dcb_dev]) begin // if (is_tap[dcb_dev])
                    dcb_table[dcb_dev].dcb_cmd_stat <= RSP_BUSY; // RSP_ACK + 8'h0B;
                    $display("Adamnet (HDL): %s Unknown device #%d (dev_num %02x add_code %02x cmd %02x)",
                             z80_data_wr==CMD_READ? "Reading":"Writing", dcb_dev,
                             dcb_table[dcb_dev].dcb_dev_num, dcb_table[dcb_dev].dcb_add_code, z80_data_wr);
                  end
                end // if (|z80_data_wr[6:0] && ~z80_data_wr[7])
              end // case: DCB_CMD_STAT
              DCB_BA_LO: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_ba_lo <= z80_data_wr;
              end
              DCB_BA_HI: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_ba_hi <= z80_data_wr;
              end
              DCB_BUF_LEN_LO: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_buf_len_lo <= z80_data_wr;
              end
              DCB_BUF_LEN_HI: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_buf_len_hi <= z80_data_wr;
              end
              DCB_SEC_NUM_0: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_sec_num_0 <= z80_data_wr;
              end
              DCB_SEC_NUM_1: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_sec_num_1 <= z80_data_wr;
              end
              DCB_SEC_NUM_2: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_sec_num_2 <= z80_data_wr;
              end
              DCB_SEC_NUM_3: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_sec_num_3 <= z80_data_wr;
              end
              DCB_DEV_NUM: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_dev_num <= z80_data_wr;
              end
              DCB_RETRY_LO: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_retry_lo <= z80_data_wr;
              end
              DCB_RETRY_HI: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_retry_hi <= z80_data_wr;
              end
              DCB_ADD_CODE: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_add_code <= z80_data_wr;
              end
              DCB_MAXL_LO: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_maxl_lo <= z80_data_wr;
              end
              DCB_MAXL_HI: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_maxl_hi <= z80_data_wr;
              end
              DCB_DEV_TYPE: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_dev_type <= z80_data_wr;
              end
              DCB_NODE_TYPE: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_node_type <= z80_data_wr;
              end
              default: begin
              end
            endcase // case (z80_addr - DCB_SIZE * dcb_dev)
          end // if (dcb_dev_hit_any)
        end // if ((USE_REQ == 1) ? adamnet_ack_n : adamnet_wait_n)
      end // case: IDLE
    endcase // case (adam_state)

    // We don't handle the error
    if (disk_done) dcb_table[done_dev].dcb_cmd_stat <= RSP_STATUS;
    // short read (fewer keys than requested) reports RSP_STATUS + 0x0C, as ColEm does
    if (kbd_done)  dcb_table[kbd_dev].dcb_cmd_stat <= kbd_partial ? (RSP_STATUS + 8'h0C) : RSP_STATUS;
    if (prn_done) begin
      dcb_table[prn_dev].dcb_cmd_stat <= RSP_STATUS;
      prn_busy                        <= '0;
    end
    if (set_lastblock) begin
      disk_lastblock[done_dev] <= sec;
    end

    // A FujiNet job finished: the result into its DCB, as the master 6801 writes it.
    // A STATUS also brings the device's block size, type and node type (the
    // status byte of the wire packet, whose low nibble EOS reads as the media
    // state); a character read reports how many bytes actually came.
    if (fj_clear) fuji_pending[fj_dev] <= '0;
    if (fj_finish) begin
      dcb_table[fj_dev].dcb_cmd_stat <= fj_result;
      if (fj_set_status) begin
        dcb_table[fj_dev].dcb_maxl_lo   <= fj_maxl[7:0];
        dcb_table[fj_dev].dcb_maxl_hi   <= fj_maxl[15:8];
        dcb_table[fj_dev].dcb_dev_type  <= fj_devtype;
        // The whole status byte, as the master 6801 writes it: EOS reads the
        // low nibble as the media state (0 ok, 3 no media...; the high nibble
        // is the second tape's, eos52.asm A61989), and FujiNet's network devices
        // report their connection state in the high bits (programming guide,
        // "Network Device Status Codes").
        dcb_table[fj_dev].dcb_node_type <= fj_node;
      end
      if (fj_set_len) begin
        dcb_table[fj_dev].dcb_buf_len_lo <= fj_len[7:0];
        dcb_table[fj_dev].dcb_buf_len_hi <= fj_len[15:8];
      end
    end

    if (~adam_reset_pcb_n_i) begin
      // On reset setup PCB table
      pcb_addr       <= PCB_BASE_INIT;
      adam_state     <= MOVE_PCB;
      relocating     <= '0;
      adamnet_wait_n <= '1;
      kbd_status     <= RSP_STATUS;
      prn_busy       <= '0;
      fuji_pending   <= '0;
    end
  end

  initial begin
    // On reset setup PCB table
    pcb_addr       = PCB_BASE_INIT;
    adam_state     = MOVE_PCB;
    relocating     = '0;
    adamnet_req_n  = '1;
    adamnet_wait_n = '1;
    kbd_status     = RSP_STATUS;
    prn_busy       = '0;
  end

  logic pcb_hit;
  assign pcb_hit = (z80_addr - pcb_base) < PCB_SIZE;

  // Readback
  always_comb begin
    adamnet_dout = '0;
    adamnet_sel  = '0;
    if (pcb_hit) begin
      adamnet_sel  = '1;
      case (z80_addr - pcb_base)
        PCB_CMD_STAT: adamnet_dout = pcb_table.pcb_cmd_stat;
        PCB_BA_LO:    adamnet_dout = pcb_addr[7:0];
        PCB_BA_HI:    adamnet_dout = pcb_addr[15:8];
        PCB_MAX_DCB:  adamnet_dout = pcb_table.pcb_max_dcb;
        default:      adamnet_dout = '0;
      endcase // case (z80_addr - pcb_base)
    end else if (dcb_dev_hit_any) begin
      adamnet_sel  = '1;
      case (dcb_reg)
        DCB_CMD_STAT:   adamnet_dout = dcb_table[dcb_dev].dcb_cmd_stat;
        DCB_BA_LO:      adamnet_dout = dcb_table[dcb_dev].dcb_ba_lo;
        DCB_BA_HI:      adamnet_dout = dcb_table[dcb_dev].dcb_ba_hi;
        DCB_BUF_LEN_LO: adamnet_dout = dcb_table[dcb_dev].dcb_buf_len_lo;
        DCB_BUF_LEN_HI: adamnet_dout = dcb_table[dcb_dev].dcb_buf_len_hi;
        DCB_SEC_NUM_0:  adamnet_dout = dcb_table[dcb_dev].dcb_sec_num_0;
        DCB_SEC_NUM_1:  adamnet_dout = dcb_table[dcb_dev].dcb_sec_num_1;
        DCB_SEC_NUM_2:  adamnet_dout = dcb_table[dcb_dev].dcb_sec_num_2;
        DCB_SEC_NUM_3:  adamnet_dout = dcb_table[dcb_dev].dcb_sec_num_3;
        DCB_DEV_NUM:    adamnet_dout = dcb_table[dcb_dev].dcb_dev_num;
        DCB_RETRY_LO:   adamnet_dout = dcb_table[dcb_dev].dcb_retry_lo;
        DCB_RETRY_HI:   adamnet_dout = dcb_table[dcb_dev].dcb_retry_hi;
        DCB_ADD_CODE:   adamnet_dout = dcb_table[dcb_dev].dcb_add_code;
        DCB_MAXL_LO:    adamnet_dout = dcb_table[dcb_dev].dcb_maxl_lo;
        DCB_MAXL_HI:    adamnet_dout = dcb_table[dcb_dev].dcb_maxl_hi;
        DCB_DEV_TYPE:   adamnet_dout = dcb_table[dcb_dev].dcb_dev_type;
        DCB_NODE_TYPE:  adamnet_dout = dcb_table[dcb_dev].dcb_node_type;
        default:        adamnet_dout = '0;
      endcase // case (z80_addr - DCB_SIZE * dcb_dev)
    end
  end // always_comb

  logic [15:0] ram_buffer;
  logic [15:0] disk_len;
  logic [31:0] disk_sec;
  logic [15:0] int_ramb_addr[2];
  logic        int_ramb_wr[2];
  logic        int_ramb_rd[2];
  logic        kbd_sel;
  logic [7:0]  kbd_data;

  //assign ramb_dout    = clear_strobe ? code : disk_data;
  assign ramb_dout    = fuji_sel ? fj_wdata : kbd_sel ? kbd_data : disk_data[disk_dev];

  typedef enum bit [3:0]
               {
                DISK_IDLE,
                DISK_READ[4],
                DISK_WRITE[4]
                } disk_state_t;

  disk_state_t disk_state, tape_state;

  typedef enum bit [2:0]
               {
                KBD_IDLE,
                KBD_KEY,
                KBD_PAUSE
                } kbd_state_t;

  kbd_state_t kbd_state;

  // Printer fetch. It reads the DCB buffer through the same RAM port as the
  // disk and keyboard, but only while neither of them wants it: a disk or tape
  // block and a key delivery always win, and a byte whose read they interrupt
  // is simply read again. Port B of the RAM is registered, so the address has
  // to sit still for two clocks before ramb_din holds the byte.
  typedef enum bit [1:0]
               {
                PRN_IDLE,
                PRN_NEXT,
                PRN_FETCH1,
                PRN_FETCH2
                } prn_state_t;

  prn_state_t  prn_state;
  logic [15:0] prn_addr;
  logic [15:0] prn_left;
  logic        prn_port_free;

  logic        lastpress;
  logic        press_btn;
  logic [7:0]  code;
  logic [15:0] kbd_buffer;
  logic [15:0] kbd_len;
  logic [15:0] kbd_ramb_addr;
  logic [7:0]  key_code;
  logic [7:0]  shift_key_code;
  logic [7:0]  ctrl_key_code;
  logic        shift;
  logic        ctrl;
  logic        caps_lock;
  logic        osd;
  logic        watch_key;
  logic [22:0] key_rep_timer;
  logic        disk_active;

  // Keystroke FIFO.
  //
  // Keys used to be held in a single slot and, worse, the ASCII byte was decoded
  // combinationally from the LIVE ps2_key at the moment EOS got around to
  // consuming it -- by which time ps2_key usually held the key *release* event,
  // which decodes to 0x00 (^@). Any key pressed while the disk FSM was busy was
  // simply dropped, since the RAM port could only be stolen when disk_state was
  // DISK_IDLE. Now the byte is decoded at press time and queued, so it no longer
  // matters how long EOS takes to come collect it.
  localparam int KBD_FIFO_AW = 4;                  // 16 keys
  logic [7:0]              kbd_fifo[2**KBD_FIFO_AW];
  logic [KBD_FIFO_AW-1:0]  kbd_wp, kbd_rp;
  logic                    kbd_empty;
  logic                    kbd_full;
  logic [7:0]              kbd_byte;               // decoded at press time

  assign kbd_empty = (kbd_wp == kbd_rp);
  assign kbd_full  = ((kbd_wp + 1'b1) == kbd_rp);
  // the key as it reads *right now*, while ps2_key still holds the press
  assign kbd_byte  = ctrl ? ctrl_key_code : (shift | caps_lock) ? shift_key_code : key_code;

  initial begin
    disk_state = DISK_IDLE;
    tape_state = DISK_IDLE;
    kbd_state  = KBD_IDLE;
    prn_state  = PRN_IDLE;
  end

  assign watch_key = (kbd_state == KBD_KEY) && (disk_state == DISK_IDLE) && !kbd_empty && !fj_ram_active;
  assign prn_port_free = (disk_state == DISK_IDLE) && !disk_req && !tape_req && !watch_key && !fj_ram_active;

  assign disk_sector = disk_active ? {disk_sec[31:3], InterleaveTable(disk_sec[2:0])} : disk_sec;

  always_ff @(posedge clk_i) begin
    ramb_addr        <= watch_key ? kbd_buffer : int_ramb_addr[0];
    ramb_wr          <= watch_key ? '1 : int_ramb_wr[0];
    ramb_rd          <= watch_key ? '0 : int_ramb_rd[0];
    kbd_data         <= kbd_fifo[kbd_rp];   // key as decoded when it was pressed
    int_ramb_addr[1] <= watch_key ? kbd_buffer : int_ramb_addr[0];
    int_ramb_wr[1]   <= watch_key ? '1 : int_ramb_wr[0];
    int_ramb_rd[1]   <= watch_key ? '0: int_ramb_rd[0];
    int_ramb_wr[0]   <= '0;
    int_ramb_rd[0]   <= '0;
    disk_done        <= '0;
    disk_wr          <= '0;
    disk_flush       <= '0;
    kbd_done         <= '0;
    prn_strobe       <= '0;
    prn_done         <= '0;
    kbd_sel          <= watch_key;
    fuji_sel         <= fj_ram_active;
    adamnet_req_n    <= '1;
    set_lastblock    <= '0;

    //if (ramb_wr) $display("Writing to RAM %x: %x  kbd_data %x kbd_sel %x ps2_key %x key_code %c %x shift %x caps %x", ramb_addr, ramb_dout,kbd_data,kbd_sel,ps2_key[8:0],key_code,key_code,shift,caps_lock);
`ifdef SIM_DEBUG
    if (ramb_wr) $display("Writing to RAM %0x: %0x", ramb_addr, ramb_dout);
    if (ramb_rd_ack) $display("Reading from RAM %0x: %0x", ramb_addr, ramb_din);
    if (|disk_wr) $display("Writing to Disk sec %0x A %0x: %0x", disk_sec, disk_addr, disk_din);
`endif

    case (disk_state)
      DISK_IDLE: begin
        disk_active <= '0;
        if (disk_req) begin
          disk_active <= '1;
          ram_buffer  <= buffer;
          disk_len    <= len;
          dcb_counter <= len;
          disk_sec    <= sec<<1;
          disk_dev    <= diskid;
          done_dev    <= dcb_dev;
          disk_state  <= DISK_READ0;
        end else if (tape_req) begin
          ram_buffer  <= buffer;
          disk_len    <= len;
          dcb_counter <= len;
          disk_sec    <= sec<<1;
          disk_dev    <= NUM_DISKS+tapeid;
          done_dev    <= dcb_dev;
          // Tape blocks use the same states as disk blocks, without the disk's sector
          // interleave (disk_active stays low). The old tape-only copy of these states had
          // no write path: TAPE_WRITE0 was a $finish stub, so on hardware a tape save
          // never finished and AdamNet stopped serving disks and tapes until reset.
          disk_state  <= DISK_READ0;
        end
      end // case: DISK_IDLE
      DISK_READ0: begin
        ramb_addr   <= ram_buffer;
        ramb_rd     <= disk_rd;
        //disk_sector <= {disk_sec[31:3], InterleaveTable(disk_sec[2:0])};
        disk_load[disk_dev]   <= '1;
        adamnet_req_n <= '0;
        if (disk_sector_loaded[disk_dev]) begin
          $display("Adamnet (HDL): Disk %s: %s %d bytes, sector 0x%X, memory 0x%04X\n",
                   dcb_dev+65,disk_rd? "Reading":"Writing",dcb_counter,{disk_sec[31:3], InterleaveTable(disk_sec[2:0])},ram_buffer);
          int_ramb_addr[0]    <= ram_buffer - 1'b1; // We will advance automatically in next state
          disk_load[disk_dev] <= '0;
          data_counter        <= '0;
          disk_state          <= disk_rd ? DISK_READ1 : DISK_WRITE0;
        end
      end // case: DISK_READ0
      DISK_READ1: begin
        adamnet_req_n <= '0;
        if (~adamnet_ack_n || USE_REQ == 0) disk_state   <= disk_rd ? DISK_READ2 : DISK_WRITE0;
      end
      DISK_READ2: begin
        adamnet_req_n <= '0;
        // Read up to 512 bytes (might be less)
        disk_addr <= data_counter;
        if (data_counter < dcb_counter && data_counter < 16'h200) begin
          // We are within the sector
          int_ramb_addr[0]    <= int_ramb_addr[0] + 1'b1;
          int_ramb_wr[0]      <= '1;
          data_counter <= data_counter + 1'b1;
          ram_buffer   <= ram_buffer + 1'b1;
        end else if (data_counter < dcb_counter) begin
          // We are leaving the sector, but we have more data to read.
          // Flush the current sector
          disk_wr[disk_dev] <= '0;
          disk_flush[disk_dev]   <= '1;
          data_counter <= '0;
          disk_sec     <= disk_sec + 1'b1; // Advance for next sector
          dcb_counter  <= dcb_counter - 16'h200;
          disk_state   <= DISK_READ3;
        end else begin
          // Done reading
          disk_wr[disk_dev]    <= '0;
          disk_flush[disk_dev] <= '1;
          disk_state <= DISK_IDLE;
          disk_done  <= '1;
          adamnet_req_n <= '1;
        end // else: !if(data_counter < dcb_counter)
      end // case: DISK_READ1
      DISK_READ3: begin
        adamnet_req_n <= '0;
        if (~disk_sector_loaded[disk_dev]) disk_state <= DISK_READ0;
      end
      DISK_WRITE0: begin
`ifdef SIM_DEBUG
        $display("write0 %0h, %0h", ramb_addr, dcb_counter);
`endif
        adamnet_req_n <= '0;
        ramb_addr     <= ramb_addr + 1'b1;
        ramb_rd       <= '1;
        disk_state    <= DISK_WRITE1;
      end
      DISK_WRITE1: begin
        ramb_addr    <= ramb_addr;
        int_ramb_addr[0]    <= int_ramb_addr[0];
`ifdef SIM_DEBUG
        $display("write0 %0h, %0h", ramb_addr, dcb_counter);
`endif
        // Write up to 512 bytes (might be less)
        disk_addr         <= data_counter;
        disk_din          <= ramb_din;
        if (data_counter < dcb_counter && data_counter < 16'h200) begin
          // We are within the sector
          disk_wr[disk_dev] <= '1;
          ramb_addr    <= ramb_addr + 1'b1;
          int_ramb_addr[0]    <= int_ramb_addr[0] + 1'b1;
          ramb_rd      <= '1;
          data_counter <= data_counter + 1'b1;
          ram_buffer   <= ram_buffer + 1'b1;
        end else if (data_counter < dcb_counter) begin
          // We are leaving the sector, but we have more data to read.
          // Flush the current sector
          disk_flush[disk_dev]   <= '1;
          disk_state   <= DISK_WRITE2;
        end else begin
          // Done reading
          disk_flush[disk_dev] <= '1;
          disk_state <= DISK_IDLE;
          disk_done  <= '1;
          adamnet_req_n <= '1;
        end // else: !if(data_counter < dcb_counter)
      end // case: DISK_READ1
      DISK_WRITE2: begin
        ramb_addr    <= ramb_addr;
        int_ramb_addr[0]    <= int_ramb_addr[0];
        // only our own drive's flush completion may advance the sector: disk_flushed
        // is a per-drive vector and reduction-OR let any other drive (or a read
        // completion) step disk_sec early.
        if (disk_flushed[disk_dev]) begin
          ramb_addr    <= ramb_addr - 1'b1;
          data_counter <= '0;
          disk_sec     <= disk_sec + 1'b1; // Advance for next sector
          dcb_counter  <= dcb_counter - 16'h200;
          disk_state   <= DISK_WRITE3;
        end
      end
      DISK_WRITE3: begin
`ifdef SIM_DEBUG
        $display("write0 %0h, %0h", ramb_addr, dcb_counter);
`endif
        ramb_addr     <= ramb_addr + 1'b1;
        ramb_rd       <= '1;
        disk_state    <= DISK_WRITE1;
      end
      default: begin
      end
    endcase

    // Printer: comes after the disk case so its RAM address overrides the idle default
    if (prn_req) begin
      prn_addr  <= prn_buf;
      prn_left  <= prn_len;
      prn_state <= PRN_NEXT;
    end else begin
      case (prn_state)
        PRN_NEXT: begin
          if (prn_left == 0) begin
            prn_done  <= '1;
            prn_state <= PRN_IDLE;
          end else if (prn_port_free && prn_ready) begin
            ramb_addr <= prn_addr;
            prn_state <= PRN_FETCH1;
          end
        end
        PRN_FETCH1: begin
          if (prn_port_free) begin
            ramb_addr <= prn_addr;
            prn_state <= PRN_FETCH2;
          end else begin
            prn_state <= PRN_NEXT;
          end
        end
        PRN_FETCH2: begin
          if (prn_port_free) begin
            ramb_addr  <= prn_addr;
            prn_data   <= ramb_din;
            prn_strobe <= '1;
            prn_addr   <= prn_addr + 1'b1;
            prn_left   <= prn_left - 1'b1;
`ifdef ADAMNET_TRACE
            $display("Adamnet trace: PRN byte %02x from %04x", ramb_din, prn_addr);
`endif
          end
          prn_state <= PRN_NEXT;
        end
        default: begin
        end
      endcase
    end

    // FujiNet copy: last, so it overrides the idle default while it holds the port.
    // It only starts while the disk, keyboard and printer engines are idle, and
    // they stay idle while fj_ram_active is high (watch_key, prn_port_free).
    if (fj_ram_active) begin
      ramb_addr <= fj_ram_addr;
      ramb_wr   <= fj_ram_wr;
      ramb_rd   <= fj_ram_rd;
    end
    if (fj_bus_req) adamnet_req_n <= '0;

    lastpress <= ps2_key[10];
    if(lastpress != ps2_key[10]) begin
      if      (ps2_key[8:0] == 9'h014) ctrl  <= ps2_key[9]; //CTRL
      else if (ps2_key[8:0] == 9'h012) shift <= ps2_key[9]; //LEFT SHIFT
      else if (ps2_key[8:0] == 9'h058) begin if (ps2_key[9]) caps_lock<=~caps_lock; end  //CAPSLOCK
      else if (ps2_key[8:0] == 9'h059) shift <= ps2_key[9]; //RIGHT SHIFT
                else if (ps2_key[8:0] == 9'h007) osd <= ps2_key[9]; //OSD ignore key

      else begin
        press_btn     <= ps2_key[9];
        code          <= ps2_key[7:0];
        key_rep_timer <= 'd7000000;
        // Queue the key NOW, while ps2_key still holds the press and the
        // modifiers still reflect what was held down with it.
        if (ps2_key[9] && !kbd_full) begin
`ifdef ADAMNET_TRACE
          $display("Adamnet trace: KBD queued %02x", kbd_byte);
`endif
          kbd_fifo[kbd_wp] <= kbd_byte;
          kbd_wp           <= kbd_wp + 1'b1;
        end
      end
    end else begin
        key_rep_timer <= key_rep_timer-1;
        if (key_rep_timer==0)
        begin
           key_rep_timer <= 933333;
           // Auto-repeat, but only once the queue has drained. The repeat rate is
           // ~45/s; queueing every tick would let a single held key flood the FIFO
           // and duplicate characters (it was harmless when repeats merely
           // overwrote the old single key slot).
           if (press_btn && kbd_empty) begin
             kbd_fifo[kbd_wp] <= kbd_byte;
             kbd_wp           <= kbd_wp + 1'b1;
           end
        end
    end

    case (kbd_state)
      KBD_IDLE: begin
        if (kbd_req) begin
`ifdef ADAMNET_TRACE
          $display("Adamnet trace: KBD read dcb %0d buf %04x len %0d queued %0d",
                   dcb_dev, buffer, len, kbd_wp - kbd_rp);
`endif
          kbd_buffer <= buffer;
          kbd_len    <= len;
          kbd_state  <= KBD_KEY;
          kbd_dev    <= dcb_dev;
        end
      end
      KBD_KEY: begin
        // A keyboard CMD_READ must NEVER block. ColEm (AdamNet.c UpdateKBD) is:
        //
        //   for(J=0 ; (J<N) && (V=GetKBD()) ; ++J, ...) RAM(A) = V;
        //   KBDStatus = RSP_STATUS + (J<N ? 0x0C : 0x00);
        //
        // i.e. copy however many keys are actually queued (possibly none) and
        // finish, flagging a short read with +0x0C. We used to sit here until we
        // had delivered exactly `len` keys. When EOS polled an empty keyboard it
        // got no completion, gave up, and moved on -- but we stayed in KBD_KEY
        // holding that dead request's kbd_buffer/kbd_len. The next keypress was
        // then written into the ABANDONED DCB and completed it, so EOS's next
        // read never saw the key. That is the swallowed first keystroke.
        if (kbd_len == 0) begin
          // asked for nothing / already gave everything
          kbd_state   <= KBD_IDLE;
          kbd_done    <= '1;
          kbd_partial <= '0;
        end else if (!watch_key) begin
          // Can't deliver right now -- FIFO empty, or the disk FSM owns the RAM
          // port. Either way finish short rather than hold the request open; the
          // key stays queued and EOS picks it up on its next poll.
          kbd_state   <= KBD_IDLE;
          kbd_done    <= '1;
          kbd_partial <= '1;
`ifdef ADAMNET_TRACE
          $display("Adamnet trace: KBD short read, queued %0d, disk_state %0d", kbd_wp - kbd_rp, disk_state);
`endif
        end else begin
          // watch_key = EOS wants a key, disk FSM is idle, FIFO has one.
          // The RAM write is issued by the muxes above.
          kbd_rp     <= kbd_rp + 1'b1;
`ifdef ADAMNET_TRACE
          $display("Adamnet trace: KBD deliver %02x to %04x", kbd_data, kbd_buffer);
`endif
          kbd_buffer <= kbd_buffer + 1'b1;
          kbd_len    <= kbd_len - 1'b1;

          if (kbd_len == 1) begin
            kbd_state   <= KBD_IDLE;
            kbd_done    <= '1;
            kbd_partial <= '0;      // delivered the full N
          end else begin
            kbd_state <= KBD_PAUSE;
          end
        end
      end // case: KBD_KEY
      KBD_PAUSE: kbd_state  <= KBD_KEY;
      default: begin
      end
    endcase // case (kbd_state)

  end

  //--------------------------------------------------------------------------
  // FujiNet: the wire master, and the job state machine that feeds it
  //--------------------------------------------------------------------------
  cv_adamnet_master #(.CLK_HZ(CLK_HZ)) fuji_master
    (
     .clk(clk_i),
     .reset(~adam_reset_pcb_n_i),
     .enable(fuji_en),
     .req(fm_req),
     .req_op(fm_op),
     .req_dev(fm_dev),
     .req_block(fm_block),
     .req_len(fm_len),
     .req_patient(fm_patient),
     .busy(fm_busy),
     .done(fm_done),
     .result(fm_result),
     .rsp_len(fm_rsp_len),
     .rsp_devtype(fm_rsp_devtype),
     .rsp_status(fm_rsp_status),
     .buf_addr(fm_buf_addr),
     .buf_wr(fm_buf_wr),
     .buf_din(fm_buf_din),
     .buf_q(fm_buf_q),
     .tx_data(fuji_tx_data),
     .tx_wr(fuji_tx_wr),
     .tx_busy(fuji_tx_busy),
     .rx_data(fuji_rx_data),
     .rx_avail(fuji_rx_avail),
     .rx_rd(fuji_rx_rd),
     .rx_flush(fuji_rx_flush)
     );

  typedef enum logic [3:0] {
    FJ_IDLE, FJ_AUX, FJ_PICK, FJ_DISPATCH, FJ_FETCH_REQ, FJ_FETCH,
    FJ_START, FJ_WAIT, FJ_STORE_REQ, FJ_STORE, FJ_FINISH
  } fj_state_t;

  fj_state_t   fj_state;
  logic        fj_reset_pend;    // after a reset: MN_RESET to FujiNet's devices...
  logic        fj_probe_pend;    // ...then a patient STATUS to the gateway, so a daemon
                                 // still starting is there before EOS scans for devices
  logic        fuji_online;      // the last probe was answered
  logic [7:0]  fj_cmd;
  logic [7:0]  fj_devid;
  logic [15:0] fj_buf;
  logic [15:0] fj_dlen;
  logic [31:0] fj_blk;
  logic        fj_isblk;
  logic [15:0] fj_n;             // bytes to copy
  logic [15:0] fj_i;
  logic [2:0]  fj_phase;
  logic        engines_idle;
  logic [15:0] fj_n_calc;

  assign engines_idle = (disk_state == DISK_IDLE) && !disk_req && !tape_req &&
                        !watch_key && (prn_state == PRN_IDLE);
  // The DCB block clears fuji_pending a clock after fj_clear, so the job just
  // finished still shows as pending in the clock FJ_IDLE looks: mask it out, or
  // every command would run twice.
  logic [14:0] fuji_waiting;
  always_comb begin
    fuji_waiting = fuji_pending;
    if (fj_clear) fuji_waiting[fj_dev] = 1'b0;
  end
  // a block device always moves 1024 bytes; a character device what the DCB says, capped
  assign fj_n_calc = fj_isblk ? 16'd1024 : (fj_dlen > 16'd1024) ? 16'd1024 : fj_dlen;

  always_ff @(posedge clk_i) begin
    fm_req     <= '0;
    fuji_boot_cfg_ack <= '0;
    fm_buf_wr  <= '0;
    fj_ram_wr  <= '0;
    fj_ram_rd  <= '0;
    fj_clear   <= '0;
    fj_finish  <= '0;

    case (fj_state)
      FJ_IDLE: begin
        fj_ram_active <= '0;
        fj_bus_req    <= '0;
        if (!fuji_en) begin
          fj_reset_pend <= '0;
          fj_probe_pend <= '0;
          fuji_online   <= '0;
        end
        if (fuji_en && fj_reset_pend) begin
          fm_op         <= FOP_RESET_ALL;
          fm_dev        <= 4'hF;
          fm_patient    <= '0;
          fm_req        <= '1;
          fj_reset_pend <= '0;
          fj_state      <= FJ_AUX;
        end else if (fuji_en && fj_probe_pend) begin
          fm_op         <= FOP_STATUS;
          fm_dev        <= 4'hF;
          fm_patient    <= '1;
          fm_req        <= '1;
          fj_probe_pend <= '0;
          fj_state      <= FJ_AUX;
        end else if (fuji_en && fuji_boot_cfg && fuji_online && !fuji_boot_cfg_ack) begin
          // After the probe and before any DCB: EOS's first look at disk 1
          // waits in the queue until CONFIG is back in FujiNet's slot 1.
          fm_op         <= FOP_BOOT_CONFIG;
          fm_dev        <= 4'hF;
          fm_patient    <= '0;
          fm_req        <= '1;
          fj_state      <= FJ_AUX;
        end else if (|fuji_waiting) begin
          // the lowest numbered DCB waiting (the loop's last assignment wins)
          for (int i = 14; i >= 0; i--) if (fuji_waiting[i]) fj_dev <= i;
          fj_state <= FJ_PICK;
        end
      end

      // a reset broadcast or the probe: nothing to write back
      FJ_AUX: begin
        if (fm_done) begin
          if (fm_op == FOP_BOOT_CONFIG) begin
            fuji_boot_cfg_ack <= '1;      // done or refused: either way, don't retry
            $display("Adamnet (HDL): FujiNet boot CONFIG -> %02x", fm_result);
          end
          if (fm_op == FOP_STATUS) begin
            fuji_online <= (fm_result == FRES_OK);
            $display("Adamnet (HDL): FujiNet gateway %s", fm_result == FRES_OK ? "answered" : "did not answer");
          end
          fj_state <= FJ_IDLE;
        end
      end

      FJ_PICK: begin
`ifdef ADAMNET_TRACE
        $display("Adamnet trace: FujiNet pick dcb %0d, pending %015b", fj_dev, fuji_pending);
`endif
        fj_cmd   <= fuji_cmd[fj_dev];
        fj_devid <= {dcb_table[fj_dev].dcb_dev_num[3:0], dcb_table[fj_dev].dcb_add_code[3:0]};
        fj_buf   <= {dcb_table[fj_dev].dcb_ba_hi, dcb_table[fj_dev].dcb_ba_lo};
        fj_dlen  <= {dcb_table[fj_dev].dcb_buf_len_hi, dcb_table[fj_dev].dcb_buf_len_lo};
        fj_blk   <= {dcb_table[fj_dev].dcb_sec_num_3, dcb_table[fj_dev].dcb_sec_num_2,
                     dcb_table[fj_dev].dcb_sec_num_1, dcb_table[fj_dev].dcb_sec_num_0};
        fj_isblk <= ({dcb_table[fj_dev].dcb_dev_num[3:0], dcb_table[fj_dev].dcb_add_code[3:2]} == 6'b000001);
        fj_state <= FJ_DISPATCH;
      end

      FJ_DISPATCH: begin
        $display("Adamnet (HDL): FujiNet dev %02x cmd %02x buf %04x len %04x block %0d",
                 fj_devid, fj_cmd, fj_buf, fj_dlen, fj_blk);
        fj_n      <= fj_n_calc;
        fj_result <= FRES_OK;
        fj_i      <= '0;
        fj_phase  <= '0;
        if (!fuji_en) begin
          // the link went away with this queued
          fj_result <= FRES_TIMEOUT;
          fj_state  <= FJ_FINISH;
        end else begin
          case (fj_cmd)
            CMD_STATUS: begin
              fm_op    <= FOP_STATUS;
              fj_state <= FJ_START;
            end
            CMD_SOFT_RESET: begin
              fm_op    <= FOP_RESET;
              fj_state <= FJ_START;
            end
            CMD_WRITE: begin
              fm_op    <= fj_isblk ? FOP_WRITE_BLOCK : FOP_CHAR_WRITE;
              fj_state <= (fj_n_calc == 0) ? FJ_START : FJ_FETCH_REQ;
            end
            CMD_READ: begin
              fm_op    <= fj_isblk ? FOP_READ_BLOCK : FOP_CHAR_READ;
              fj_state <= FJ_START;
            end
            default: fj_state <= FJ_FINISH;
          endcase
        end
      end

      // Take the bus, once the other engines are quiet, for a copy
      FJ_FETCH_REQ, FJ_STORE_REQ: begin
        fj_bus_req <= '1;
        if ((~adamnet_ack_n || USE_REQ == 0) && engines_idle) begin
          fj_ram_active <= '1;
          fj_i          <= '0;
          fj_phase      <= '0;
          fj_state      <= (fj_state == FJ_FETCH_REQ) ? FJ_FETCH : FJ_STORE;
        end
      end

      // ADAM RAM -> the master's buffer, one byte per 6 clocks. The address is
      // registered on its way out of this module and the RAM's read port is
      // registered too, so ramb_din holds the byte three clocks after the address.
      FJ_FETCH: begin
        fj_phase <= fj_phase + 1'b1;
        case (fj_phase)
          3'd0: begin
            fj_ram_addr <= fj_buf + fj_i;
            fj_ram_rd   <= '1;
          end
          3'd4: begin
            fm_buf_addr <= fj_i[9:0];
            fm_buf_din  <= ramb_din;
            fm_buf_wr   <= '1;
          end
          3'd5: begin
            fj_phase <= '0;
            fj_i     <= fj_i + 1'b1;
            if (fj_i + 1'b1 == fj_n) begin
              fj_ram_active <= '0;
              fj_bus_req    <= '0;
              fj_state      <= FJ_START;
            end
          end
          default: begin
          end
        endcase
      end

      FJ_START: begin
        fm_dev     <= fj_devid[3:0];
        fm_block   <= fj_blk;
        fm_len     <= fj_n;         // bytes to send, or room for a reply
        fm_patient <= '0;
        fm_req     <= '1;
        fj_state   <= FJ_WAIT;
      end

      FJ_WAIT: begin
        if (fm_done) begin
          fj_result <= fm_result;
          if (fj_cmd == CMD_READ) begin
            if (fm_result == FRES_OK && fm_rsp_len != 0) begin
              fj_n     <= (fm_rsp_len > fj_n) ? fj_n : fm_rsp_len;
              fj_state <= FJ_STORE_REQ;
            end else begin
              fj_n     <= '0;
              fj_state <= FJ_FINISH;
            end
          end else begin
            fj_state <= FJ_FINISH;
          end
        end
      end

      // The master's buffer -> ADAM RAM, one byte per 8 clocks. The buffer reads
      // back a clock after its address; the write strobe and data go out through
      // the registered port two clocks later and must sit still around it.
      FJ_STORE: begin
        fj_phase <= fj_phase + 1'b1;
        case (fj_phase)
          3'd0: fm_buf_addr <= fj_i[9:0];
          3'd2: begin
            fj_wdata    <= fm_buf_q;
            fj_ram_addr <= fj_buf + fj_i;
          end
          3'd4: fj_ram_wr <= '1;
          3'd7: begin
            fj_i <= fj_i + 1'b1;
            if (fj_i + 1'b1 == fj_n) begin
              fj_ram_active <= '0;
              fj_bus_req    <= '0;
              fj_state      <= FJ_FINISH;
            end
          end
          default: begin
          end
        endcase
      end

      FJ_FINISH: begin
        fj_finish     <= '1;
        fj_clear      <= '1;
        fj_set_status <= (fj_cmd == CMD_STATUS) && (fj_result == FRES_OK);
        fj_maxl       <= fm_rsp_len;
        fj_devtype    <= fm_rsp_devtype;
        fj_node       <= fm_rsp_status;
        fj_set_len    <= (fj_cmd == CMD_READ) && !fj_isblk && (fj_result == FRES_OK);
        fj_len        <= fj_n;
        $display("Adamnet (HDL): FujiNet dev %02x cmd %02x -> %02x, %0d bytes", fj_devid, fj_cmd, fj_result, fj_n);
        fj_state      <= FJ_IDLE;
      end

      default: fj_state <= FJ_IDLE;
    endcase

    if (~adam_reset_pcb_n_i) begin
      fj_state      <= FJ_IDLE;
      fj_ram_active <= '0;
      fj_bus_req    <= '0;
      fj_reset_pend <= '1;
      fj_probe_pend <= '1;
    end
  end

  initial begin
    fj_state      = FJ_IDLE;
    fj_ram_active = '0;
    fj_bus_req    = '0;
    fj_reset_pend = '0;
    fj_probe_pend = '0;
    fuji_online   = '0;
    fuji_pending  = '0;
  end

always @(*) begin
  key_code = 'h000;
        case (ps2_key[8:0])
        9'h000 : key_code = 'h00;
        9'h001 : key_code = 'h87;	//F9
        9'h002 : key_code = 'h87;
        9'h003 : key_code = 'h85;	//F5
        9'h004 : key_code = 'h83;	//F3
        9'h005 : key_code = 'h81;	//F1
        9'h006 : key_code = 'h82;	//F2
        9'h007 : key_code = 'h87;	//F12 <OSD>
        9'h008 : key_code = 'h87;
        9'h009 : key_code = 'h87;	//F10
        9'h00a : key_code = 'h90;	//F8 = WILD CARD
        9'h00b : key_code = 'h86;	//F6
        9'h00c : key_code = 'h84;	//F4
        9'h00d : key_code = 'h09;	//TAB
        9'h00e : key_code = 'h7E;	//~ (`)
        9'h00f : key_code = 'h87;
        9'h010 : key_code = 'h87;
        9'h011 : key_code = 'h6f;	//LEFT ALT (command)
          //9'h012 : key_code = 'h87;      //key_code = 'h071;	//LEFT SHIFT
        9'h013 : key_code = 'h87;
          //9'h014 : key_code = 'h87;	//CTRL (not mapped)
        9'h015 : key_code = 'h71;	//q
        9'h016 : key_code = 'h31;	//1
        9'h017 : key_code = 'h87;
        9'h018 : key_code = 'h87;
        9'h019 : key_code = 'h87;
        9'h01a : key_code = 'h7a;	//z
        9'h01b : key_code = 'h73;	//s
        9'h01c : key_code = 'h61;	//a
        9'h01d : key_code = 'h77;	//w
        9'h01e : key_code = 'h32;	//2
        9'h01f : key_code = 'h87;
        9'h020 : key_code = 'h87;
        9'h021 : key_code = 'h63;	//c
        9'h022 : key_code = 'h78;	//x
        9'h023 : key_code = 'h64;	//d
        9'h024 : key_code = 'h65;	//e
        9'h025 : key_code = 'h34;	//4
        9'h026 : key_code = 'h33;	//3
        9'h027 : key_code = 'h87;
        9'h028 : key_code = 'h87;
        9'h029 : key_code = 'h20;	//SPACE
        9'h02a : key_code = 'h76;	//v
        9'h02b : key_code = 'h66;	//f
        9'h02c : key_code = 'h74;	//t
        9'h02d : key_code = 'h72;	//r
        9'h02e : key_code = 'h35;	//5
        9'h02f : key_code = 'h87;
        9'h030 : key_code = 'h87;
        9'h031 : key_code = 'h6e;	//n
        9'h032 : key_code = 'h62;	//b
        9'h033 : key_code = 'h68;	//h
        9'h034 : key_code = 'h67;	//g
        9'h035 : key_code = 'h79;	//y
        9'h036 : key_code = 'h36;	//6
        9'h037 : key_code = 'h87;
        9'h038 : key_code = 'h87;
        9'h039 : key_code = 'h87;
        9'h03a : key_code = 'h6d;	//m
        9'h03b : key_code = 'h6a;	//j
        9'h03c : key_code = 'h75;	//u
        9'h03d : key_code = 'h37;	//7
        9'h03e : key_code = 'h38;	//8
        9'h03f : key_code = 'h87;
        9'h040 : key_code = 'h87;
        9'h041 : key_code = 'h2c;	//<,
        9'h042 : key_code = 'h6b;	//k
        9'h043 : key_code = 'h69;	//i
        9'h044 : key_code = 'h6f;	//o
        9'h045 : key_code = 'h30;	//0
        9'h046 : key_code = 'h39;	//9
        9'h047 : key_code = 'h87;
        9'h048 : key_code = 'h87;
        9'h049 : key_code = 'h2e;	//>.
        9'h04a : key_code = 'h2f;	//FORWARD SLASH
        9'h04b : key_code = 'h6c;	//l
        9'h04c : key_code = 'h3b;	//;
        9'h04d : key_code = 'h70;	//p
        9'h04e : key_code = 'h2d;	//-
        9'h04f : key_code = 'h87;
        9'h050 : key_code = 'h87;
        9'h051 : key_code = 'h87;
        9'h052 : key_code = 'h27;	//'"
        9'h053 : key_code = 'h87;
        9'h054 : key_code = 'h5b;	//[
        9'h055 : key_code = 'h3d;	// =
        9'h056 : key_code = 'h87;
        9'h057 : key_code = 'h87;
          //9'h058 : key_code = 'h87;      //CAPSLOCK
          //9'h059 : key_code = 'h87;      //key_code = 'h071;	//RIGHT SHIFT
        9'h05a : key_code = 'h0d;	//ENTER
        9'h05b : key_code = 'h5d;	//]
        9'h05c : key_code = 'h87;
        9'h05d : key_code = 'h5c;	//BACKSLASH
        9'h05e : key_code = 'h87;
        9'h05f : key_code = 'h87;
        9'h060 : key_code = 'h87;
        9'h061 : key_code = 'h71;	//international left shift cut out (German '<>' key), 0x56 Set#1 code
        9'h062 : key_code = 'h87;
        9'h063 : key_code = 'h87;
        9'h064 : key_code = 'h87;
        9'h065 : key_code = 'h87;
        9'h066 : key_code = 'h08;	//BACKSPACE
        9'h067 : key_code = 'h87;
        9'h068 : key_code = 'h87;
        9'h069 : key_code = 'h27;	//KP 1
        9'h06a : key_code = 'h87;
        9'h06b : key_code = 'h2d;	//KP 4
        9'h06c : key_code = 'h33;	//KP 7
        9'h06d : key_code = 'h87;
        9'h06e : key_code = 'h87;
        9'h06f : key_code = 'h87;
        9'h070 : key_code = 'h25;	//KP 0
        9'h071 : key_code = 'h03;	//KP .
        9'h072 : key_code = 'h29;	//KP 2
        9'h073 : key_code = 'h2f;	//KP 5
        9'h074 : key_code = 'h31;	//KP 6
        9'h075 : key_code = 'h37;	//KP 8
        9'h076 : key_code = 'h1b;	//ESCAPE
        9'h077 : key_code = 'h87;	//NUMLOCK (Mac keypad clear?)
        9'h078 : key_code = 'h87;	//F11 <OSD>
        9'h079 : key_code = 'h0d;	//KP +
        9'h07a : key_code = 'h2b;	//KP 3
        9'h07b : key_code = 'h1d;	//KP -
        9'h07c : key_code = 'h05;	//KP *
        9'h07d : key_code = 'h39;	//KP 9
        9'h07e : key_code = 'h87;	//SCROLL LOCK / KP )
        9'h07f : key_code = 'h87;
        9'h080 : key_code = 'h87;
        9'h081 : key_code = 'h87;
        9'h082 : key_code = 'h87;
        9'h083 : key_code = 'h91;	//F7 = UNDO
        9'h084 : key_code = 'h87;
        9'h085 : key_code = 'h87;
        9'h086 : key_code = 'h87;
        9'h087 : key_code = 'h87;
        9'h088 : key_code = 'h87;
        9'h089 : key_code = 'h87;
        9'h08a : key_code = 'h87;
        9'h08b : key_code = 'h87;
        9'h08c : key_code = 'h87;
        9'h08d : key_code = 'h87;
        9'h08e : key_code = 'h87;
        9'h08f : key_code = 'h87;
        9'h090 : key_code = 'h87;
        9'h091 : key_code = 'h87;
        9'h092 : key_code = 'h87;
        9'h093 : key_code = 'h87;
        9'h094 : key_code = 'h87;
        9'h095 : key_code = 'h87;
        9'h096 : key_code = 'h87;
        9'h097 : key_code = 'h87;
        9'h098 : key_code = 'h87;
        9'h099 : key_code = 'h87;
        9'h09a : key_code = 'h87;
        9'h09b : key_code = 'h87;
        9'h09c : key_code = 'h87;
        9'h09d : key_code = 'h87;
        9'h09e : key_code = 'h87;
        9'h09f : key_code = 'h87;
        9'h0a0 : key_code = 'h87;
        9'h0a1 : key_code = 'h87;
        9'h0a2 : key_code = 'h87;
        9'h0a3 : key_code = 'h87;
        9'h0a4 : key_code = 'h87;
        9'h0a5 : key_code = 'h87;
        9'h0a6 : key_code = 'h87;
        9'h0a7 : key_code = 'h87;
        9'h0a8 : key_code = 'h87;
        9'h0a9 : key_code = 'h87;
        9'h0aa : key_code = 'h87;
        9'h0ab : key_code = 'h87;
        9'h0ac : key_code = 'h87;
        9'h0ad : key_code = 'h87;
        9'h0ae : key_code = 'h87;
        9'h0af : key_code = 'h87;
        9'h0b0 : key_code = 'h87;
        9'h0b1 : key_code = 'h87;
        9'h0b2 : key_code = 'h87;
        9'h0b3 : key_code = 'h87;
        9'h0b4 : key_code = 'h87;
        9'h0b5 : key_code = 'h87;
        9'h0b6 : key_code = 'h87;
        9'h0b7 : key_code = 'h87;
        9'h0b8 : key_code = 'h87;
        9'h0b9 : key_code = 'h87;
        9'h0ba : key_code = 'h87;
        9'h0bb : key_code = 'h87;
        9'h0bc : key_code = 'h87;
        9'h0bd : key_code = 'h87;
        9'h0be : key_code = 'h87;
        9'h0bf : key_code = 'h87;
        9'h0c0 : key_code = 'h87;
        9'h0c1 : key_code = 'h87;
        9'h0c2 : key_code = 'h87;
        9'h0c3 : key_code = 'h87;
        9'h0c4 : key_code = 'h87;
        9'h0c5 : key_code = 'h87;
        9'h0c6 : key_code = 'h87;
        9'h0c7 : key_code = 'h087;
        9'h0c8 : key_code = 'h087;
        9'h0c9 : key_code = 'h087;
        9'h0ca : key_code = 'h087;
        9'h0cb : key_code = 'h087;
        9'h0cc : key_code = 'h087;
        9'h0cd : key_code = 'h087;
        9'h0ce : key_code = 'h087;
        9'h0cf : key_code = 'h087;
        9'h0d0 : key_code = 'h087;
        9'h0d1 : key_code = 'h087;
        9'h0d2 : key_code = 'h087;
        9'h0d3 : key_code = 'h087;
        9'h0d4 : key_code = 'h087;
        9'h0d5 : key_code = 'h087;
        9'h0d6 : key_code = 'h087;
        9'h0d7 : key_code = 'h087;
        9'h0d8 : key_code = 'h087;
        9'h0d9 : key_code = 'h087;
        9'h0da : key_code = 'h087;
        9'h0db : key_code = 'h087;
        9'h0dc : key_code = 'h087;
        9'h0dd : key_code = 'h087;
        9'h0de : key_code = 'h087;
        9'h0df : key_code = 'h087;
        9'h0e0 : key_code = 'h087;	//ps2 extended key
        9'h0e1 : key_code = 'h087;
        9'h0e2 : key_code = 'h087;
        9'h0e3 : key_code = 'h087;
        9'h0e4 : key_code = 'h087;
        9'h0e5 : key_code = 'h087;
        9'h0e6 : key_code = 'h087;
        9'h0e7 : key_code = 'h087;
        9'h0e8 : key_code = 'h087;
        9'h0e9 : key_code = 'h087;
        9'h0ea : key_code = 'h087;
        9'h0eb : key_code = 'h087;
        9'h0ec : key_code = 'h087;
        9'h0ed : key_code = 'h087;
        9'h0ee : key_code = 'h087;
        9'h0ef : key_code = 'h087;
        9'h0f0 : key_code = 'h087;	//ps2 release code
        9'h0f1 : key_code = 'h087;
        9'h0f2 : key_code = 'h087;
        9'h0f3 : key_code = 'h087;
        9'h0f4 : key_code = 'h087;
        9'h0f5 : key_code = 'h087;
        9'h0f6 : key_code = 'h087;
        9'h0f7 : key_code = 'h087;
        9'h0f8 : key_code = 'h087;
        9'h0f9 : key_code = 'h087;
        9'h0fa : key_code = 'h087;	//ps2 ack code
        9'h0fb : key_code = 'h087;
        9'h0fc : key_code = 'h087;
        9'h0fd : key_code = 'h087;
        9'h0fe : key_code = 'h087;
        9'h0ff : key_code = 'h087;
        9'h100 : key_code = 'h087;
        9'h101 : key_code = 'h087;
        9'h102 : key_code = 'h087;
        9'h103 : key_code = 'h087;
        9'h104 : key_code = 'h087;
        9'h105 : key_code = 'h087;
        9'h106 : key_code = 'h087;
        9'h107 : key_code = 'h087;
        9'h108 : key_code = 'h087;
        9'h109 : key_code = 'h087;
        9'h10a : key_code = 'h087;
        9'h10b : key_code = 'h087;
        9'h10c : key_code = 'h087;
        9'h10d : key_code = 'h087;
        9'h10e : key_code = 'h087;
        9'h10f : key_code = 'h087;
        9'h110 : key_code = 'h087;
        9'h111 : key_code = 'h06f;	//RIGHT ALT (command)
        9'h112 : key_code = 'h087;
        9'h113 : key_code = 'h087;
        9'h114 : key_code = 'h087;
        9'h115 : key_code = 'h087;
        9'h116 : key_code = 'h087;
        9'h117 : key_code = 'h087;
        9'h118 : key_code = 'h087;
        9'h119 : key_code = 'h087;
        9'h11a : key_code = 'h087;
        9'h11b : key_code = 'h087;
        9'h11c : key_code = 'h087;
        9'h11d : key_code = 'h087;
        9'h11e : key_code = 'h087;
        9'h11f : key_code = 'h075;	//WINDOWS OR APPLICATION KEY (option)
        9'h120 : key_code = 'h087;
        9'h121 : key_code = 'h087;
        9'h122 : key_code = 'h087;
        9'h123 : key_code = 'h087;
        9'h124 : key_code = 'h087;
        9'h125 : key_code = 'h087;
        9'h126 : key_code = 'h087;
        9'h127 : key_code = 'h87;
        9'h128 : key_code = 'h87;
        9'h129 : key_code = 'h87;
        9'h12a : key_code = 'h87;
        9'h12b : key_code = 'h87;
        9'h12c : key_code = 'h87;
        9'h12d : key_code = 'h87;
        9'h12e : key_code = 'h87;
        9'h12f : key_code = 'h87;
        9'h130 : key_code = 'h87;
        9'h131 : key_code = 'h87;
        9'h132 : key_code = 'h87;
        9'h133 : key_code = 'h87;
        9'h134 : key_code = 'h87;
        9'h135 : key_code = 'h87;
        9'h136 : key_code = 'h87;
        9'h137 : key_code = 'h87;
        9'h138 : key_code = 'h87;
        9'h139 : key_code = 'h87;
        9'h13a : key_code = 'h87;
        9'h13b : key_code = 'h87;
        9'h13c : key_code = 'h87;
        9'h13d : key_code = 'h87;
        9'h13e : key_code = 'h87;
        9'h13f : key_code = 'h87;
        9'h140 : key_code = 'h87;
        9'h141 : key_code = 'h87;
        9'h142 : key_code = 'h87;
        9'h143 : key_code = 'h87;
        9'h144 : key_code = 'h87;
        9'h145 : key_code = 'h87;
        9'h146 : key_code = 'h87;
        9'h147 : key_code = 'h87;
        9'h148 : key_code = 'h87;
        9'h149 : key_code = 'h87;
        9'h14a : key_code = 'h1b;	//KP /
        9'h14b : key_code = 'h87;
        9'h14c : key_code = 'h87;
        9'h14d : key_code = 'h87;
        9'h14e : key_code = 'h87;
        9'h14f : key_code = 'h87;
        9'h150 : key_code = 'h87;
        9'h151 : key_code = 'h87;
        9'h152 : key_code = 'h87;
        9'h153 : key_code = 'h87;
        9'h154 : key_code = 'h87;
        9'h155 : key_code = 'h87;
        9'h156 : key_code = 'h87;
        9'h157 : key_code = 'h87;
        9'h158 : key_code = 'h87;
        9'h159 : key_code = 'h87;
        9'h15a : key_code = 'h19;	//KP ENTER
        9'h15b : key_code = 'h87;
        9'h15c : key_code = 'h87;
        9'h15d : key_code = 'h87;
        9'h15e : key_code = 'h87;
        9'h15f : key_code = 'h87;
        9'h160 : key_code = 'h87;
        9'h161 : key_code = 'h87;
        9'h162 : key_code = 'h87;
        9'h163 : key_code = 'h87;
        9'h164 : key_code = 'h87;
        9'h165 : key_code = 'h87;
        9'h166 : key_code = 'h87;
        9'h167 : key_code = 'h87;
        9'h168 : key_code = 'h87;
        9'h169 : key_code = 'h96;	//END = CLEAR
        9'h16a : key_code = 'h87;
        9'h16b : key_code = 'hA3;	//ARROW LEFT
        9'h16c : key_code = 'h80;	//HOME
        9'h16d : key_code = 'h87;
        9'h16e : key_code = 'h87;
        9'h16f : key_code = 'h87;
        9'h170 : key_code = 'h94;	//INSERT
        9'h171 : key_code = 'h97;	//DELETE (+CTRL = 7f)
        9'h172 : key_code = 'hA2;	//ARROW DOWN
        9'h173 : key_code = 'h87;
        9'h174 : key_code = 'hA1;	//ARROW RIGHT
        9'h175 : key_code = 'hA0;	//ARROW UP
        9'h176 : key_code = 'h87;
        9'h177 : key_code = 'h87;
        9'h178 : key_code = 'h87;
        9'h179 : key_code = 'h87;
        9'h17a : key_code = 'h93;	//PGDN = STORE / FETCH
        9'h17b : key_code = 'h87;
        9'h17c : key_code = 'h95;	//PRTSCR = PRINT
        9'h17d : key_code = 'h92;	//PGUP = MOVE / COPY
        9'h17e : key_code = 'h87;	//ctrl+break
        9'h17f : key_code = 'h87;
        9'h180 : key_code = 'h87;
        9'h181 : key_code = 'h87;
        9'h182 : key_code = 'h87;
        9'h183 : key_code = 'h87;
        9'h184 : key_code = 'h87;
        9'h185 : key_code = 'h87;
        9'h186 : key_code = 'h87;
        9'h187 : key_code = 'h87;
        9'h188 : key_code = 'h87;
        9'h189 : key_code = 'h87;
        9'h18a : key_code = 'h87;
        9'h18b : key_code = 'h87;
        9'h18c : key_code = 'h87;
        9'h18d : key_code = 'h87;
        9'h18e : key_code = 'h87;
        9'h18f : key_code = 'h87;
        9'h190 : key_code = 'h87;
        9'h191 : key_code = 'h87;
        9'h192 : key_code = 'h87;
        9'h193 : key_code = 'h87;
        9'h194 : key_code = 'h87;
        9'h195 : key_code = 'h87;
        9'h196 : key_code = 'h87;
        9'h197 : key_code = 'h87;
        9'h198 : key_code = 'h87;
        9'h199 : key_code = 'h87;
        9'h19a : key_code = 'h87;
        9'h19b : key_code = 'h87;
        9'h19c : key_code = 'h87;
        9'h19d : key_code = 'h87;
        9'h19e : key_code = 'h87;
        9'h19f : key_code = 'h87;
        9'h1a0 : key_code = 'h87;
        9'h1a1 : key_code = 'h87;
        9'h1a2 : key_code = 'h87;
        9'h1a3 : key_code = 'h87;
        9'h1a4 : key_code = 'h87;
        9'h1a5 : key_code = 'h87;
        9'h1a6 : key_code = 'h87;
        9'h1a7 : key_code = 'h87;
        9'h1a8 : key_code = 'h87;
        9'h1a9 : key_code = 'h87;
        9'h1aa : key_code = 'h87;
        9'h1ab : key_code = 'h87;
        9'h1ac : key_code = 'h87;
        9'h1ad : key_code = 'h87;
        9'h1ae : key_code = 'h87;
        9'h1af : key_code = 'h87;
        9'h1b0 : key_code = 'h87;
        9'h1b1 : key_code = 'h87;
        9'h1b2 : key_code = 'h87;
        9'h1b3 : key_code = 'h87;
        9'h1b4 : key_code = 'h87;
        9'h1b5 : key_code = 'h87;
        9'h1b6 : key_code = 'h87;
        9'h1b7 : key_code = 'h87;
        9'h1b8 : key_code = 'h87;
        9'h1b9 : key_code = 'h87;
        9'h1ba : key_code = 'h87;
        9'h1bb : key_code = 'h87;
        9'h1bc : key_code = 'h87;
        9'h1bd : key_code = 'h87;
        9'h1be : key_code = 'h87;
        9'h1bf : key_code = 'h87;
        9'h1c0 : key_code = 'h87;
        9'h1c1 : key_code = 'h87;
        9'h1c2 : key_code = 'h87;
        9'h1c3 : key_code = 'h87;
        9'h1c4 : key_code = 'h87;
        9'h1c5 : key_code = 'h87;
        9'h1c6 : key_code = 'h87;
        9'h1c7 : key_code = 'h87;
        9'h1c8 : key_code = 'h87;
        9'h1c9 : key_code = 'h87;
        9'h1ca : key_code = 'h87;
        9'h1cb : key_code = 'h87;
        9'h1cc : key_code = 'h87;
        9'h1cd : key_code = 'h87;
        9'h1ce : key_code = 'h87;
        9'h1cf : key_code = 'h87;
        9'h1d0 : key_code = 'h87;
        9'h1d1 : key_code = 'h87;
        9'h1d2 : key_code = 'h87;
        9'h1d3 : key_code = 'h87;
        9'h1d4 : key_code = 'h87;
        9'h1d5 : key_code = 'h87;
        9'h1d6 : key_code = 'h87;
        9'h1d7 : key_code = 'h87;
        9'h1d8 : key_code = 'h87;
        9'h1d9 : key_code = 'h87;
        9'h1da : key_code = 'h87;
        9'h1db : key_code = 'h87;
        9'h1dc : key_code = 'h87;
        9'h1dd : key_code = 'h87;
        9'h1de : key_code = 'h87;
        9'h1df : key_code = 'h87;
        9'h1e0 : key_code = 'h87;	//ps2 extended key(duplicate, see $e0)
        9'h1e1 : key_code = 'h87;
        9'h1e2 : key_code = 'h87;
        9'h1e3 : key_code = 'h87;
        9'h1e4 : key_code = 'h87;
        9'h1e5 : key_code = 'h87;
        9'h1e6 : key_code = 'h87;
        9'h1e7 : key_code = 'h87;
        9'h1e8 : key_code = 'h87;
        9'h1e9 : key_code = 'h87;
        9'h1ea : key_code = 'h87;
        9'h1eb : key_code = 'h87;
        9'h1ec : key_code = 'h87;
        9'h1ed : key_code = 'h87;
        9'h1ee : key_code = 'h87;
        9'h1ef : key_code = 'h87;
        9'h1f0 : key_code = 'h87;	//ps2 release code(duplicate, see $f0)
        9'h1f1 : key_code = 'h87;
        9'h1f2 : key_code = 'h87;
        9'h1f3 : key_code = 'h87;
        9'h1f4 : key_code = 'h87;
        9'h1f5 : key_code = 'h87;
        9'h1f6 : key_code = 'h87;
        9'h1f7 : key_code = 'h87;
        9'h1f8 : key_code = 'h87;
        9'h1f9 : key_code = 'h87;
        9'h1fa : key_code = 'h87;	//ps2 ack code(duplicate see $fa)
        9'h1fb : key_code = 'h87;
        9'h1fc : key_code = 'h87;
        9'h1fd : key_code = 'h87;
        9'h1fe : key_code = 'h87;
        9'h1ff : key_code = 'h87;
        default: key_code = 'h000;
    endcase
end


always @(*) begin
  shift_key_code = '0;
case (key_code)
    8'h00 : shift_key_code = 'h00;
    8'h01 : shift_key_code = 'h01;
    8'h02 : shift_key_code = 'h02;
    8'h03 : shift_key_code = 'h03;
    8'h04 : shift_key_code = 'h04;
    8'h05 : shift_key_code = 'h05;
    8'h06 : shift_key_code = 'h06;
    8'h07 : shift_key_code = 'h07;
    8'h08 : shift_key_code = 'hb8;
    8'h09 : shift_key_code = 'hb9;
    8'h0a : shift_key_code = 'h0a;
    8'h0b : shift_key_code = 'h0b;
    8'h0c : shift_key_code = 'h0c;
    8'h0d : shift_key_code = 'h0d;
    8'h0e : shift_key_code = 'h0e;
    8'h0f : shift_key_code = 'h0f;
    8'h10 : shift_key_code = 'h10;
    8'h11 : shift_key_code = 'h11;
    8'h12 : shift_key_code = 'h12;
    8'h13 : shift_key_code = 'h13;
    8'h14 : shift_key_code = 'h14;
    8'h15 : shift_key_code = 'h15;
    8'h16 : shift_key_code = 'h16;
    8'h17 : shift_key_code = 'h17;
    8'h18 : shift_key_code = 'h18;
    8'h19 : shift_key_code = 'h19;
    8'h1a : shift_key_code = 'h1a;
    8'h1b : shift_key_code = 'h1b;
    8'h1c : shift_key_code = 'h1c;
    8'h1d : shift_key_code = 'h1d;
    8'h1e : shift_key_code = 'h1e;
    8'h1f : shift_key_code = 'h1f;
    8'h20 : shift_key_code = 'h20;
    8'h21 : shift_key_code = 'h21;
    8'h22 : shift_key_code = 'h22;
    8'h23 : shift_key_code = 'h23;
    8'h24 : shift_key_code = 'h24;
    8'h25 : shift_key_code = 'h25;
    8'h26 : shift_key_code = 'h26;
    8'h27 : shift_key_code = 'h22;
    8'h28 : shift_key_code = 'h28;
    8'h29 : shift_key_code = 'h29;
    8'h2a : shift_key_code = 'h2a;
    8'h2b : shift_key_code = 'h2b;
    8'h2c : shift_key_code = 'h3c;
    8'h2d : shift_key_code = 'h5f;
    8'h2e : shift_key_code = 'h3e;
    8'h2f : shift_key_code = 'h3f;
    8'h30 : shift_key_code = 'h29;
    8'h31 : shift_key_code = 'h21;
    8'h32 : shift_key_code = 'h40;
    8'h33 : shift_key_code = 'h23;
    8'h34 : shift_key_code = 'h24;
    8'h35 : shift_key_code = 'h25;
    8'h36 : shift_key_code = 'h5e;
    8'h37 : shift_key_code = 'h26;
    8'h38 : shift_key_code = 'h2a;
    8'h39 : shift_key_code = 'h28;
    8'h3a : shift_key_code = 'h3a;
    8'h3b : shift_key_code = 'h3a;
    8'h3c : shift_key_code = 'h3c;
    8'h3d : shift_key_code = 'h2b;
    8'h3e : shift_key_code = 'h3e;
    8'h3f : shift_key_code = 'h3f;
    8'h40 : shift_key_code = 'h40;
    8'h41 : shift_key_code = 'h61;
    8'h42 : shift_key_code = 'h62;
    8'h43 : shift_key_code = 'h63;
    8'h44 : shift_key_code = 'h64;
    8'h45 : shift_key_code = 'h65;
    8'h46 : shift_key_code = 'h66;
    8'h47 : shift_key_code = 'h67;
    8'h48 : shift_key_code = 'h68;
    8'h49 : shift_key_code = 'h69;
    8'h4a : shift_key_code = 'h6a;
    8'h4b : shift_key_code = 'h6b;
    8'h4c : shift_key_code = 'h6c;
    8'h4d : shift_key_code = 'h6d;
    8'h4e : shift_key_code = 'h6e;
    8'h4f : shift_key_code = 'h6f;
    8'h50 : shift_key_code = 'h70;
    8'h51 : shift_key_code = 'h71;
    8'h52 : shift_key_code = 'h72;
    8'h53 : shift_key_code = 'h73;
    8'h54 : shift_key_code = 'h74;
    8'h55 : shift_key_code = 'h75;
    8'h56 : shift_key_code = 'h76;
    8'h57 : shift_key_code = 'h77;
    8'h58 : shift_key_code = 'h78;
    8'h59 : shift_key_code = 'h79;
    8'h5a : shift_key_code = 'h7a;
    8'h5b : shift_key_code = 'h7b;
    8'h5c : shift_key_code = 'h7c;
    8'h5d : shift_key_code = 'h7d;
    8'h5e : shift_key_code = 'h5e;
    8'h5f : shift_key_code = 'h5f;
    8'h60 : shift_key_code = 'h7e;
    8'h61 : shift_key_code = 'h41;
    8'h62 : shift_key_code = 'h42;
    8'h63 : shift_key_code = 'h43;
    8'h64 : shift_key_code = 'h44;
    8'h65 : shift_key_code = 'h45;
    8'h66 : shift_key_code = 'h46;
    8'h67 : shift_key_code = 'h47;
    8'h68 : shift_key_code = 'h48;
    8'h69 : shift_key_code = 'h49;
    8'h6a : shift_key_code = 'h4a;
    8'h6b : shift_key_code = 'h4b;
    8'h6c : shift_key_code = 'h4c;
    8'h6d : shift_key_code = 'h4d;
    8'h6e : shift_key_code = 'h4e;
    8'h6f : shift_key_code = 'h4f;
    8'h70 : shift_key_code = 'h50;
    8'h71 : shift_key_code = 'h51;
    8'h72 : shift_key_code = 'h52;
    8'h73 : shift_key_code = 'h53;
    8'h74 : shift_key_code = 'h54;
    8'h75 : shift_key_code = 'h55;
    8'h76 : shift_key_code = 'h56;
    8'h77 : shift_key_code = 'h57;
    8'h78 : shift_key_code = 'h58;
    8'h79 : shift_key_code = 'h59;
    8'h7a : shift_key_code = 'h5a;
    8'h7b : shift_key_code = 'h7b;
    8'h7c : shift_key_code = 'h7c;
    8'h7d : shift_key_code = 'h7d;
    8'h7e : shift_key_code = 'h7e;
    8'h7f : shift_key_code = 'h7f;
    8'h80 : shift_key_code = 'h80;
    8'h81 : shift_key_code = 'h89;
    8'h82 : shift_key_code = 'h8a;
    8'h83 : shift_key_code = 'h8b;
    8'h84 : shift_key_code = 'h8c;
    8'h85 : shift_key_code = 'h8d;
    8'h86 : shift_key_code = 'h8e;
    8'h87 : shift_key_code = 'h87;
    8'h88 : shift_key_code = 'h88;
    8'h89 : shift_key_code = 'h89;
    8'h8a : shift_key_code = 'h8a;
    8'h8b : shift_key_code = 'h8b;
    8'h8c : shift_key_code = 'h8c;
    8'h8d : shift_key_code = 'h8d;
    8'h8e : shift_key_code = 'h8e;
    8'h8f : shift_key_code = 'h8f;
    8'h90 : shift_key_code = 'h98;
    8'h91 : shift_key_code = 'h99;
    8'h92 : shift_key_code = 'h9a;
    8'h93 : shift_key_code = 'h9b;
    8'h94 : shift_key_code = 'h9c;
    8'h95 : shift_key_code = 'h9d;
    8'h96 : shift_key_code = 'h9e;
    8'h97 : shift_key_code = 'h9f;
    8'h98 : shift_key_code = 'h98;
    8'h99 : shift_key_code = 'h99;
    8'h9a : shift_key_code = 'h9a;
    8'h9b : shift_key_code = 'h9b;
    8'h9c : shift_key_code = 'h9c;
    8'h9d : shift_key_code = 'h9d;
    8'h9e : shift_key_code = 'h9e;
    8'h9f : shift_key_code = 'h9f;
    8'ha0 : shift_key_code = 'ha0;
    8'ha1 : shift_key_code = 'ha1;
    8'ha2 : shift_key_code = 'ha2;
    8'ha3 : shift_key_code = 'ha3;
    8'ha4 : shift_key_code = 'ha4;
    8'ha5 : shift_key_code = 'ha5;
    8'ha6 : shift_key_code = 'ha6;
    8'ha7 : shift_key_code = 'ha7;
    8'ha8 : shift_key_code = 'ha8;
    8'ha9 : shift_key_code = 'ha9;
    8'haa : shift_key_code = 'haa;
    8'hab : shift_key_code = 'hab;
    8'hac : shift_key_code = 'hac;
    8'had : shift_key_code = 'had;
    8'hae : shift_key_code = 'hae;
    8'haf : shift_key_code = 'haf;
    8'hb0 : shift_key_code = 'hb0;
    8'hb1 : shift_key_code = 'hb1;
    8'hb2 : shift_key_code = 'hb2;
    8'hb3 : shift_key_code = 'hb3;
    8'hb4 : shift_key_code = 'hb4;
    8'hb5 : shift_key_code = 'hb5;
    8'hb6 : shift_key_code = 'hb6;
    8'hb7 : shift_key_code = 'hb7;
    8'hb8 : shift_key_code = 'hb8;
    8'hb9 : shift_key_code = 'hb9;
    8'hba : shift_key_code = 'hba;
    8'hbb : shift_key_code = 'hbb;
    8'hbc : shift_key_code = 'hbc;
    8'hbd : shift_key_code = 'hbd;
    8'hbe : shift_key_code = 'hbe;
    8'hbf : shift_key_code = 'hbf;
    8'hc0 : shift_key_code = 'hc0;
    8'hc1 : shift_key_code = 'hc1;
    8'hc2 : shift_key_code = 'hc2;
    8'hc3 : shift_key_code = 'hc3;
    8'hc4 : shift_key_code = 'hc4;
    8'hc5 : shift_key_code = 'hc5;
    8'hc6 : shift_key_code = 'hc6;
    8'hc7 : shift_key_code = 'hc7;
    8'hc8 : shift_key_code = 'hc8;
    8'hc9 : shift_key_code = 'hc9;
    8'hca : shift_key_code = 'hca;
    8'hcb : shift_key_code = 'hcb;
    8'hcc : shift_key_code = 'hcc;
    8'hcd : shift_key_code = 'hcd;
    8'hce : shift_key_code = 'hce;
    8'hcf : shift_key_code = 'hcf;
    8'hd0 : shift_key_code = 'hd0;
    8'hd1 : shift_key_code = 'hd1;
    8'hd2 : shift_key_code = 'hd2;
    8'hd3 : shift_key_code = 'hd3;
    8'hd4 : shift_key_code = 'hd4;
    8'hd5 : shift_key_code = 'hd5;
    8'hd6 : shift_key_code = 'hd6;
    8'hd7 : shift_key_code = 'hd7;
    8'hd8 : shift_key_code = 'hd8;
    8'hd9 : shift_key_code = 'hd9;
    8'hda : shift_key_code = 'hda;
    8'hdb : shift_key_code = 'hdb;
    8'hdc : shift_key_code = 'hdc;
    8'hdd : shift_key_code = 'hdd;
    8'hde : shift_key_code = 'hde;
    8'hdf : shift_key_code = 'hdf;
    8'he0 : shift_key_code = 'he0;
    8'he1 : shift_key_code = 'he1;
    8'he2 : shift_key_code = 'he2;
    8'he3 : shift_key_code = 'he3;
    8'he4 : shift_key_code = 'he4;
    8'he5 : shift_key_code = 'he5;
    8'he6 : shift_key_code = 'he6;
    8'he7 : shift_key_code = 'he7;
    8'he8 : shift_key_code = 'he8;
    8'he9 : shift_key_code = 'he9;
    8'hea : shift_key_code = 'hea;
    8'heb : shift_key_code = 'heb;
    8'hec : shift_key_code = 'hec;
    8'hed : shift_key_code = 'hed;
    8'hee : shift_key_code = 'hee;
    8'hef : shift_key_code = 'hef;
    8'hf0 : shift_key_code = 'hf0;
    8'hf1 : shift_key_code = 'hf1;
    8'hf2 : shift_key_code = 'hf2;
    8'hf3 : shift_key_code = 'hf3;
    8'hf4 : shift_key_code = 'hf4;
    8'hf5 : shift_key_code = 'hf5;
    8'hf6 : shift_key_code = 'hf6;
    8'hf7 : shift_key_code = 'hf7;
    8'hf8 : shift_key_code = 'hf8;
    8'hf9 : shift_key_code = 'hf9;
    8'hfa : shift_key_code = 'hfa;
    8'hfb : shift_key_code = 'hfb;
    8'hfc : shift_key_code = 'hfc;
    8'hfd : shift_key_code = 'hfd;
    8'hfe : shift_key_code = 'hfe;
    default: shift_key_code = '0;
endcase
end


always @(*) begin
  ctrl_key_code = 'h00;
case (key_code)
    8'h00 : ctrl_key_code = 'h00;
    8'h01 : ctrl_key_code = 'h01;
    8'h02 : ctrl_key_code = 'h02;
    8'h03 : ctrl_key_code = 'h03;
    8'h04 : ctrl_key_code = 'h04;
    8'h05 : ctrl_key_code = 'h05;
    8'h06 : ctrl_key_code = 'h06;
    8'h07 : ctrl_key_code = 'h07;
    8'h08 : ctrl_key_code = 'h08;
    8'h09 : ctrl_key_code = 'h09;
    8'h0a : ctrl_key_code = 'h0a;
    8'h0b : ctrl_key_code = 'h0b;
    8'h0c : ctrl_key_code = 'h0c;
    8'h0d : ctrl_key_code = 'h0d;
    8'h0e : ctrl_key_code = 'h0e;
    8'h0f : ctrl_key_code = 'h0f;
    8'h10 : ctrl_key_code = 'h10;
    8'h11 : ctrl_key_code = 'h11;
    8'h12 : ctrl_key_code = 'h12;
    8'h13 : ctrl_key_code = 'h13;
    8'h14 : ctrl_key_code = 'h14;
    8'h15 : ctrl_key_code = 'h15;
    8'h16 : ctrl_key_code = 'h16;
    8'h17 : ctrl_key_code = 'h17;
    8'h18 : ctrl_key_code = 'h18;
    8'h19 : ctrl_key_code = 'h19;
    8'h1a : ctrl_key_code = 'h1a;
    8'h1b : ctrl_key_code = 'h1b;
    8'h1c : ctrl_key_code = 'h1c;
    8'h1d : ctrl_key_code = 'h1d;
    8'h1e : ctrl_key_code = 'h1e;
    8'h1f : ctrl_key_code = 'h1f;
    8'h20 : ctrl_key_code = 'h20;
    8'h21 : ctrl_key_code = 'h21;
    8'h22 : ctrl_key_code = 'h22;
    8'h23 : ctrl_key_code = 'h23;
    8'h24 : ctrl_key_code = 'h24;
    8'h25 : ctrl_key_code = 'h25;
    8'h26 : ctrl_key_code = 'h26;
    8'h27 : ctrl_key_code = 'h27;
    8'h28 : ctrl_key_code = 'h28;
    8'h29 : ctrl_key_code = 'h29;
    8'h2a : ctrl_key_code = 'h2a;
    8'h2b : ctrl_key_code = 'h2b;
    8'h2c : ctrl_key_code = 'h2c;
    8'h2d : ctrl_key_code = 'h2d;
    8'h2e : ctrl_key_code = 'h2e;
    8'h2f : ctrl_key_code = 'h2f;
    8'h30 : ctrl_key_code = 'h30;
    8'h31 : ctrl_key_code = 'h31;
    8'h32 : ctrl_key_code = 'h00;
    8'h33 : ctrl_key_code = 'h33;
    8'h34 : ctrl_key_code = 'h34;
    8'h35 : ctrl_key_code = 'h35;
    8'h36 : ctrl_key_code = 'h1f;
    8'h37 : ctrl_key_code = 'h37;
    8'h38 : ctrl_key_code = 'h38;
    8'h39 : ctrl_key_code = 'h39;
    8'h3a : ctrl_key_code = 'h3a;
    8'h3b : ctrl_key_code = 'h3b;
    8'h3c : ctrl_key_code = 'h3c;
    8'h3d : ctrl_key_code = 'h3d;
    8'h3e : ctrl_key_code = 'h3e;
    8'h3f : ctrl_key_code = 'h3f;
    8'h40 : ctrl_key_code = 'h40;
    8'h41 : ctrl_key_code = 'h01;
    8'h42 : ctrl_key_code = 'h02;
    8'h43 : ctrl_key_code = 'h03;
    8'h44 : ctrl_key_code = 'h04;
    8'h45 : ctrl_key_code = 'h05;
    8'h46 : ctrl_key_code = 'h06;
    8'h47 : ctrl_key_code = 'h07;
    8'h48 : ctrl_key_code = 'h08;
    8'h49 : ctrl_key_code = 'h09;
    8'h4a : ctrl_key_code = 'h0a;
    8'h4b : ctrl_key_code = 'h0b;
    8'h4c : ctrl_key_code = 'h0c;
    8'h4d : ctrl_key_code = 'h0d;
    8'h4e : ctrl_key_code = 'h0e;
    8'h4f : ctrl_key_code = 'h0f;
    8'h50 : ctrl_key_code = 'h10;
    8'h51 : ctrl_key_code = 'h11;
    8'h52 : ctrl_key_code = 'h12;
    8'h53 : ctrl_key_code = 'h13;
    8'h54 : ctrl_key_code = 'h14;
    8'h55 : ctrl_key_code = 'h15;
    8'h56 : ctrl_key_code = 'h16;
    8'h57 : ctrl_key_code = 'h17;
    8'h58 : ctrl_key_code = 'h18;
    8'h59 : ctrl_key_code = 'h19;
    8'h5a : ctrl_key_code = 'h1a;
    8'h5b : ctrl_key_code = 'h1b;
    8'h5c : ctrl_key_code = 'h1c;
    8'h5d : ctrl_key_code = 'h1d;
    8'h5e : ctrl_key_code = 'h1e;
    8'h5f : ctrl_key_code = 'h5f;
    8'h60 : ctrl_key_code = 'h60;
    8'h61 : ctrl_key_code = 'h01;
    8'h62 : ctrl_key_code = 'h02;
    8'h63 : ctrl_key_code = 'h03;
    8'h64 : ctrl_key_code = 'h04;
    8'h65 : ctrl_key_code = 'h05;
    8'h66 : ctrl_key_code = 'h06;
    8'h67 : ctrl_key_code = 'h07;
    8'h68 : ctrl_key_code = 'h08;
    8'h69 : ctrl_key_code = 'h09;
    8'h6a : ctrl_key_code = 'h0a;
    8'h6b : ctrl_key_code = 'h0b;
    8'h6c : ctrl_key_code = 'h0c;
    8'h6d : ctrl_key_code = 'h0d;
    8'h6e : ctrl_key_code = 'h0e;
    8'h6f : ctrl_key_code = 'h0f;
    8'h70 : ctrl_key_code = 'h10;
    8'h71 : ctrl_key_code = 'h11;
    8'h72 : ctrl_key_code = 'h12;
    8'h73 : ctrl_key_code = 'h13;
    8'h74 : ctrl_key_code = 'h14;
    8'h75 : ctrl_key_code = 'h15;
    8'h76 : ctrl_key_code = 'h16;
    8'h77 : ctrl_key_code = 'h17;
    8'h78 : ctrl_key_code = 'h18;
    8'h79 : ctrl_key_code = 'h19;
    8'h7a : ctrl_key_code = 'h1a;
    8'h7b : ctrl_key_code = 'h1b;
    8'h7c : ctrl_key_code = 'h1c;
    8'h7d : ctrl_key_code = 'h1d;
    8'h7e : ctrl_key_code = 'h1e;
    8'h7f : ctrl_key_code = 'h7f;
    8'h80 : ctrl_key_code = 'h80;
    8'h81 : ctrl_key_code = 'h81;
    8'h82 : ctrl_key_code = 'h82;
    8'h83 : ctrl_key_code = 'h83;
    8'h84 : ctrl_key_code = 'h84;
    8'h85 : ctrl_key_code = 'h85;
    8'h86 : ctrl_key_code = 'h86;
    8'h87 : ctrl_key_code = 'h87;
    8'h88 : ctrl_key_code = 'h88;
    8'h89 : ctrl_key_code = 'h89;
    8'h8a : ctrl_key_code = 'h8a;
    8'h8b : ctrl_key_code = 'h8b;
    8'h8c : ctrl_key_code = 'h8c;
    8'h8d : ctrl_key_code = 'h8d;
    8'h8e : ctrl_key_code = 'h8e;
    8'h8f : ctrl_key_code = 'h8f;
    8'h90 : ctrl_key_code = 'h90;
    8'h91 : ctrl_key_code = 'h91;
    8'h92 : ctrl_key_code = 'h92;
    8'h93 : ctrl_key_code = 'h93;
    8'h94 : ctrl_key_code = 'h94;
    8'h95 : ctrl_key_code = 'h95;
    8'h96 : ctrl_key_code = 'h96;
    8'h97 : ctrl_key_code = 'h7f;
    8'h98 : ctrl_key_code = 'h98;
    8'h99 : ctrl_key_code = 'h99;
    8'h9a : ctrl_key_code = 'h9a;
    8'h9b : ctrl_key_code = 'h9b;
    8'h9c : ctrl_key_code = 'h9c;
    8'h9d : ctrl_key_code = 'h9d;
    8'h9e : ctrl_key_code = 'h9e;
    8'h9f : ctrl_key_code = 'h9f;
    8'ha0 : ctrl_key_code = 'ha4;
    8'ha1 : ctrl_key_code = 'ha5;
    8'ha2 : ctrl_key_code = 'ha6;
    8'ha3 : ctrl_key_code = 'ha7;
    8'ha4 : ctrl_key_code = 'ha4;
    8'ha5 : ctrl_key_code = 'ha5;
    8'ha6 : ctrl_key_code = 'ha6;
    8'ha7 : ctrl_key_code = 'ha7;
    8'ha8 : ctrl_key_code = 'ha8;
    8'ha9 : ctrl_key_code = 'ha9;
    8'haa : ctrl_key_code = 'haa;
    8'hab : ctrl_key_code = 'hab;
    8'hac : ctrl_key_code = 'hac;
    8'had : ctrl_key_code = 'had;
    8'hae : ctrl_key_code = 'hae;
    8'haf : ctrl_key_code = 'haf;
    8'hb0 : ctrl_key_code = 'hb0;
    8'hb1 : ctrl_key_code = 'hb1;
    8'hb2 : ctrl_key_code = 'hb2;
    8'hb3 : ctrl_key_code = 'hb3;
    8'hb4 : ctrl_key_code = 'hb4;
    8'hb5 : ctrl_key_code = 'hb5;
    8'hb6 : ctrl_key_code = 'hb6;
    8'hb7 : ctrl_key_code = 'hb7;
    8'hb8 : ctrl_key_code = 'hb8;
    8'hb9 : ctrl_key_code = 'hb9;
    8'hba : ctrl_key_code = 'hba;
    8'hbb : ctrl_key_code = 'hbb;
    8'hbc : ctrl_key_code = 'hbc;
    8'hbd : ctrl_key_code = 'hbd;
    8'hbe : ctrl_key_code = 'hbe;
    8'hbf : ctrl_key_code = 'hbf;
    8'hc0 : ctrl_key_code = 'hc0;
    8'hc1 : ctrl_key_code = 'hc1;
    8'hc2 : ctrl_key_code = 'hc2;
    8'hc3 : ctrl_key_code = 'hc3;
    8'hc4 : ctrl_key_code = 'hc4;
    8'hc5 : ctrl_key_code = 'hc5;
    8'hc6 : ctrl_key_code = 'hc6;
    8'hc7 : ctrl_key_code = 'hc7;
    8'hc8 : ctrl_key_code = 'hc8;
    8'hc9 : ctrl_key_code = 'hc9;
    8'hca : ctrl_key_code = 'hca;
    8'hcb : ctrl_key_code = 'hcb;
    8'hcc : ctrl_key_code = 'hcc;
    8'hcd : ctrl_key_code = 'hcd;
    8'hce : ctrl_key_code = 'hce;
    8'hcf : ctrl_key_code = 'hcf;
    8'hd0 : ctrl_key_code = 'hd0;
    8'hd1 : ctrl_key_code = 'hd1;
    8'hd2 : ctrl_key_code = 'hd2;
    8'hd3 : ctrl_key_code = 'hd3;
    8'hd4 : ctrl_key_code = 'hd4;
    8'hd5 : ctrl_key_code = 'hd5;
    8'hd6 : ctrl_key_code = 'hd6;
    8'hd7 : ctrl_key_code = 'hd7;
    8'hd8 : ctrl_key_code = 'hd8;
    8'hd9 : ctrl_key_code = 'hd9;
    8'hda : ctrl_key_code = 'hda;
    8'hdb : ctrl_key_code = 'hdb;
    8'hdc : ctrl_key_code = 'hdc;
    8'hdd : ctrl_key_code = 'hdd;
    8'hde : ctrl_key_code = 'hde;
    8'hdf : ctrl_key_code = 'hdf;
    8'he0 : ctrl_key_code = 'he0;
    8'he1 : ctrl_key_code = 'he1;
    8'he2 : ctrl_key_code = 'he2;
    8'he3 : ctrl_key_code = 'he3;
    8'he4 : ctrl_key_code = 'he4;
    8'he5 : ctrl_key_code = 'he5;
    8'he6 : ctrl_key_code = 'he6;
    8'he7 : ctrl_key_code = 'he7;
    8'he8 : ctrl_key_code = 'he8;
    8'he9 : ctrl_key_code = 'he9;
    8'hea : ctrl_key_code = 'hea;
    8'heb : ctrl_key_code = 'heb;
    8'hec : ctrl_key_code = 'hec;
    8'hed : ctrl_key_code = 'hed;
    8'hee : ctrl_key_code = 'hee;
    8'hef : ctrl_key_code = 'hef;
    8'hf0 : ctrl_key_code = 'hf0;
    8'hf1 : ctrl_key_code = 'hf1;
    8'hf2 : ctrl_key_code = 'hf2;
    8'hf3 : ctrl_key_code = 'hf3;
    8'hf4 : ctrl_key_code = 'hf4;
    8'hf5 : ctrl_key_code = 'hf5;
    8'hf6 : ctrl_key_code = 'hf6;
    8'hf7 : ctrl_key_code = 'hf7;
    8'hf8 : ctrl_key_code = 'hf8;
    8'hf9 : ctrl_key_code = 'hf9;
    8'hfa : ctrl_key_code = 'hfa;
    8'hfb : ctrl_key_code = 'hfb;
    8'hfc : ctrl_key_code = 'hfc;
    8'hfd : ctrl_key_code = 'hfd;
    8'hfe : ctrl_key_code = 'hfe;
    default: ctrl_key_code = 'h00;
endcase
end

endmodule
