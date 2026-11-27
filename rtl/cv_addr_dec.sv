//-----------------------------------------------------------------------------
//
// FPGA Colecovision
//
// $Id: cv_addr_dec.vhd,v 1.3 2006/01/05 22:22:29 arnim Exp $
//
// Address Decoder
//
//-----------------------------------------------------------------------------
//
// Copyright (c) 2006, Arnim Laeuger (arnim.laeuger@gmx.net)
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
// SystemVerilog conversion (c) 2022 Frank Bruno (fbruno@asicsolutions.com)
//
//-----------------------------------------------------------------------------

module cv_addr_dec (
    input               clk_i,
    input               reset_n_i,
    input               mode,
    input               game_mode_i,
    input        [15:0] a_i,
    input        [ 7:0] d_i,
    input        [ 5:0] cart_pages_i,
    input               ocm_i,          // Opcode Super Game Cartridge (cv_cart_detect)
    output logic [ 6:0] cart_bank_o,    // 8K bank of the cartridge image; A12-A0 complete it
    output logic        cart_wr_o,      // write cycle to the cartridge's flash (OCM only)
    input               iorq_n_i,
    input               rd_n_i,
    input               wr_n_i,
    input               mreq_n_i,
    input               rfsh_n_i,
    output logic        bios_rom_ce_n_o,
    output logic        eos_rom_ce_n_o,
    output logic        writer_rom_ce_n_o,
    output logic        ram_ce_n_o,
    output logic        lowerexpansion_ram_ce_n_o,
    output logic        upper_ram_ce_n_o,
    output logic        expansion_ram_ce_n_o,
    output logic        expansion_rom_ce_n_o,
    output logic        cartridge_rom_ce_n_o,
    output logic        vdp_r_n_o,
    output logic        vdp_w_n_o,
    output logic        psg_we_n_o,
    output logic        ay_addr_we_n_o,
    output logic        ay_data_we_n_o,
    output logic        ay_data_rd_n_o,
    output logic        adam_reset_pcb_n_o,
    output logic        ctrl_r_n_o,
    output logic        ctrl_en_key_n_o,
    output logic        ctrl_en_joy_n_o,
    output logic        ram_mirror_o,
    output logic [7:0]  exp_bank_o
);

  logic       megacart_en;
  logic [5:0] megacart_page;
  logic [6:0] ocm_bank [4];
  logic [6:0] ocm_mask;
  logic       cart_space;     // 8000-FFFF is the cartridge in the current memory map
  logic       ocm_reg_wr;
 // logic       bios_en;
  logic       eos_en;
  logic       last_35_reset_bit;
  logic [1:0] lower_mem_adam;
  logic [1:0] upper_mem_adam;
  logic [1:0] lower_mem_nadam;
  logic [1:0] upper_mem_nadam;
  logic [1:0] lower_mem;
  logic [1:0] upper_mem;
  logic       sgm_ram_en;

  // A ColecoVision has 1K of RAM, decoded at 6000-7FFF and mirrored through it, with
  // 2000-5FFF left to the expansion port. The ADAM has 24K there, as does the Super Game
  // Module once port 53h enables it.
  assign ram_mirror_o = mode & ~sgm_ram_en & (lower_mem == 2'b11);
  //---------------------------------------------------------------------------
  // Process dec
  //
  // Purpose:
  //   Implements the address decoding logic.
  //
  always_comb begin : dec
    // default assignments
    bios_rom_ce_n_o = '1;
    eos_rom_ce_n_o = '1;
    writer_rom_ce_n_o = '1;
    eos_rom_ce_n_o = '1;
    ram_ce_n_o = '1;
    lowerexpansion_ram_ce_n_o = '1;
    upper_ram_ce_n_o = '1;
    expansion_ram_ce_n_o = '1;
    expansion_rom_ce_n_o = '1;
    cartridge_rom_ce_n_o = '1;
    vdp_r_n_o = '1;
    vdp_w_n_o = '1;
    psg_we_n_o = '1;
    ay_addr_we_n_o = '1;
    ay_data_we_n_o = '1;
    ay_data_rd_n_o = '1;
    adam_reset_pcb_n_o = '1;
    ctrl_r_n_o = '1;
    ctrl_en_key_n_o = '1;
    ctrl_en_joy_n_o = '1;


    megacart_en = ~ocm_i &
                  (cart_pages_i == 6'b000011 |
                   cart_pages_i == 6'b000111 |
                   cart_pages_i == 6'b001111 |
                   cart_pages_i == 6'b011111 |
                   cart_pages_i == 6'b111111);		// 1M

    // Paging, in 8K banks. MegaCart: the last 16K page fixed at 8000h and the page selected by
    // a read of FFC0-FFFF at C000h. OCM: four 8K windows, each with its own register; the order
    // is FBNeo's (d_coleco.cpp update_map), which the games' start-up code confirms - Mooncresta
    // runs on from 8000h after writing FFFFh=00 and jumps into C000h after FFFDh=0Eh.
    if (ocm_i) begin
      case (a_i[14:13])
        2'b00: cart_bank_o = ocm_bank[3];   // 8000-9FFF  FFFFh
        2'b01: cart_bank_o = ocm_bank[0];   // A000-BFFF  FFFCh
        2'b10: cart_bank_o = ocm_bank[1];   // C000-DFFF  FFFDh
        2'b11: cart_bank_o = ocm_bank[2];   // E000-FFFF  FFFEh
      endcase
    end else begin
      case (a_i[15:14])
        2'b10: cart_bank_o = {megacart_en ? cart_pages_i : 6'b000000, a_i[13]};
        2'b11: cart_bank_o = {megacart_en ? megacart_page : 6'b000001, a_i[13]};
        default: cart_bank_o = {6'b000000, a_i[13]};
      endcase
    end

    // Memory access ----------------------------------------------------------
    if (~mreq_n_i && rfsh_n_i) begin

      //      end else begin
      if (~a_i[15]) begin
        if (lower_mem == 2'b11) begin  // OS7 / 24k RAM
          case (a_i[15:13])
            3'b000:                         bios_rom_ce_n_o = '0;
            3'b001, 3'b010:                 if (~ram_mirror_o) ram_ce_n_o = '0;  // 2000 - 5fff
            3'b011:                         ram_ce_n_o = '0;  // 6000 - 7fff
            3'b100, 3'b101, 3'b110, 3'b111: cartridge_rom_ce_n_o = '0;
            default: begin
            end
          endcase

        end else if (lower_mem == 2'b10) begin  // RAM expansion
          lowerexpansion_ram_ce_n_o = '0;
        end else if (lower_mem == 2'b01) begin  // 32K of RAM
          ram_ce_n_o = '0;  // 2000 - 7fff = 24k
        end else if (lower_mem == 2'b00) begin  // WRITER ROM (when do we use EOS?)
          if (eos_en) eos_rom_ce_n_o = '0;
          else writer_rom_ce_n_o = '0;
        end
      end

      if (a_i[15]) begin

        if (upper_mem == 2'b10) begin  // RAM expansion
          expansion_ram_ce_n_o = '0;
        end else if (upper_mem == 2'b01) begin  // ROM expansion
          expansion_rom_ce_n_o = '0;
        end else if (upper_mem == 2'b00) begin  // 32k RAM
          upper_ram_ce_n_o = '0;
        end else if (upper_mem == 2'b11) begin  // ROM Cartridge
          cartridge_rom_ce_n_o = '0;
        end
      end
    end

    // I/O access -------------------------------------------------------------
    if (~iorq_n_i) begin
      if (a_i[7]) begin
        case ({
          a_i[6], a_i[5], wr_n_i
        })
          3'b000: ctrl_en_key_n_o = '0;
          3'b010: vdp_w_n_o = '0;
          3'b011: if (~rd_n_i) vdp_r_n_o = '0;
          3'b100: ctrl_en_joy_n_o = '0;
          3'b110: psg_we_n_o = '0;
          3'b111: if (~rd_n_i) ctrl_r_n_o = '0;
          default: begin
          end
        endcase
      end

      if (a_i[7:0] == 8'h50 && ~wr_n_i) ay_addr_we_n_o = '0;
      else if (a_i[7:0] == 8'h51 && ~wr_n_i) ay_data_we_n_o = '0;
      else if (a_i[7:0] == 8'h52 && ~rd_n_i) ay_data_rd_n_o = '0;
      // The ADAM decodes these by range, like the ColecoVision ports above: its memory board
      // (MIOC U7) sees only BA13-BA15 and the game board's AUX DECODE lines, not A0-A4. Ports
      // 20h-3Fh are net reset / EOS enable and 60h-7Fh memory map control; Coleco's 64K
      // Expansion RAM Test writes 20h and 60h.
      else if (a_i[7:5] == 3'b001 && ~wr_n_i && d_i == 8'h0F) adam_reset_pcb_n_o = '0;
    end
  end

  //
  //---------------------------------------------------------------------------
  always @(negedge reset_n_i, posedge clk_i) begin : m_adam
    if (~reset_n_i) begin
      // Console, or the ADAM's cartridge reset: OS-7 + 24K RAM and the cartridge. The ADAM's
      // computer reset selects SmartWRITER and intrinsic RAM (ADAM Technical Manual 2.3, 2.6).
      lower_mem_adam <= (mode | game_mode_i) ? 2'b11 : 2'b00;
      upper_mem_adam <= (mode | game_mode_i) ? 2'b11 : 2'b00;
      sgm_ram_en <= 1'b0;
      exp_bank_o <= 8'h00;
    end else begin
      // Expansion RAM bank on memory expanders past 64K (MESS adam.c: "42-42 (W) = Expansion
      // RAM page selection"). T-DOS and RAMTEST v2.0 write a bank number here and then use the
      // ordinary expansion windows of port 7Fh; each bank is 64K.
      if (~iorq_n_i && mreq_n_i && rfsh_n_i && ~wr_n_i && (a_i[7:0] == 8'h42))
        exp_bank_o <= d_i;
      if (~iorq_n_i && mreq_n_i && rfsh_n_i && ~wr_n_i && (a_i[7:0] == 8'h53))
        sgm_ram_en <= d_i[0];
      if (~iorq_n_i && mreq_n_i && rfsh_n_i && ~wr_n_i && (a_i[7:5] == 3'b011)) begin
        $display("D CHANGING MEM 7F lower %x upper %x", d_i[1:0], d_i[3:2]);
        lower_mem_adam <= d_i[1:0];
        upper_mem_adam <= d_i[3:2];
      end else
		  if (~iorq_n_i && mreq_n_i && rfsh_n_i && ~wr_n_i && (a_i[7:5] == 3'b001)) begin
        $display("B SETTING MEMORY MODE 3F? addr %x data %x", a_i, d_i);
        last_35_reset_bit <= d_i[0];
        eos_en <= d_i[1];
      end

    end
  end


  assign lower_mem = lower_mem_adam;
  assign upper_mem = upper_mem_adam;

  // OCM: a write to FFFC-FFFF sets a bank register; any other write to the cartridge goes to its
  // flash chip (cv_ocm_flash). The bank number is masked to the image, 8K banks.
  assign cart_space = a_i[15] & (upper_mem == 2'b11);
  assign ocm_reg_wr = ocm_i & cart_space & ~mreq_n_i & rfsh_n_i & ~wr_n_i & (a_i[15:2] == 14'h3FFF);
  assign cart_wr_o  = ocm_i & cart_space & ~mreq_n_i & rfsh_n_i & ~wr_n_i & (a_i[15:2] != 14'h3FFF);
  assign ocm_mask   = {cart_pages_i, 1'b1};

  always @(negedge reset_n_i, posedge clk_i) begin : ocm_regs
    if (~reset_n_i) begin
      // FBNeo's reset values. Only bank 0 at 8000h matters: it holds the header and the start-up
      // code, which sets all four registers before it leaves 8000-801Fh.
      ocm_bank[0] <= 7'd3;
      ocm_bank[1] <= 7'd2;
      ocm_bank[2] <= 7'd1;
      ocm_bank[3] <= 7'd0;
    end else if (ocm_reg_wr) begin
      ocm_bank[a_i[1:0]] <= d_i[6:0] & ocm_mask;
    end
  end

  always @(negedge reset_n_i, posedge clk_i) begin : megacart
    if (~reset_n_i) begin
      megacart_page     <= '0;
      //bios_en           <= '1;
      last_35_reset_bit <= '0;
      //eos_en            <= '0;
    end else begin
      // MegaCart paging
      if (megacart_en && rfsh_n_i && ~mreq_n_i && ~rd_n_i && (a_i[15:6] == {8'hFF, 2'b11}))
        megacart_page <= a_i[5:0] & cart_pages_i;


//      if (~iorq_n_i && mreq_n_i && rfsh_n_i && ~wr_n_i && (a_i[7:0] == 8'h7f)) begin
//        bios_en <= d_i[1];
//        $display("A SETTING MEMORY MODE 7F? %x", a_i, d_i);
//      end

      // just 3F or all addresses?
//      if (~iorq_n_i && mreq_n_i && rfsh_n_i && ~wr_n_i && (a_i[7:0] == 8'h3f)) begin
//        $display("B SETTING MEMORY MODE 3F? addr %x data %x", a_i, d_i);
//        last_35_reset_bit <= d_i[0];
//        eos_en <= d_i[1];
//      end
    end
  end

endmodule
