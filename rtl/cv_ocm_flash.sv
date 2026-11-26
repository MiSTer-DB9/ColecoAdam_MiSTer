//-----------------------------------------------------------------------------
//
// The flash chip on Opcode's Super Game Cartridge (the "OCM" mapper).
//
// The cartridge is one JEDEC flash chip (SST39SF010/020 style), and the games keep their high
// scores in it. The save routine (Mooncresta, file offset 1E6FCh) runs from RAM and talks to
// the chip through the E000h window, selecting a bank with FFFEh before each command write so
// that the chip sees the standard command addresses:
//
//   bank 2 + F555h = chip 5555h,  bank 1 + EAAAh = chip 2AAAh
//
//   byte program   AA@5555  55@2AAA  A0@5555  data@addr            then poll until data reads back
//   sector erase   AA@5555  55@2AAA  80@5555  AA@5555  55@2AAA  30@sector   then poll until FFh
//
// The sector is 4K; the games erase chip 1F000h (bank 0Fh + F000h), whose contents in the
// Mooncresta dump are a checksummed score table. The cartridge image in memory is the chip, so a
// program writes straight into it and an erase fills the 4K sector with FFh. While the erase runs
// the chip answers every read with its status, as real flash does (DQ7 = 0, DQ6 toggling on
// each read), and the CPU is kept off memory; the games run the poll from RAM. A program is
// written while the CPU's write cycle is held by the cartridge wait, so it is already done when
// the game reads it back.
//
// Not modelled: software ID (90h) and chip erase (10h), which none of the known games issue.
// A chip erase would wipe the game, so it is ignored. A program writes the byte as given rather
// than ANDing it into the old contents; the games always erase first.
//
// Saves last until the cartridge is loaded again; nothing is written back to the SD card.
//
//-----------------------------------------------------------------------------

module cv_ocm_flash (
    input               clk_i,
    input               reset_n_i,
    input               en_i,          // OCM cartridge loaded
    input               wr_i,          // CPU write cycle in the cartridge space, not FFFC-FFFF
    input               rd_i,          // CPU read cycle in the cartridge space
    input        [19:0] a_i,           // chip address of that cycle, after banking
    input        [ 7:0] d_i,
    input               ready_i,       // cartridge memory has finished its last request
    output logic        busy_o,        // erasing: reads get status_o, CPU stays off memory
    output logic [ 7:0] status_o,
    output logic        mem_wr_o,      // write cartridge memory at mem_a_o
    output logic [19:0] mem_a_o,
    output logic [ 7:0] mem_d_o
);

  typedef enum logic [2:0] {
    S_READ, S_UNLOCK1, S_CMD, S_PROGRAM, S_ERASE_AA, S_ERASE_55, S_ERASE_CMD
  } cmd_t;

  cmd_t        cmd;
  logic        wr_q, rd_q;
  logic        prog;           // this CPU write cycle programs a byte
  logic        toggle;         // DQ6
  logic [ 7:0] sector;         // 4K sector being erased: chip address bits 19:12
  logic [11:0] ecnt;
  logic [ 1:0] estate;         // 0 idle, 1 issue write, 2 let the request register, 3 wait ready

  logic [14:0] ca;
  assign ca = a_i[14:0];       // the command decode sees A14-A0

  always_ff @(posedge clk_i, negedge reset_n_i) begin
    if (~reset_n_i) begin
      cmd    <= S_READ;
      wr_q   <= 1'b0;
      rd_q   <= 1'b0;
      prog   <= 1'b0;
      toggle <= 1'b0;
      estate <= 2'd0;
      ecnt   <= '0;
      sector <= '0;
    end else begin
      wr_q <= wr_i;
      rd_q <= rd_i;
      if (~wr_i) prog <= 1'b0;

      // One step per CPU write cycle, on its leading edge. Writes are ignored mid-erase.
      if (en_i && wr_i && ~wr_q && estate == 2'd0) begin
        cmd <= S_READ;
        case (cmd)
          S_READ:      if (ca == 15'h5555 && d_i == 8'hAA) cmd <= S_UNLOCK1;
          S_UNLOCK1:   if (ca == 15'h2AAA && d_i == 8'h55) cmd <= S_CMD;
          S_CMD:       if (ca == 15'h5555) begin
                         if (d_i == 8'hA0) cmd <= S_PROGRAM;
                         if (d_i == 8'h80) cmd <= S_ERASE_AA;
                       end
          S_PROGRAM:   prog <= 1'b1;
          S_ERASE_AA:  if (ca == 15'h5555 && d_i == 8'hAA) cmd <= S_ERASE_55;
          S_ERASE_55:  if (ca == 15'h2AAA && d_i == 8'h55) cmd <= S_ERASE_CMD;
          S_ERASE_CMD: if (d_i == 8'h30) begin
                         sector <= a_i[19:12];
                         ecnt   <= '0;
                         estate <= 2'd1;
                       end
          default: ;
        endcase
      end

      // Sector erase: 4096 writes of FFh, each waiting for the memory to take the last.
      case (estate)
        2'd1: estate <= 2'd2;
        2'd2: estate <= 2'd3;
        2'd3: if (ready_i) begin
                ecnt   <= ecnt + 1'd1;
                estate <= (ecnt == 12'hFFF) ? 2'd0 : 2'd1;
              end
        default: ;
      endcase

      if (busy_o && rd_i && ~rd_q) toggle <= ~toggle;
    end
  end

  assign busy_o   = estate != 2'd0;
  assign status_o = {1'b0, toggle, 6'b0};
  assign mem_wr_o = (estate == 2'd1) | prog;
  assign mem_a_o  = busy_o ? {sector, ecnt} : a_i;
  assign mem_d_o  = busy_o ? 8'hFF : d_i;

endmodule
