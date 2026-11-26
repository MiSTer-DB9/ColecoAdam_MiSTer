//-----------------------------------------------------------------------------
//
// Cartridge mapper detection, from the image as it is downloaded.
//
// A ColecoVision cartridge starts only if the BIOS finds 55AAh or AA55h at 8000h. Each paging
// scheme puts a different part of the ROM there at reset, so where the header sits in the image
// says which scheme it was built for:
//
//   MegaCart     the last 16K page is fixed at 8000-BFFF, so the header is at (size - 16K).
//   Opcode OCM   bank 0 is at 8000h after reset, so the header is at offset 0 (Opcode's Super
//                Game Cartridges: Gradius, Mooncresta, Pac-Man DX, DK Arcade and others).
//
// A paged image with a header at offset 0 and none in its last page is therefore OCM. A
// MegaCart without a header in its last page could never have started, so no working MegaCart is
// reclassified, and an image with headers in both places stays a MegaCart.
//
//-----------------------------------------------------------------------------

module cv_cart_detect (
    input               clk_i,
    input               wr_i,          // ioctl_wr for the cartridge download
    input        [24:0] addr_i,
    input        [ 7:0] d_i,
    input        [ 5:0] cart_pages_i,  // highest 16K page written, as the top level keeps it
    output logic        ocm_o
);

  logic [7:0] first_q;      // byte 0 of the image
  logic [7:0] page_q;       // byte 0 of the current 16K page
  logic       hdr_first;    // header at offset 0
  logic       hdr_page;     // header at the start of the latest page seen, which ends as the last

  function automatic logic is_hdr(input logic [7:0] b0, input logic [7:0] b1);
    return ({b0, b1} == 16'h55AA) || ({b0, b1} == 16'hAA55);
  endfunction

  always_ff @(posedge clk_i) begin
    if (wr_i) begin
      if (addr_i == 25'd0) begin
        first_q   <= d_i;
        hdr_first <= 1'b0;
      end
      if (addr_i == 25'd1) hdr_first <= is_hdr(first_q, d_i);
      if (addr_i[13:0] == 14'd0) begin
        page_q   <= d_i;
        hdr_page <= 1'b0;
      end
      if (addr_i[13:0] == 14'd1) hdr_page <= is_hdr(page_q, d_i);
    end
  end

  // The paged sizes cv_addr_dec treats as a MegaCart: 64K to 1M, a power of two.
  logic paged;
  assign paged = (cart_pages_i == 6'b000011) | (cart_pages_i == 6'b000111) |
                 (cart_pages_i == 6'b001111) | (cart_pages_i == 6'b011111) |
                 (cart_pages_i == 6'b111111);

  assign ocm_o = paged & hdr_first & ~hdr_page;

endmodule
