//-----------------------------------------------------------------------------
//
// Shared definitions for the FujiNet bridge: the operations cv_adamnet asks
// cv_adamnet_master to run on the AdamNet wire, and the result codes it hands
// back, which are the DCB status values the ADAM's master 6801 would write
// (ADAM Technical Manual, EOS device status codes).
//
//-----------------------------------------------------------------------------

package cv_fuji_pkg;

  // Operations
  localparam [2:0] FOP_STATUS      = 3'd0;  // MN_STATUS, expect the 6 byte status packet
  localparam [2:0] FOP_READ_BLOCK  = 3'd1;  // SEND block#, RECEIVE until ACK, CLR, 1024 bytes back
  localparam [2:0] FOP_WRITE_BLOCK = 3'd2;  // SEND block#, SEND 1024 bytes
  localparam [2:0] FOP_CHAR_WRITE  = 3'd3;  // SEND req_len bytes
  localparam [2:0] FOP_CHAR_READ   = 3'd4;  // RECEIVE until ACK (or NAK = nothing), CLR, bytes back
  localparam [2:0] FOP_RESET       = 3'd5;  // MN_RESET to one device, no reply
  localparam [2:0] FOP_RESET_ALL   = 3'd6;  // MN_RESET to every device FujiNet serves

  // Results, as DCB status bytes: the master 6801's completion codes (FujiNet
  // Programming Guide for the ADAM, Appendix A)
  localparam [7:0] FRES_OK           = 8'h80;  // ADAMNET_OK
  localparam [7:0] FRES_SEND_TIMEOUT = 8'h83;  // SEND packet not answered in time
  localparam [7:0] FRES_SEND_NACK    = 8'h88;  // the device NACKed the data sent
  localparam [7:0] FRES_RECV_TIMEOUT = 8'h89;  // RECEIVE never answered
  localparam [7:0] FRES_RECV_NACK    = 8'h8C;  // the device NACKed a RECEIVE: nothing to read
  localparam [7:0] FRES_CLR_TIMEOUT  = 8'h8D;  // the device did not send its data in time
  localparam [7:0] FRES_STAT_TIMEOUT = 8'h93;  // STATUS not answered: no such device
  localparam [7:0] FRES_TIMEOUT      = 8'h9B;  // general timeout; EOS and fujinet-lib retry on it

  // AdamNet command and response nibbles (high nibble of each wire byte)
  localparam [3:0] MN_RESET   = 4'h0;
  localparam [3:0] MN_STATUS  = 4'h1;
  localparam [3:0] MN_ACK     = 4'h2;
  localparam [3:0] MN_CLR     = 4'h3;
  localparam [3:0] MN_RECEIVE = 4'h4;
  localparam [3:0] MN_CANCEL  = 4'h5;
  localparam [3:0] MN_SEND    = 4'h6;
  localparam [3:0] MN_NAK     = 4'h7;
  localparam [3:0] MN_READY   = 4'hD;
  localparam [3:0] NM_STATUS  = 4'h8;
  localparam [3:0] NM_ACK     = 4'h9;
  localparam [3:0] NM_CANCEL  = 4'hA;
  localparam [3:0] NM_SEND    = 4'hB;
  localparam [3:0] NM_NAK     = 4'hC;

endpackage
