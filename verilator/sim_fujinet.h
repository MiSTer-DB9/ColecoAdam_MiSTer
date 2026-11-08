#pragma once
//============================================================================
//  FujiNet link for the simulator: Bus over IP.
//
//  The core's AdamNet master talks to FujiNet-PC through cv_fuji_uart. On the
//  MiSTer the far end is /dev/ttyS1; here it is a TCP socket, the way the
//  ADAMEm emulator connects to FujiNet-PC ("Connecting an Emulator to
//  FujiNet-PC", 2026, ch. 3 and 7): the emulator listens, FujiNet-PC built for
//  the ADAM connects in (its default is localhost:65216), and raw AdamNet bytes
//  flow both ways. So the unmodified native FujiNet-PC build is the device
//  under test, and the sim's slowness does not matter, since that transport
//  has no inter-byte timeout.
//
//  Works on the pins: TXD is decoded as a UART would see it, at the rate the
//  core's UART was told, and RXD is driven bit by bit at the same rate. The
//  socket is point to point; nothing is echoed (FujiNet-PC fakes the one-wire
//  echo on its own side, guide ch. 12).
//============================================================================

#include <cstdint>
#include <cstdio>
#include <deque>
#include <vector>

struct FujiNetLink {
	// --- options ---------------------------------------------------------
	int   port          = 0;        // 0 = FujiNet off
	int   baud          = 115200;   // what the core's UART and this decoder both use
	bool  wait_for_peer = true;     // hold the core in reset until FujiNet-PC connects
	int   wait_seconds  = 30;       // ...for at most this long
	FILE* log           = nullptr;  // --fujinet-log: one line per byte
	bool  verbose       = false;    // hexdump each frame's traffic on stdout

	bool enabled() const { return port != 0; }

	void    start();                      // listen
	bool    holding_reset();              // true while the core should stay in reset
	void    step(int txd, int frame);     // one clk_sys rising edge
	uint8_t rxd() const { return rx_level; }
	void    end_frame(int frame);
	void    summary();

	long tx_count = 0, rx_count = 0, tx_framing_errors = 0;

private:
	static constexpr double CLK_HZ = 10738635.0;
	double bit_clocks() const { return CLK_HZ / baud; }

	int  listen_fd = -1, conn_fd = -1;
	long accept_poll = 0, recv_poll = 0;
	double wait_started = 0;
	bool gave_up = false;

	bool connected();
	void disconnect(const char* why);
	void poll_socket();
	void send_byte(uint8_t b, int frame);

	// TX decoder (core -> socket)
	long   clock = 0;
	double tx_next = 0;
	int    tx_bit = -2;          // -2 idle, -1 start, 0-7 data, 8 stop
	int    tx_value = 0;

	// RX driver (socket -> core)
	std::deque<uint8_t> rxq;
	uint16_t rx_frame = 0;
	int      rx_bits_left = 0;
	double   rx_next = 0;
	uint8_t  rx_level = 1;

	std::vector<uint8_t> tx_line, rx_line;
	void print_line(const char* dir, std::vector<uint8_t>& v, int frame);
};

extern FujiNetLink fujinet;
