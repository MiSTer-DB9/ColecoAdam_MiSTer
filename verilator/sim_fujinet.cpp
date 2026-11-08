#include "sim_fujinet.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

FujiNetLink fujinet;

static double now_seconds() {
	using namespace std::chrono;
	return duration<double>(steady_clock::now().time_since_epoch()).count();
}

void FujiNetLink::start() {
	listen_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (listen_fd < 0) { perror("fujinet: socket"); exit(1); }
	int on = 1;
	setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof on);
	sockaddr_in a{};
	a.sin_family = AF_INET;
	a.sin_addr.s_addr = htonl(INADDR_ANY);
	a.sin_port = htons((uint16_t)port);
	if (bind(listen_fd, (sockaddr*)&a, sizeof a) < 0) { perror("fujinet: bind"); exit(1); }
	if (listen(listen_fd, 1) < 0) { perror("fujinet: listen"); exit(1); }
	fcntl(listen_fd, F_SETFL, fcntl(listen_fd, F_GETFL) | O_NONBLOCK);
	wait_started = now_seconds();
	printf("FujiNet: listening on TCP port %d for FujiNet-PC (ADAM build, [BOIP] host=localhost port=%d); UART at %d baud\n",
	       port, port, baud);
	if (wait_for_peer) printf("FujiNet: holding reset for up to %d s until it connects\n", wait_seconds);
	fflush(stdout);
}

bool FujiNetLink::connected() {
	if (conn_fd >= 0) return true;
	if (listen_fd < 0) return false;
	int fd = accept(listen_fd, nullptr, nullptr);
	if (fd < 0) return false;
	int on = 1;
	// Request/response one byte at a time: Nagle would sit on every poll.
	setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof on);
	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
	conn_fd = fd;
	printf("FujiNet: FujiNet-PC connected\n");
	fflush(stdout);
	return true;
}

void FujiNetLink::disconnect(const char* why) {
	if (conn_fd >= 0) close(conn_fd);
	conn_fd = -1;
	rxq.clear();
	printf("FujiNet: FujiNet-PC disconnected (%s); listening again\n", why);
	fflush(stdout);
}

bool FujiNetLink::holding_reset() {
	if (!enabled() || !wait_for_peer || gave_up) return false;
	// accept() is a syscall; the main loop calls this every half clock
	if (++accept_poll < 2048) return conn_fd < 0;
	accept_poll = 0;
	if (connected()) return false;
	if (now_seconds() - wait_started > wait_seconds) {
		gave_up = true;
		printf("FujiNet: no FujiNet-PC after %d s; booting without it (it can still connect later)\n", wait_seconds);
		fflush(stdout);
		return false;
	}
	return true;
}

void FujiNetLink::poll_socket() {
	if (!connected()) return;
	uint8_t buf[512];
	ssize_t n = recv(conn_fd, buf, sizeof buf, 0);
	if (n > 0) { rxq.insert(rxq.end(), buf, buf + n); return; }
	if (n == 0) { disconnect("closed"); return; }
	if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) disconnect(strerror(errno));
}

void FujiNetLink::send_byte(uint8_t b, int frame) {
	tx_count++;
	tx_line.push_back(b);
	if (log) fprintf(log, "%6d TX %02X\n", frame, b);
	if (conn_fd < 0) return;
	ssize_t w = send(conn_fd, &b, 1, 0);
	if (w < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) disconnect(strerror(errno));
}

void FujiNetLink::step(int txd, int frame) {
	clock++;
	const double bclk = bit_clocks();

	// ---- TX: decode what the core puts on the pin, 8N1, sampled mid-bit ----
	if (tx_bit == -2) {
		if (!txd) { tx_bit = -1; tx_value = 0; tx_next = clock + bclk / 2; }
	} else if (clock >= tx_next) {
		tx_next += bclk;
		if (tx_bit == -1) {
			tx_bit = txd ? -2 : 0;                    // a high here was a glitch
		} else if (tx_bit < 8) {
			tx_value |= (txd & 1) << tx_bit;
			tx_bit++;
		} else {
			if (!txd) tx_framing_errors++;            // no stop bit
			send_byte((uint8_t)tx_value, frame);
			tx_bit = -2;
		}
	}

	// ---- RX: drive the pin from the socket, 8N1 at the same rate ----
	if (++recv_poll >= 4096) { recv_poll = 0; poll_socket(); }
	if (rx_bits_left == 0 && !rxq.empty() && clock >= rx_next) {
		uint8_t b = rxq.front();
		rxq.pop_front();
		rx_count++;
		rx_line.push_back(b);
		if (log) fprintf(log, "%6d RX %02X\n", frame, b);
		rx_frame = (uint16_t)(1u << 9) | ((uint16_t)b << 1);   // start 0, data, stop 1
		rx_bits_left = 10;
		rx_next = clock;                                        // first bit now
	}
	if (rx_bits_left > 0 && clock >= rx_next) {
		rx_level = rx_frame & 1;
		rx_frame >>= 1;
		rx_bits_left--;
		rx_next += bclk;                 // each bit, the stop bit too, gets a full bit time
	} else if (rx_bits_left == 0) {
		rx_level = 1;
	}
}

void FujiNetLink::print_line(const char* dir, std::vector<uint8_t>& v, int frame) {
	if (v.empty()) return;
	if (verbose) {
		for (size_t i = 0; i < v.size(); i += 16) {
			char hex[16 * 3 + 1] = {0};
			size_t n = std::min<size_t>(16, v.size() - i);
			for (size_t j = 0; j < n; j++) snprintf(hex + j * 3, 4, "%02X ", v[i + j]);
			printf("FUJI %s %6d  %s\n", dir, frame, hex);
		}
		fflush(stdout);
	}
	v.clear();
}

void FujiNetLink::end_frame(int frame) {
	print_line("TX", tx_line, frame);
	print_line("RX", rx_line, frame);
	if (log) fflush(log);
}

void FujiNetLink::summary() {
	printf("fujinet: %ld bytes to FujiNet-PC, %ld from it, %ld framing errors, %s\n",
	       tx_count, rx_count, tx_framing_errors, conn_fd >= 0 ? "connected" : "not connected");
}
