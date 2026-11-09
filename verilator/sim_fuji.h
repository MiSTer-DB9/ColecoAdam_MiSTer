#pragma once
//============================================================================
//  A stand-in for FujiNet-PC on the FujiNet link (--fuji-disk FILE), for runs
//  that need no daemon, a set block latency, or lost packets.
//
//  It answers the AdamNet wire protocol the way FujiNet-PC's ADAM build does
//  (lib/device/adamnet/disk.cpp, lib/bus/adamnet): disk 1 (device 4) serves a
//  .dsk image with the ADAM's 5:1 sector interleave (a .ddp in order), and the
//  other devices FujiNet registers answer STATUS. A block number SEND is ACKed
//  at once and the block becomes ready --fuji-latency ms later, as a TNFS fetch
//  does; RECEIVE gets no answer until then. --fuji-drop-clr N ignores every Nth
//  CLR, as one lost on the line.
//
//  It sits behind FujiNetLink (sim_fujinet.cpp) in place of the socket, so the
//  core's UART pins and their byte timing are the same as with FujiNet-PC.
//  Contributed with the 2026-11-08 test-package fixes; adapted from a
//  byte-level model to the pin-level link.
//============================================================================

#include <cstdint>
#include <cstdio>
#include <deque>
#include <initializer_list>
#include <string>
#include <vector>

struct FakeFuji {
	static constexpr double CLK_HZ = 10738635.0;

	bool on = false;
	bool linear = false;        // a .ddp: blocks in order, no sector interleave
	std::vector<uint8_t> disk;
	double latency_ms = 300;    // --fuji-latency: a block number to the block being ready
	int drop_clr = 0;           // --fuji-drop-clr N
	int clr_count = 0;

	std::deque<uint8_t> out;    // reply bytes, released from out_at on
	long out_at = 0;

	int st = 0;                 // 0 command, 1 len hi, 2 len lo, 3 data, 4 checksum
	uint8_t cmd = 0, dev = 0;
	std::vector<uint8_t> pkt;
	int pkt_len = 0;
	uint32_t block = 0xFFFFFFFF;
	long ready_at = -1;
	long clk = 0;

	bool load(const std::string& path) {
		FILE* f = fopen(path.c_str(), "rb");
		if (!f) return false;
		uint8_t b[4096];
		size_t n;
		while ((n = fread(b, 1, sizeof b, f)) > 0) disk.insert(disk.end(), b, b + n);
		fclose(f);
		linear = path.size() > 4 && path.compare(path.size() - 4, 4, ".ddp") == 0;
		on = true;
		return true;
	}

	double ms() const { return clk * 1000.0 / CLK_HZ; }
	long clks(double m) const { return (long)(m * CLK_HZ / 1000); }

	void reply(std::initializer_list<uint8_t> bytes, double delay_ms) {
		if (out.empty()) out_at = clk + clks(delay_ms);
		out.insert(out.end(), bytes);
	}

	static int interleave(int s) {
		static const int t[8] = { 0, 5, 2, 7, 4, 1, 6, 3 };
		return (s & ~7) | t[s & 7];
	}

	void send_block() {
		if (out.empty()) out_at = clk;
		out.push_back(0xB0 | dev);
		out.push_back(0x04);
		out.push_back(0x00);
		uint8_t ck = 0;
		for (int half = 0; half < 2; half++) {
			int sector = block * 2 + half;
			size_t off = (size_t)(linear ? sector : interleave(sector)) * 512;
			for (int i = 0; i < 512; i++) {
				uint8_t v = off + i < disk.size() ? disk[off + i] : 0;
				out.push_back(v);
				ck ^= v;
			}
		}
		out.push_back(ck);
	}

	void command(uint8_t b) {
		cmd = b >> 4;
		dev = b & 0x0F;
		bool known = dev == 2 || dev == 3 || (dev >= 4 && dev <= 7) || dev == 9 || dev == 0xA || dev == 0xF;
		switch (cmd) {
		case 0x0:   // RESET: no reply
			if (dev == 4) { block = 0xFFFFFFFF; ready_at = -1; }
			break;
		case 0x1: { // STATUS
			if (!known) break;
			uint8_t len_lo = 0x00, len_hi = 0x04, type = dev >= 4 && dev <= 7 ? 0x01 : 0x00;
			uint8_t status = dev == 4 ? 0x40 : (dev >= 5 && dev <= 7) ? 0x43 : 0x00;
			uint8_t ck = len_lo ^ len_hi ^ type ^ status;
			reply({ (uint8_t)(0x80 | dev), len_lo, len_hi, type, status, ck }, 1.0);
			break;
		}
		case 0x4:   // RECEIVE: ACK once the data is ready, silence until then
			if (dev == 4) {
				if (ready_at >= 0 && clk >= ready_at) reply({ (uint8_t)(0x90 | dev) }, 0.5);
			} else if (known) {
				reply({ (uint8_t)(0xC0 | dev) }, 0.5);
			}
			break;
		case 0x3:   // CLR: the data packet
			if (dev == 4 && ready_at >= 0 && clk >= ready_at) {
				if (drop_clr && ++clr_count % drop_clr == 0) {
					fprintf(stderr, "fuji %9.1f ms: CLR dropped\n", ms());
					break;
				}
				fprintf(stderr, "fuji %9.1f ms: CLR -> block %u\n", ms(), block);
				send_block();
			}
			break;
		case 0x6:   // SEND: a packet follows
			st = 1;
			pkt.clear();
			break;
		default:    // ACK, CANCEL, NAK, READY: nothing
			break;
		}
	}

	// One byte from the core, at clock now
	void byte_in(uint8_t b, long now) {
		clk = now;
		switch (st) {
		case 0: command(b); break;
		case 1: pkt_len = b << 8; st = 2; break;
		case 2: pkt_len |= b; st = pkt_len ? 3 : 4; break;
		case 3: pkt.push_back(b); if ((int)pkt.size() == pkt_len) st = 4; break;
		case 4:
			st = 0;
			if (dev == 4 && pkt_len == 5) {
				block = pkt[0] | pkt[1] << 8 | pkt[2] << 16 | (uint32_t)pkt[3] << 24;
				ready_at = clk + clks(latency_ms);
				fprintf(stderr, "fuji %9.1f ms: BLOCK %u\n", ms(), block);
			} else {
				fprintf(stderr, "fuji %9.1f ms: SEND %x, %d bytes\n", ms(), dev, pkt_len);
			}
			reply({ (uint8_t)(0x90 | dev) }, 0.5);
			break;
		}
	}

	// Reply bytes due by clock now, moved to the link's transmit queue
	void pull(long now, std::deque<uint8_t>& q) {
		clk = now;
		if (out.empty() || now < out_at) return;
		q.insert(q.end(), out.begin(), out.end());
		out.clear();
	}
};
