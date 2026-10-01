// Testbench for rtl/cv_printer_uart.sv: the FIFO and serial transmitter that carry
// the ADAM's printer bytes to the HPS UART.
//
// The producer behaves like cv_adamnet's printer fetch: it looks at ready, and a
// byte arrives three clocks later. The receiver is an ordinary 8N1 UART that
// samples each bit in the middle, at the real 42.666 MHz clock.
//
// What is checked:
//   1  a burst larger than the FIFO comes out whole and in order at 115200 baud,
//      with every stop bit high
//   2  ready drops while the FIFO is full, so the ADAM is held off rather than
//      bytes being lost
//   3  the same at 9600 baud, where the bit timing error would show up first
//   4  with the port not in Printer mode, bytes are dropped, ready stays high and
//      TXD stays at the idle mark level

#include <verilated.h>
#include "Vcv_printer_uart.h"
#include <cstdio>
#include <cstdlib>
#include <vector>

static Vcv_printer_uart* top;
static vluint64_t main_time = 0;
double sc_time_stamp() { return main_time; }

static const double CLK_HZ = 42666000.0;
static int failures = 0;

static void fail(const char* what) { printf("FAIL: %s\n", what); failures++; }

// Receiver state, advanced every clock
struct Receiver {
    double bit_clocks = 0;
    int state = 0;          // 0 idle, 1 receiving
    double next_sample = 0;
    int bit = 0;
    int value = 0;
    long clock = 0;
    int framing_errors = 0;
    std::vector<int> got;

    void reset(int baud) { bit_clocks = CLK_HZ / baud; state = 0; got.clear(); framing_errors = 0; clock = 0; }

    void step(int txd) {
        clock++;
        if (state == 0) {
            if (txd == 0) {               // start bit edge: sample its middle, then each bit
                state = 1;
                next_sample = clock + bit_clocks * 0.5;
                bit = -1;
                value = 0;
            }
            return;
        }
        if (clock < next_sample) return;
        next_sample += bit_clocks;
        if (bit == -1) {
            if (txd != 0) { state = 0; framing_errors++; }   // glitch, not a start bit
        } else if (bit < 8) {
            value |= txd << bit;
        } else {
            if (txd != 1) framing_errors++;
            got.push_back(value);
            state = 0;
            return;
        }
        bit++;
    }
};

static Receiver rx;

static void tick() {
    top->clk = 0; top->eval(); main_time++;
    top->clk = 1; top->eval(); main_time++;
    rx.step(top->txd);
}

// Send data the way cv_adamnet does, then let the line drain. Returns how many
// clocks ready was low for.
static long run(const std::vector<int>& data, long drain_clocks) {
    long ready_low = 0;
    size_t next = 0;
    int pending = -1, delay = 0;
    while (next < data.size() || pending >= 0) {
        top->wr = 0;
        if (pending >= 0) {
            if (--delay == 0) { top->din = pending; top->wr = 1; pending = -1; }
        } else if (top->ready) {
            pending = data[next++];
            delay = 3;
        }
        if (!top->ready) ready_low++;
        tick();
    }
    top->wr = 0;
    for (long i = 0; i < drain_clocks; i++) tick();
    return ready_low;
}

static void check_stream(const char* name, const std::vector<int>& sent) {
    if (rx.framing_errors) { printf("  %s: %d framing errors\n", name, rx.framing_errors); fail(name); }
    if (rx.got.size() != sent.size()) {
        printf("  %s: sent %zu bytes, received %zu\n", name, sent.size(), rx.got.size());
        fail(name);
        return;
    }
    for (size_t i = 0; i < sent.size(); i++) {
        if (rx.got[i] != sent[i]) {
            printf("  %s: byte %zu sent %02x received %02x\n", name, i, sent[i], rx.got[i]);
            fail(name);
            return;
        }
    }
    printf("  %s: %zu bytes intact\n", name, sent.size());
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    top = new Vcv_printer_uart;

    top->reset = 1; top->enable = 1; top->baud = 115200; top->wr = 0;
    for (int i = 0; i < 10; i++) tick();
    top->reset = 0;

    // 1, 2: a burst bigger than the 512 byte FIFO at 115200
    std::vector<int> burst;
    for (int i = 0; i < 1500; i++) burst.push_back((i * 37 + (i >> 3)) & 0xFF);
    rx.reset(115200);
    long held = run(burst, (long)(CLK_HZ / 115200 * 10 * 520));   // drain a full FIFO
    check_stream("115200 burst", burst);
    if (held == 0) fail("ready never dropped, so the FIFO was never full");
    else printf("  ready was low for %ld clocks while the FIFO was full\n", held);

    // 3: 9600 baud
    top->baud = 9600;
    for (int i = 0; i < 10; i++) tick();
    std::vector<int> text;
    for (const char* p = "\r         HELLO WORLD\r\x0b\x0b\x0e.WOR\x0f\x08\x7f\x00\xff"; *p; p++) text.push_back((unsigned char)*p);
    text.push_back(0x00);
    text.push_back(0xFF);
    rx.reset(9600);
    run(text, (long)(CLK_HZ / 9600 * 10 * (text.size() + 2)));
    check_stream("9600 text", text);

    // 4: not in Printer mode
    top->enable = 0;
    for (int i = 0; i < 10; i++) tick();
    rx.reset(9600);
    bool ready_dropped = false, txd_dropped = false;
    for (int i = 0; i < 2000; i++) {
        top->din = 0x55; top->wr = (i % 3) == 0;
        if (!top->ready) ready_dropped = true;
        tick();
        if (!top->txd) txd_dropped = true;
    }
    top->wr = 0;
    if (ready_dropped) fail("ready dropped with the port disabled");
    if (txd_dropped) fail("TXD left the mark level with the port disabled");
    if (!rx.got.empty()) fail("bytes came out with the port disabled");
    if (!ready_dropped && !txd_dropped && rx.got.empty()) printf("  disabled: bytes dropped, ready high, TXD idle\n");

    printf(failures ? "FAILED (%d)\n" : "PASSED\n", failures);
    delete top;
    return failures ? 1 : 0;
}
