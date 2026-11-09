# Handoff: testing branch `fujinet` on the MiSTer

Branch `fujinet`, on top of `adam-printer` (`0946799`, the AdamNet printer to the HPS UART).
Nothing on it has been on hardware. Everything below was checked in the Verilator simulator
against the real FujiNet firmware, FujiNet-PC built for the ADAM, connected over a socket the way
the ADAMEm emulator does it. `STATUS.md` (FujiNet section) has the evidence, `TODO.md` section 0a
the open work, and `CLAUDE.md` (FujiNet section) how the pieces fit and how to run the simulator.

The previous handoff covered branch `sdram-expansion`, released as `ColecoAdam_20260922`; its
unfinished hardware items are listed at the end of `TODO.md` section 1 and are not this branch's.

## 1. The build to test

`docs/builds/ColecoAdam_fujinet_20261109.rbf` (md5 `d4f986b8785a`), built 2026-11-09 on
misterubuntu, Quartus 17.0.2, 0 errors. It is `20261108d` (the "FujiNet: boot CONFIG" OSD item,
which passed on mister192) plus a tester's three fixes for ADAM CP/M and for loading from the
menu (section 2). It is not on mister192 yet. `files.qip` lists the three new files
(`rtl/cv_fuji_pkg.sv` first, since the other two import it).

| Resource | `20260922` release | This build |
|---|---|---|
| Logic (ALMs) | 37% | 40% |
| RAM blocks | 210 / 553 | 214 / 553 |
| `clk_sys` setup slack | +5.4 ns | +4.75 ns |

Build it again with `quartus_sh --flow compile ColecoAdam` in a copy of the tree (`CLAUDE.md`
has the rsync).

## 2. What changed

| # | Change | Files | Behaviour you'll notice |
|---|---|---|---|
| 15 | **FujiNet.** With the OSD's UART connection set to FujiNet (mode 8) the core is the AdamNet bus master for the devices FujiNet-PC serves on the HPS: printer 2, clock 3, disks 4-7 that have no image mounted, network 9-A, the Fuji gateway F. Keyboard and tapes stay in the core; a floppy with an image mounted stays local | `rtl/cv_adamnet_master.sv`, `rtl/cv_fuji_uart.sv`, `rtl/cv_fuji_pkg.sv` (new); `rtl/cv_adamnet.sv`, `rtl/cv_console.sv`, `ColecoAdam.sv`, `files.qip` | With floppy 1 empty the ADAM boots FujiNet's CONFIG instead of SmartWRITER. In any other UART mode nothing changes |

| 16 | **CP/M and load fixes** from a tester of the 20261108 package: a FujiNet disk answers a new block busy (9Bh) first; a lost CLR is re-polled; after a core load the ADAM is held in reset until Main's UART mode arrives (at most 1.5 s), and the FujiNet on/off reset lasts 1.5 ms | `rtl/cv_adamnet.sv`, `rtl/cv_adamnet_master.sv`, `ColecoAdam.sv` | ADAM CP/M 2.2 from a FujiNet disk reaches `A>`. Every core load waits up to 1.5 s before the ADAM starts when the UART connection is None |

The wire protocol and its timing are in the header of `cv_adamnet_master.sv`. After a reset the
core sends MN_RESET to every FujiNet device and then probes the gateway for up to 8 s, so a daemon
still starting (Main restarts it on every core load) is there before EOS scans for devices; with
FujiNet mode on and no daemon at all, a reset takes about 8 s longer to reach SmartWRITER, plus
300 ms for each FujiNet device EOS polls.

`UART_TXD` is shared: the printer's transmitter drives it in mode 7, the FujiNet UART in mode 8.
Printing in mode 8 goes to FujiNet's own printer device instead of `mister_printerd`.

Simulator and tooling, no effect on the FPGA: `--fujinet` and the Bus-over-IP bridge in
`verilator/sim_fujinet.cpp`, `verilator/fujinet_test.sh`, `--no-media`.

## 3. The HPS side

All in `../FujiNet_MiSTer` (the same repo the CoCo3 FujiNet work lives in), none of it installed
on the MiSTer yet:

1. `dist/fujinet-adam/` is FujiNet-PC built for the ADAM target for the ARM (`./build-docker.sh
   ADAM`). Copy it to `/media/fat/fujinet/adam/`. Its `fnconfig.ini` is `config/fnconfig-adam.ini`:
   serial path on `/dev/ttyS1`, Bus-over-IP off, four disk slots, the usual TNFS hosts.
2. `scripts/uartmode.fujinet` replaces `/sbin/uartmode` (the root filesystem is read-only; the
   script's header has the remount). Its mode 8 picks `/media/fat/fujinet/<platform>/` from
   `/tmp/CORENAME` (`adam` for this core, `coco` for the CoCo3), writes the OSD's UART speed into
   that config's `[Serial] baud`, and serves `/media/fat/games/Adam` as host 1 (SD). The hand-edited
   mode 8 already on the MiSTer only knows the single `/media/fat/fujinet/` install.
3. Main must know mode 8 ("FujiNet" in the UART connection list): the `printer-fujinet` Main
   from the CoCo3 test package (`MiSTer.fujinet-20260925`, md5 `72152e87`). It only reads the
   saved `config/uartmode.<core>` when the core's `CONF_STR` has a `UART` line, which this branch
   has and the 20260922 release does not, and it ignores a saved mode 8 unless
   `/media/fat/fujinet/fujinet` exists. `config/uartmode.Adam` = `08 00 00 00` and
   `config/uartspeed.Adam` = three little-endian words with 115200 in the third preset the mode
   and speed without the OSD.
4. `patches/0002-adam-serial-port-and-baud-from-config.patch` is already in the build: upstream's
   ADAM serial path probed `/dev/ttyUSB0` then `/dev/ttyS0` at a fixed 62,500 baud.

## 4. Hardware checklist

As of 2026-11-08 everything in section 3 is installed on **mister192** (`ssh mister`): the
FujiNet Main (the old one is `/media/fat/MiSTer.before-fujinet-20261107`), `/sbin/uartmode` with
mode 8 (the stock one is `/media/fat/linux/uartmode.stock-20261107`), the daemon in
`/media/fat/fujinet/adam/` and a copy in `/media/fat/fujinet/`, `config/uartmode.Adam` and
`uartspeed.Adam` preset to FujiNet at 115200, and `_AdamTests/F1 FujiNet boot.mgl`, which loads
the core and resets it 10 s later. The MiSTerKVMS CLI captures its screen
(`--mister 10.0.2.192 --stream 'mkvm://10.0.2.128/Mister%20192'`).

OSD: **System menu > UART connection = FujiNet, speed 115200** (the MidiLink list, as for the
CoCo3). Floppy 1 unmounted. The core and FujiNet-PC must agree on the speed: `uartmode` writes it
into `fnconfig.ini` each time it starts the daemon. Watch from the MiSTer with
`tail -f /media/fat/fujinet/fujinet.log`. In the OSD, Up from the top goes to **exit**, not Reset;
Up twice is Reset.

- [x] **Loading the core boots CONFIG** (HOST SLOTS: SD, fujinet.online, apps.irata.online,
      fujinet.diller.org). Passed 2026-11-08 on mister192 with `ColecoAdam_fujinet_20261108c.rbf`
      from a plain `load_core`, no reset needed. In the log: `ADAMNET SETUP`, `TTY set_baudrate:
      115200`, the reset broadcast (`UNHANDLED device=0x0f type=0x00`, harmless), `BLOCK: 0`,
      then 33 blocks read twice each at about 100 ms a read; CONFIG is up in 10 s. The `b` build
      missed it once: Main took 7 s to have the daemon listening again after a core load and the
      core gave up after 8 s; `c` waits 20 s. If the daemon's log says `TTY set_baudrate: 57600`,
      its config lost the `[Serial] baud` line (an older daemon dropped it when CONFIG saved).
- [ ] **If it boots to SmartWRITER instead:** check `pidof fujinet` and the daemon's start in
      `fujinet.log`; with the daemon stopped (`rm /tmp/uartmode8; killall fujinet`),
      `stty -F /dev/ttyS1 115200 raw; cat /dev/ttyS1 | xxd` shows what the core sends.
- [x] **Mount and boot a disk from a TNFS host.** Passed 2026-11-08: fujinet.online, `ADAM/`,
      `weather.ddp` into slot 1 read-only (keys: Return on the host, Return on the folder, Return
      on the file, `1`, V, then VI to boot). The daemon fetches each block over TNFS the first
      time EOS reads it, about 0.4 s, and 0.1 s for EOS's second read; the image was running in
      25 s. `[WiFi] enabled=1` is required in `fnconfig.ini` or the mount stalls after resolving
      the host.
- [x] **A network program.** Passed 2026-11-08 for plain HTTP: `weather.ddp` opens
      `N:HTTP://api.ipstack.com/...` on network device 9, FujiNet-PC fetches it (result 200,
      157 bytes), the program sets the JSON parser and reads latitude, longitude, city, country
      and region, closes, and asks the gateway for the time. Three firmware fixes were needed
      first (patch 0003 in `FujiNet_MiSTer`; `TODO.md` 0a). Its next two requests are HTTPS
      and fail inside the daemon's TLS on the ARM build ("TLS handshake: -0x7780"), so the
      program ends at PARSING WEATHER DATA with nothing to parse. Fixed later the same day: the failures were firmware regressions (a line feed
      and a 256 byte cut in the URL, patch 0003), and the forecast now shows; only
      api.geotimezone.com fails, its server refusing TLS to every client.
- [ ] **ADAM CP/M from FujiNet** (`20261109`): CONFIG, apps.irata.online (or SD with
      `cpm22.dsk`), CP/M 2.2 into slot 1, boot: the `A>` prompt, and `DIR` lists the disk. The
      tester saw it pass on their board with the same three changes.
- [ ] **Loading from the menu, five times**: CONFIG each time, never a black screen. Then with the
      UART connection None: SmartWRITER after about 1.5 s.
- [ ] **One daemon after reloads**: load the core three times, `pidof fujinet` shows one pid
      (the `uartmode` token fix).
- [ ] **A floppy with an image mounted stays local**: mount a local DSK in floppy 1, reset: it
      boots as before, and FujiNet's slot 1 is not seen. Mount it in floppy 2 instead: CONFIG
      boots from FujiNet's slot 1 and the local disk is drive 2.
- [ ] **Printer**: print from SmartWRITER in FujiNet mode; the output appears in FujiNet's web UI
      (`http://mister.local:8000/`, printer page), not as a `mister_printerd` PDF.
- [ ] **UART connection None / Printer**: SmartWRITER boots after the 1.5 s load hold (no 8 s wait), the printer
      still prints to `mister_printerd` in mode 7.
- [ ] **Keyboard during a FujiNet load**: the ADAM stays responsive; keys typed while CONFIG loads
      are not lost (the master holds the Z80 off its bus only for the 1 KB copy).

## 5. Backing out

| Change | Edit |
|---|---|
| 15 FujiNet | OSD UART connection to anything but FujiNet: `fuji_en` is low and `cv_adamnet` emulates every device as before. No rebuild. To take it out of the build, the three new files and the `fuji_*` ports are the whole of it |

## 6. Simulation results at handoff

- `verilator/fujinet_test.sh` boots FujiNet's CONFIG from the native FujiNet-PC ADAM build at
  115,200 baud: the reset broadcast, the gateway probe, EOS's 15-device scan (every status
  packet's checksum good), the CONFIG boot loader's 33 blocks each read twice as EOS does, and
  CONFIG's own gateway traffic (WiFi status, host slots, device slots). HOST SLOTS is on screen
  by frame 600, with no framing errors and no bus-state assertion in FujiNet-PC's log.
- The block FujiNet served was compared byte for byte with the disk image at C800h.
- Selecting fujinet.online in CONFIG mounts it over TNFS and lists its directory (ADAM/, APPLE2/,
  ATARI/, ...), so the character device path, the 258 byte OPEN DIRECTORY write included, works.
- 2026-11-09, with the CP/M fixes: CP/M 2.2 through CONFIG from FujiNet-PC's SD host reaches
  `A>` (it stopped blank before them), CONFIG boots in 34 block fetches, weather.ddp shows its
  forecast; with the stand-in (`--fuji-disk`) CP/M reaches `A>` at 300 ms a block, with every
  third CLR lost, and with the UART mode 60 ms late. SmartWRITER without FujiNet is unchanged.
- With FujiNet off the ADAM scenario sweep against ColEm matches the committed baseline scenario
  for scenario (`verilator/compare/work/adam_fuji`); Diablo's `worst_fg` 0.9611 to 0.9647 is the
  R register change already in the branch, not this one.
