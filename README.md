# wii-ra-adapter

**RetroAchievements on original Wii hardware — for both Wii and GameCube games — with no PC, no emulator and no game patching.**

The wii-ra-adapter is an ESP32-S3 firmware. The board plugs into the Wii's GameCube **memory card Slot B**, pretending to be an EXI device. It connects to your Wi-Fi, logs into [RetroAchievements](https://retroachievements.org), and runs the official [rcheevos](https://github.com/RetroAchievements/rcheevos) client — while the Wii, ~60 times per second, streams snapshots of the running game's RAM to it over the EXI bus. Achievements pop on real hardware, on real discs/backups, in real time.

This is the Wii/GameCube member of a family of adapters that started with the [nes-ra-adapter](https://github.com/odelot/nes-ra-adapter).

> **This repository is the distribution point**: releases here bundle the binaries of **all four projects** needed for a working setup (ESP32 firmware + WiiFlow Lite + d2x cIOS + Nintendont).

## The four projects

Getting achievements working on an unmodified game requires cooperating software in three places: the loader (PowerPC), the memory server (ARM/Starlet, running next to the game), and the brain (ESP32). Four projects cover both consoles:

| Project | Runs on | Role |
|---|---|---|
| **wii-ra-adapter** (this repo) | ESP32-S3 | The brain. Wi-Fi + RetroAchievements login, runs `rc_client`, decides which memory addresses to watch, evaluates achievements every frame, hosts a live web dashboard. |
| **[WiiFlow Lite (fork)](https://github.com/odelot/WiiFlow_Lite)** | Wii (PowerPC loader) | The front end for **Wii games**. Detects the adapter, computes the game's RA hash on-console, performs the `LOAD_GAME` handshake before booting, injects the VBlank hook + trophy overlay, and offers RA settings (including adapter credential reset). |
| **[d2x cIOS (fork)](https://github.com/odelot/d2x-cios)** | Wii (ARM/Starlet, IOS) | The memory server for **Wii games**: a new `ra-module` inside the custom IOS. Every VBlank it reads the watched addresses from game RAM (MEM1/MEM2), ships a snapshot to the ESP32 over EXI, and relays unlock events (LED blink + on-screen trophy). |
| **[Nintendont (fork)](https://github.com/odelot/Nintendont)** | Wii (ARM/Starlet, kernel) | The equivalent for **GameCube games**: an `ra_module` inside the Nintendont kernel. Computes the GC RA hash from the disc/ISO, does the handshake and the per-frame snapshot loop itself — loader, memory server and celebration (LED + overlay + rumble) all in one place. |

All four talk the same compact binary protocol ([`gc_ra_protocol.h`](main/gc_ra_protocol.h)), and the ESP32 firmware is console-agnostic: it identifies the game by its RA hash and lets the console side worry about where the bytes live.

## How it works

```
                        Wii console                                ESP32-S3 (memory card Slot B)
 ┌─────────────────────────────────────────────────┐             ┌────────────────────────────────┐
 │  PowerPC (game, unmodified)                     │             │  Core 1 (realtime)             │
 │   └─ C0 VBlank hook: frame counter + trophy     │   EXI bus   │   ├─ EXI/SPI slave             │
 │                                                 │  (SPI-like) │   └─ rc_client_do_frame()      │
 │  ARM "Starlet" (runs beside the game)           │◄───────────►│                                │
 │   ├─ Wii games: ra-module inside d2x cIOS       │  snapshots  │  Core 0 (network)              │
 │   └─ GC games:  ra_module inside Nintendont     │   events    │   ├─ Wi-Fi + HTTPS to RA       │
 │      (reads game RAM every VBlank)              │             │   └─ web dashboard (wii-ra.local) │
 └─────────────────────────────────────────────────┘             └────────────────────────────────┘
```

1. **Boot** — the loader (WiiFlow for Wii games, Nintendont for GC games) probes EXI channel 1 for the adapter (`IDENTIFY`), computes the game's RetroAchievements MD5 hash on-console, and sends `LOAD_GAME`.
2. **Load** — the ESP32 fetches the achievement set from the RA servers, compiles the list of memory addresses the achievements reference, and hands the console a **watchlist**.
3. **Play** — every VBlank the Starlet-side module reads the watched bytes straight out of the running game's RAM and sends a **snapshot**. Snapshots are queued on the ESP32 so no frame is lost, and each one is fed to `rc_client_do_frame()`, exactly like an emulator would with direct memory access.
4. **Unlock** — when an achievement triggers, the ESP32 submits it to RetroAchievements and sends an event back: the disc-slot LED blinks, a trophy indicator is drawn on screen, and (on GameCube) the controller rumbles. Connected browsers get a live toast via WebSocket.

The tricky part — achievements that chase **pointer chains** through RAM — is handled by a chain-descriptor table: the ESP32 compiles every eligible pointer chain into a flat table the console fetches once per game, so the console *walks the chains itself* in fresh RAM each frame and no round-trips are wasted resolving moved pointers. The watchlist also grows/shrinks incrementally at runtime, kept replica-consistent on both sides by sequence numbers.

### Wii games vs GameCube games

| | Wii games | GameCube games |
|---|---|---|
| Loader / handshake | WiiFlow Lite (before the IOS reload, while the PPC still owns EXI) | Nintendont kernel (background thread, game already running) |
| Memory server | `ra-module` in d2x cIOS | `ra_module` in the Nintendont kernel |
| RA hash | Computed on-console from the WBFS/ISO image (`rc_hash_wii_disc` port), with a game-ID fallback table on the ESP32 | Computed in-kernel from the disc/image (`rc_hash_gamecube` algorithm) |
| Frame sync | Gecko C0 VBlank hook injected by WiiFlow increments a MEM1 frame counter the ra-module polls (timer fallback if absent) | VBlank-synced from the Nintendont kernel |

## Hardware

Any ESP32-S3 board with PSRAM works; two board variants are wired in [`board_config.h`](main/board_config.h):

- **BOARD_DEV** — ESP32-S3 DevKit (WS2812 RGB status LED)
- **BOARD_XIAO** — Seeed XIAO ESP32S3 (tiny — fits inside a memory card shell)

Wiring to the GameCube memory card connector (Slot B):

| MC pin | Signal | S3 DevKit | XIAO ESP32S3 |
|---|---|---|---|
| 7 | 3.3V | 3.3V | 3.3V |
| 1 | GND | GND | GND |
| 3 | CS | GPIO10 | GPIO7 (D8) |
| 4 | CLK | GPIO12 | GPIO8 (D9) |
| 5 | DI (Wii → ESP) | GPIO11 | GPIO9 (D10) |
| 6 | DO (ESP → Wii) | GPIO13 | GPIO5 (D4) |
| 2 | INT | GPIO14 | GPIO6 (D5) |

The ESP32 acts as SPI slave on the EXI bus. The per-frame snapshot path (Wii games, d2x ra-module) runs at **16 MHz**; the boot handshake (WiiFlow) and the Nintendont module currently run at 8 MHz. A single EXI transaction carries up to 8 KB, enough for a full snapshot of 6144 watched addresses plus pointer-chain data.

## First-time setup

1. Flash the firmware (or grab a release binary) and plug the adapter into memory card **Slot B**.
2. On first boot the adapter opens a Wi-Fi access point named **`WII_RA_ADAPTER`**. Connect with your phone and enter your Wi-Fi credentials and your RetroAchievements username + password at the captive portal.
3. Install the bundled **d2x cIOS** (for Wii games) and copy the bundled **WiiFlow Lite** and/or **Nintendont** to your SD/USB as usual.
4. Boot a game from WiiFlow (Wii) or Nintendont (GC). The loader waits for the adapter to report *game loaded* before starting — typically 5–20 s on first load.

To re-enter setup (new Wi-Fi, new account), use **WiiFlow → Settings → Reset RA adapter credentials**; the adapter wipes its stored credentials and reboots into the portal.

### Live web dashboard

While a game is running, open **`http://wii-ra.local/`** from any device on the same network: game info, achievement list with live progress, unlock toasts, challenge indicators and rich presence — pushed over WebSocket, with zero cost to the console when no client is connected.

## Downloads

Each release on this repository bundles:

- `wii-ra-adapter` ESP32-S3 firmware images (per board variant)
- WiiFlow Lite build with RA support (`boot.dol`)
- d2x cIOS build with the `ra-module`
- Nintendont build with RA support (`boot.dol`)

Sources for the console-side projects live in their own repositories (forks of [wiidev/d2x-cios](https://github.com/wiidev/d2x-cios), [WiiFlow Lite](https://github.com/Fledge68/WiiFlow_Lite) and [FIX94's Nintendont](https://github.com/FIX94/Nintendont)).

## Credits

- [RetroAchievements](https://retroachievements.org) and [rcheevos](https://github.com/RetroAchievements/rcheevos) — the achievement runtime this project embeds
- [nes-ra-adapter](https://github.com/odelot/nes-ra-adapter) — the ancestor of this design
- The d2x cIOS, WiiFlow Lite and Nintendont communities, whose work makes running anything on this hardware possible
- [WiFiManager](https://github.com/tzapu/WiFiManager) for the credential portal

## Disclaimer

This project is intended for use with games you legally own. It comes with no warranty; you are responsible for anything you install on your console.
