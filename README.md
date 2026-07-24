# SMS_MiSTer — RetroAchievements Fork

This is a fork of the official [SMS_MiSTer](https://github.com/MiSTer-devel/SMS_MiSTer) core with **RetroAchievements** support for Sega Master System and Game Gear on MiSTer FPGA.

> **Status:** Experimental / Proof of Concept — requires the modified [Main_MiSTer binary](https://github.com/odelot/Main_MiSTer) to function.

## How to Test

Pre-built core binaries are available on the [Releases](https://github.com/odelot/SMS_MiSTer/releases) page — no compilation or Quartus needed.

1. Download the `.rbf` core file from the latest release.
2. Copy the core file to the `/media/fat/_Console` folder on your MiSTer SD card.
3. You will also need the modified Main_MiSTer binary from [odelot/Main_MiSTer](https://github.com/odelot/Main_MiSTer) (see its README for setup instructions, including RetroAchievements credentials).

## What's Different from the Original

The [upstream SMS_MiSTer](https://github.com/MiSTer-devel/SMS_MiSTer) core emulates the Sega Master System, Game Gear, and SG-1000. This fork adds the FPGA-side infrastructure needed to expose emulated RAM to the ARM binary for RetroAchievements evaluation. All original core features are preserved.

### Files Added

| File | Purpose |
|------|--------|
| `rtl/ra_ram_mirror_sms.sv` | State machine that reads emulated RAM (System RAM + NVRAM) and writes it to DDRAM using the Selective Address protocol |
| `rtl/ddram_arb_sms.sv` | DDRAM bus arbiter — shares access between the framebuffer (screen rotation) and the RA mirror |

### Files Modified

| File | Change |
|------|--------|
| `SMS.sv` | System RAM and NVRAM converted from single-port to dual-port (`dpram`), RA mirror and arbiter instantiated, NVRAM Port B muxed between SD card saves and RA reads |

### How the RAM Mirroring Works

The Master System has a Z80-based 8-bit architecture with a relatively small memory map. This core uses the **Selective Address protocol** (Option C): the ARM binary writes a list of addresses it needs to evaluate, and the FPGA reads only those values from the emulated RAM and writes them back to DDRAM.

**Memory regions exposed:**

| Region | RA Address Range | Size | Source |
|--------|-----------------|------|--------|
| System RAM | `$0000–$1FFF` | 8 KB | Z80 RAM (`$C000–$DFFF`) via dual-port BRAM Port B |
| NVRAM / Cart RAM | `$2000–$9FFF` | up to 32 KB | Cartridge SRAM via dual-port BRAM Port B |

**Key implementation details:**

- **Dual-port RAM conversion** — The original single-port System RAM and NVRAM were converted to dual-port (`dpram`). Port A remains dedicated to the CPU; Port B is read-only and used by the RA mirror. This means RAM reads for achievements never stall or interfere with the running game.
- **NVRAM Port B muxing** — NVRAM Port B is shared between SD card save/load operations (`bk_state` active) and RA mirror reads (when `bk_state` is idle). The RA mirror only accesses NVRAM when no save operation is in progress.
- **DDRAM arbitration** — A dedicated arbiter (`ddram_arb_sms.sv`) shares the DDRAM bus between the primary master (framebuffer / screen rotation) and the RA mirror. The RA mirror only gets bus access when the primary master is idle.
- **Toggle handshake protocol** — Communication between the RA mirror and the DDRAM uses a toggle req/ack protocol that cleanly handles clock domain crossing without complex synchronization logic.
- **No byte swapping** — Unlike 68K-based cores (Genesis), the Z80's 8-bit little-endian architecture requires no byte reordering.

**Per-VBlank flow:**
1. On VBlank, the RA mirror reads the address request list from DDRAM (`0x40000`).
2. For each address, it dispatches to either System RAM or NVRAM via the dual-port Port B.
3. Values are collected 8 bytes at a time into 64-bit words and written to the DDRAM response cache (`0x48000`).
4. A response header with the current frame counter is written so the ARM can detect new data.

---

## Original Features (preserved from upstream)

* Sega Master System, Game Gear, SC-3000 and [SG-1000](https://en.wikipedia.org/wiki/SG-1000) Support
* [Sega System E arcade hardware](https://segaretro.org/Sega_System_E) Support
* NTSC & PAL Support
* Hide Borders Option - Allows you to fill the screen vertically without black borders.
* FM Audio Support
* Extra Sprites Option
* Cheats
* Extended Game Gear Resolution Option
* Z80 Turbo Option
* Lightgun, Paddle controls, Keyboard(SK-1100) and Multitap Support
* Gear to Gear link cable over USERIO
* BIOS Loading Support
* Savestates

## Notes

* Some games come in .gg format but are in fact SMS games. Rename the .gg extension to .sms or .bin to fix them. These games are mostly listed in this page [SMSpower-SMS-GG list](http://www.smspower.org/Tags/SMS-GG).
* The "Aspect ratio" doesn't do much in PAL mode, that's normal.
* The "Region" parameter toggle some hardware features that are specific to the different console models. Some localized games need these modifications to work properly. If a game doesn't work right, try to toggle this setting and reset the game in order to troubleshoot.
* Each game cartridge comes with a specific mapper, which description is not included in the .gg ou .sms file. The core has a special logic to automatically determine which mapper needs to be used, but some games make a good effort to make this logic fail. The "Mapper" parameter permits to force the usage of specific mappers in case the automatic detection fails.
* The "Masked left column" option controls behaviour of left column when hidden by system (usually during horizontal scrolling). "BG" sets it to the background/overscan colour, as on original hardware. "Black" makes it black, which may look better on non full-screen settings as the column will blend in with surrounding black area. "Cut" will remove the column from the active image, so the horizontal resolution becomes 248 instead of 256. This will distort the image when scaled, particularly on integer scaling settings, but will use more of the screen. When "Border" is set to "Yes" the left column is always shown as part of the border, so "Masked left column" is disabled.
* Regular ROMs savestates are persistent in the SD card. BIOS built-in games will save to memory only (non-persistent). Workaround: load the BIOS and an empty ROM.

### Gear To Gear USERIO Mapping

| GG Signal    | Cable Pin	 | USERIO Pin|
| -------- | ------- | ------- |
| PC4 / TX  | 6    |USER_IO[1]|
| PC5 / RX | 9     |USER_IO[2]|
| PC0   | 1   |USER_IO[0]|
| PC1   | 2   |USER_IO[3]|
| PC2   | 3   |USER_IO[4]|
| PC3  | 4   |USER_IO[5]|
| PC6 / NMI  | 7   |USER_IO[6]|
| GND | 8   |GND|

