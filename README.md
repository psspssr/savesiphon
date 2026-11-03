# SaveSiphon

Back up the save from a physical **Game Boy Advance cartridge** on an **Analogue Pocket**. No PC, no cart reader. Insert the cart, launch the core, and the save lands on the SD card as a `.sav` that works in mGBA and other emulators.

```
SAVESIPHON
GAME  POKEMON EMER
CODE  BPEE
SAVE  FLASH 128K

DONE - SAVE DUMPED
[################################]

FILE
POKEMON_EMER-BPEE.sav
```

## Status

**v0.1, early.** The logic passes simulation against cartridge models for every save type below. It has **not yet been confirmed on real hardware for every type**, so treat it as a beta and keep your cart (the original save is never modified, see [Safety](#safety)).

| Save type | Size | Typical games | Status |
|---|---|---|---|
| SRAM / FRAM | 32 KiB | Golden Sun, many RPGs | simulated |
| Flash | 64 KiB | Advance Wars, Fire Emblem | simulated |
| Flash | 128 KiB | Pokémon Ruby/Sapphire/Emerald/FR/LG | simulated |
| EEPROM | 512 B | Super Mario Advance, many platformers | simulated |
| EEPROM | 8 KiB | Minish Cap, Metroid Fusion | simulated |

Hardware reports welcome. Please open an issue with the game, the save type shown on screen, and whether the `.sav` loads in an emulator.

## Install

**pupdate:** once SaveSiphon is listed in the openFPGA inventory, run pupdate and it appears with the other cores.

**Manually:** download `psspssr.SaveSiphon_<version>.zip` from [Releases](../../releases) and unzip it into the root of the SD card, merging the `Assets`, `Cores` and `Platforms` folders. On macOS, copy the folders' *contents*, because Finder replaces folders instead of merging them.

Requires current Pocket firmware (2.x).

## Use

1. With the Pocket **powered off**, insert the GBA cartridge.
2. Power on and open **openFPGA → Tools → SaveSiphon**.
3. For Flash 128K (Pokémon) and EEPROM games, press **A** at the confirm prompt.
4. Wait for `DONE - SAVE DUMPED` (about 5 seconds; most of that is the ROM scan).
5. Your save is at `/Assets/savesiphon/common/<TITLE>-<CODE>.sav`.

Dumping again never overwrites: the next dump of the same game becomes `<TITLE>-<CODE>_1.sav`, then `_2`, and so on. Press **A** to dump again. Always power off before swapping cartridges.

## Safety

SaveSiphon is built to **only read** your cartridge. Several independent layers enforce that:

1. **Read-only by design.** SRAM and 64 KiB Flash saves, and the whole ROM, are read with plain read strobes. The bus engine has no code path that writes there.
2. **Two narrowly-defined commands, hard-coded.** Only two save types need anything sent to the cart:
   * **Flash 128 KiB bank select**: `AA→5555, 55→2AAA, B0→5555, 00|01→0000`. These are constants. No path can emit an erase (`80`) or program (`A0`) command, and the chip is returned to bank 0 afterwards.
   * **EEPROM read requests**: the serial pattern `1 1 <address> 0`. The second bit is a constant `1`, so an EEPROM *write* command (`1 0 …`) cannot be formed.
3. **You confirm first.** For those two types the core stops and asks: **A** to continue, **B** to cancel. The bus is released while it waits.
4. **Independent hardware write guard** (`cart_write_guard.v`). It sits on the WR# pin itself and does not trust the sequencer. Every write strobe is checked against the exact allowed sequence using only the values on the pins: address, data, chip selects, and EEPROM bit position. Anything else never reaches the cartridge, and the core latches **SAFETY STOP** until restarted. It also stops on bus contention (CS1#+CS2# or RD#+WR# low together) and on any signal moving mid-strobe.
5. **Clean strobes even when interrupted.** If the core is reset mid-write, WR# is raised while the data is still held, so a cut-off strobe can't latch a floating value.
6. **Header validation.** A cartridge whose header fails the logo, fixed-byte and checksum checks is never touched beyond reading the header.
7. **Verified dumps.** Every save is read twice and both CRC32s must match (up to 3 attempts). Otherwise it reports **UNSTABLE READ** and writes nothing.
8. **Swap and power-loss detection.** The header is re-read after dumping and must be identical (**CARTRIDGE CHANGED** otherwise). Losing slot power halts the core at once.
9. **Never overwrites dumps.** Existing files on the SD card are kept, and new dumps get `_1`, `_2`, ….
10. **Pins released when idle.** Once a dump finishes or fails, the core stops driving the cartridge bus.

The simulation suite (`sim/run_all.sh`) covers all of this. It injects a corrupted Flash command and an EEPROM write command into the sequencer and checks the guard blocks both before they reach the cart model. It also checks that a swapped cart, unstable contacts and a cancel at the prompt each stop without writing a file.

## How it works

1. Reads the GBA header and validates it.
2. Scans the ROM for the save-library string Nintendo's SDK links into every retail game (`SRAM_V`, `FLASH_V`, `FLASH512_V`, `FLASH1M_V`, `EEPROM_V`). GBA headers carry no save type, so this is how every dumper does it.
3. Reads the save over the cartridge bus. For EEPROM, 512 B and 8 KiB chips are told apart by reading 64 blocks with 14-bit addressing: a 512 B chip only decodes 6 bits and answers the same block every time.
4. Opens a file through APF target commands (`0x0192` open/create, `0x0184` write) and writes the buffer.

Source: `src/fpga/core/`, mainly `siphon_ctrl.v` (sequencer), `gba_cart_bus.v` (bus timing) and `status_screen.v`.

## Build

CI builds every push with Quartus Lite 21.1 in Docker (`raetro/quartus`). Pushing a `v*` tag publishes a release zip.

Locally:

```sh
sim/run_all.sh                       # cartridge-model simulations (needs iverilog)
cd src/fpga && quartus_sh --flow compile ap_core
scripts/package.sh src/fpga/output_files/ap_core.rbf 0.1.0 dist
```

## Credits

* Analogue's [openFPGA core template](https://github.com/open-fpga/core-template) (`src/fpga/apf`, bridge and PLL).
* The hardware notes in [openfpga-carttools](https://github.com/kroy-the-rabbit/openfpga-carttools) documented the Pocket's GBA pin mapping, the EEPROM block-alias size probe, and the APF file-path conventions that work on real hardware. If you also want ROM dumps or GB/GBC support, use that core.
* [font8x8](https://github.com/dhepper/font8x8) by Daniel Hepper (public domain).

## License

MIT for the SaveSiphon sources. `src/fpga/apf/` and the template-derived files are provided by Analogue under their own terms.
