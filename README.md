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
3. Wait for `DONE - SAVE DUMPED` (about 5 seconds; most of that is the ROM scan).
4. Your save is at `/Assets/savesiphon/common/<TITLE>-<CODE>.sav`.

Dumping again never overwrites: the next dump of the same game becomes `<TITLE>-<CODE>_1.sav`, then `_2`, and so on. Press **A** to dump again. Always power off before swapping cartridges.

## Safety

SaveSiphon only reads your save. The complete list of things it ever writes to a cartridge:

* **Flash 128 KiB bank select**: `AA→5555, 55→2AAA, B0→5555, 00|01→0000`. These are hard-coded constants. The design has no path that can emit an erase (`80`) or program (`A0`) command. The chip is returned to bank 0 afterwards.
* **EEPROM read requests**: the serial pattern `1 1 <address> 0`. The second bit is a constant `1`, so an EEPROM *write* command (`1 0 …`) cannot be formed.
* Nothing else. ROM space, SRAM and 64 KiB Flash are read-only paths. A cartridge whose header fails validation (logo bytes, fixed byte, header checksum) is never touched beyond reading the header.

The simulation testbench (`sim/`) fails if any other write ever reaches a cartridge model.

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
