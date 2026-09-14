# _emonTH3_ Firmware

This describes the firmware provided for the [_emonTH3_](https://github.com/openenergymonitor/emonTH3) wires temperature, humidity, co2 and pulse counting unit.

This firmware is intended to be used with the [OpenEnergyMonitor](https://openenergymonitor.org) platform. Hardware systems are available directly from them.

## Getting in contact

### Problems

Issues can be reported:

- As a [GitHub issue](https://github.com/openenergymonitor/emonTH3-fw/issues)
- On the [OpenEnergyMonitor forums](https://community.openenergymonitor.org/)

Please include as much information as possible, including at least:

- The emonTH3 hardware that you using and the emonTH3 firmware version (run the 'v' command on the serial link)
- All settings (run the `l` command on the serial link)
- A full description, including a reproduction if possible, of the issue

## Functional Description

### Version information

The firmware version numbering follows [semantic versioning](https://semver.org/). That is, for version `X.Y.Z`:

- `X` : major version with no guaranteed backward compatibility with previous major versions
- `Y` : minor version where any added functionality has backward compatibility
- `Z` : improvements and bug fixes

Any firmware with `X == 0` is considered unstable and subject to change without notice.

> [!NOTE]
> Build information, including compiler version and commit, is generated during the build process and included in the binary.

### Hardware serial connection

A UART is provided for configuration and, optionally, data transmission. It has the following configuration:

- 115200 baud
- 8N1

### Hardware Configuration

#### DIP Switches

- The DIP switches set the node ID for the device. They are read at power on.
- Each DIP switch addess adds 1 to the base node ID (default 27):

  | Switch 1 | Switch 2 | ID         | Default Base-ID=27 |
  |----------|----------|----------  |------------------|
  | OFF      |  OFF     |  base-ID   |27 |
  |  ON      |  OFF     |  base-ID+1 |28 |
  | OFF      |  ON      |  base-ID+2 |29 |
  | ON       | ON       | base-ID+3  |30 |


### Run time configuration

When the emonTH3 is first powered on or reset, the **STATUS** LED indicator will slowly pulse for 5 seconds. If any character is received over the UART connection in this time, the emonTH3 will enter configuration mode. It is not possible to configure the emonTH3 outside this period.

> [!NOTE]
> All options can be listed by entering `?`.

The following options are available through the serial configuration interface.

| Command | Description | Arguments |
|---------|-------------|-----------|
| `?` | Show help text | None |
| `a<n> <m>` | Configure the SCD4x CO2 sensor | `n`: sample interval (s)<br>`m`: altitude above sea level (m) |
| `c<n>` | Enable UART output | `0`: off, `1`: on |
| `d<n>` | Set the data acquisition period | `n`: period value |
| `e<n>` | Set the number of external temperature sensors | `0`, `1`, or `4` |
| `f` | Exit configuration mode and continue boot | None |
| `j<n>` | Enable JSON serial format | `0`: off, `1`: on |
| `l` | List settings as key/value pairs | None |
| `lh` | List settings in human readable form | None |
| `m <x> <y> <z>` | Configure pulse counting | `x`: `0` off, `1` on<br>`y`: `n` no pull, `d` pull down, `u` pull up<br>`z`: minimum pulse period (ms) |
| `n<n>` | Set node ID | `[1..60]` |
| `p<n>` | Set RF power level | `n`: RF power level |
| `r` | Restore defaults | None |
| `s` | Save settings to NVM | None |
| `t<x> <yy> <yy> <yy> <yy> <yy> <yy> <yy> <yy>` | Change an external sensor's position | `x`: sensor position in the list (1-based)<br>`yy`: hexadecimal address bytes, e.g. `28 81 43 31 07 00 00 D9` |
| `v` | Print firmware and board information | None |
| `w<n>` | Enable wireless | `0`: off, `1`: on |
| `x<n>` | Set 433 MHz compatibility | `0`: `433.92 MHz`, `1`: `433.00 MHz` |

### Run time

- The LED indicator will flash for the first 5 transmissions then be disabled for power saving
- By default UART output is disabled during runtime. It can be enabled during runtime by sending `c1` over the serial interface.
- The default transmission period is 55s. This can be changed during runtime by sending `d<n>` over the serial interface, where `n` is the period in seconds.


## EmonHub Decoders

The emonTH3 requires the following emonHub decoder in `emonhub.conf`:

Assuming default node ID of 27.

### For none or one external temperature sensor
```
    [[27]]
        nodename = emonth3_27
        [[[rx]]]
            names = temperature, external temperature, humidity, battery, pulsecount, co2
            datacodes = h, h, h, h, L, h
            scales = 0.1, 0.01, 0.1, 0.01, 1, 1
            units = C, C, %, V, p, ppm
```

### For four external temperature sensors
```
    [[27]]
        nodename = emonth3_27
        [[[rx]]]
            names = temperature, external temperature1, external temperature2, external temperature3, external temperature4, humidity, battery, pulsecount, co2
            datacodes = h, h, h, h, h, h, h, L, h
            scales = 0.1, 0.01, 0.01, 0.01, 0.01, 0.1, 0.01, 1, 1
            units = C, C, C, C, C, %, V, p, ppm
```

## CO<sub>2</sub> Sensor

- CO<sub>2</sub> sensor is optional and is not fitted to the board by default.
- The CO<sub>2</sub> add-on board is connected to the board via the I2C interface, the **CO<sub>2</sub> board should be oriented towards the antenna**.
- The sensor is a SEK-STCC4 from Sensirion [datasheet](https://sensirion.com/media/documents/6AED4B15/69295E41/CD_DS_STCC4_D1.pdf).
- The CO<sub>2</sub> sensor requires 20s at startup to "recondition".
- The CO<sub>2</sub> sensor will be automatically enabled at startup, if fitted.

## Compiling and uploading

### Compiling

Compiling the firmware requires the the [Arm gcc toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain) (may be available as a package in your distribution). The Makefile is for a Cortex-M23 based microcontroller, specifically the Microchip ATSAML10E15 ([datasheet](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ProductDocuments/DataSheets/SAM-L10-L11-Family-Data-Sheet-DS60001513.pdf), [errata](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ProductDocuments/Errata/SAM-L10-L11-Family-Silicon-Errata-and-Data-Sheet-Clarification-DS80000795.pdf)).

To install the toolchain on Ubuntu:

```bash
sudo apt-get install gcc-arm-none-eabi
```

Full linux install guide: https://developer.arm.com/documentation/110477/221/Installation

> [!NOTE]
> To find which version, if any, of the toolchain is on your path, enter `arm-none-eabi-gcc --version`. You can set the path to a compiler off your path by setting the `TC_PATH` variable in `Makefile`.

To build the firmware:

  `> make -j`

In `bin/`, the following binary files will be generated:

- `emonTH-vX.Y.Z-(commit[-dirty]).bin`
- `emonTH-vX.Y.Z-(commit[-dirty]).elf`
- `emonTH-vX.Y.Z-(commit[-dirty]).hex`

The `-dirty` tag (if present) indicates that there are uncommitted changes when the binaries are built.

### Uploading

The emonTH3 is supplied with a [serial bootloader](https://github.com/openenergymonitor/bootloader_uart_saml10/) installed.

- To enter the bootloader, press the **BOOT** button while powering on the emonTH3. The LED will blink rapidly to indicate it has entered the bootloader.

- Follow the instructions for uploading the firmware in the [bootloader repository](https://github.com/openenergymonitor/bootloader_uart_saml10/).

## Modifications

### Helper scripts

> [!NOTE]
> A Python virtual environment should be setup by running `python3 -m venv venv && source venv/bin/activate && pip3 install -r requirements.txt` in `./scripts/`.

- `build_info.py`: generates `src/emonTH_build_info.c` during the build with the git revision, compiler version, build time, machine, and release metadata embedded in the firmware.
- `version_info.py`: derives the versioned output filename from the firmware version in `src/emonTH.h` and the current git revision.
- `elf_size.sh`: runs `elf-size-analyze` on `build/emonTH.elf` to break the image down by function size.
- `led_pulse.py`: generates the `ledIntensity[]` lookup table used for the startup LED pulse effect.

### Compile Time Configuration

There are no compile time configuration options.

### Assertions

Assertions are [implemented](https://interrupt.memfault.com/blog/asserts-in-embedded-systems) by the **EMONTH_ASSERT(_condition_)** macro. The microcontroller will enter a breakpoint when an assertion fails and the PC is stored in the `g_assert_info` variable. The PC is used to find the file and line where the assertion failed using `arm-none-eabi-addr2line`.

## Hardware Description

### Peripherals (SAML10)

The following table lists the peripherals used in the SAML10.

|Peripheral       | Alias           | Description                   | Usage                             |
|-----------------|-----------------|-------------------------------|-----------------------------------|
|ADC              |                 |Analog-to-digital converter    |Acquire battery voltage            |
|DMAC             |                 |DMA Controller                 |UART transmission                  |
|EIC              |                 |External interrupt controller  |External device sense              |
|PORT             |                 |GPIO handling                  |                                   |
|SERCOM0          |SERCOM_I2CM      |I2C                            |I2C for internal peripherals       |
|SERCOM1          |SERCOM_SPI       |SPI                            |Drives RFM module                  |
|SERCOM2          |SERCOM_UART      |UART                           |Configuration and data UART        |
|TC0              |TIMER_LP         |Timer/Counter (16bit)          |Low power timing                   |
|TC1              |TIMER_PULSE      |Timer/Counter (16bit)          |Pulse timing / mask window         |
|TC2              |TIMER_DELAY      |Timer/Counter (16bit)          |Delay counter, 8 us resolution     |

### Designing a new board

The files `./src/board_def.h` and `./src/board_def.c` contain options for configuring the microcontroller for a given board. Pin mappings and peripheral usage will need to be adjusted to your design.

### Porting to different microcontroller

Within the top level loop, there are no direct calls to low level hardware. You must provide functions that handle the hardware specific to the microcontroller you are using.

All peripheral drivers are in header/source pairs named **driver_\<PERIPHERAL\>**. For example, the ADC driver is in **driver_ADC.\***. If you are porting to a new microcontroller, you will need to provide implementations of all the functions exposed in **driver_\<PERIPHERAL\>.h** and any internal functions within **driver_\<PERIPHERAL\>.c**. If your microcontroller does not support a particular function (for example, it doesn't have a DMA), then either no operation or an alternative must be provided.

You will also need to ensure that the vendor's headers are included and visible to the compiler.

## Contributing

Contributions are welcome! Small PRs can be accepted at any time. Please get in touch before making _large_ changes to see if it's going to fit before spending too much time on things.

> [!TIP]
> A [clang-format](https://clang.llvm.org/docs/ClangFormat.html) autoformat pattern is included in the repository. Run the `install-hooks.sh` script to install the pre-commit hook to the autoformatter. You may need to install `clang-format` using your OS's package manager.

> [!NOTE]
> Please bear in mind that this is an open source project and PRs and enhancements may not be addressed quickly, or at all. This is no comment on the quality of the contribution, and please feel free to fork as you like!

## Acknowledgements

### Third party libraries and tools

- [mcu-starter-projects](https://github.com/ataradov/mcu-starter-projects) - good starting point for build chains for microcontrollers.
- [RFM69](https://github.com/LowPowerLab/RFM69) - RFM69 driver from Low Power Labs used as reference.
- [Using Asserts in Embedded Systems](https://interrupt.memfault.com/blog/asserts-in-embedded-systems) - custom assertions from _Interrupt by Memfault_.
- [Wintertools](https://github.com/https://github.com/wntrblm/wintertools) - various build and linker scripts from Winterbloom.

### Others

- Glyn Hudson @ [OpenEnergyMonitor](https://openenergymonitor.org/)
- Trystan Lea @ [OpenEnergyMonitor](https://openenergymonitor.org/)
