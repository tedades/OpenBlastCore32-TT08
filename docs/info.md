## How it works

OpenBlastCore32-TT08 RC1 is a 32-bit transport-triggered CPU with external
cartridge memory, monochrome framebuffer scanout, gamepad inputs and mono audio.
It requires a 50 MHz reference clock. Video and audio use a fixed 25 MHz clock
enable; memory arbitration can pause the CPU without changing display timing.

Instructions are fetched from external quad-SPI Flash. Data and the framebuffer
reside in external PSRAM, sharing the cartridge clock and four data lines with
Flash through separate chip-select signals. There is no built-in application or
full on-chip framebuffer.

The display reads a packed 160 by 120, one-bit-per-pixel framebuffer, with
20 bytes per row and 2,400 bytes per frame. Two 20-byte on-chip row buffers feed
640 by 480 VGA output using fourfold scaling in both directions. Each pixel
selects one of two programmable RGB222 colors. Sync timing has 800 clocks per
line and 525 lines per frame, approximately 59.52 Hz at the specified clock.
Audio provides two pulse voices and one noise voice through a mono PWM output.
The gamepad interface reads the dual-controller PMOD signals; two additional
inputs provide synchronized GPIO. PS/2 reception and the reserved turbo input
are not implemented in RC1.

## How to test

Select the project on a compatible Tiny Tapeout demo board, attach the cartridge,
and supply the 50 MHz clock. Use the pin assignments in `info.yaml`. Assert
active-low reset for several reference-clock cycles before releasing it.
Reset disables framebuffer display and mutes audio; VGA sync continues after
reset is released, but a blank screen alone does not demonstrate CPU execution.

Program Flash with an application matching this CPU's compressed instruction
encoding and quad-SPI read protocol. No application image is included. A useful
test program first writes and reads back PSRAM, then fills a framebuffer with
alternating all-zero and all-one rows. With display disabled, write the
four-byte-aligned PSRAM framebuffer byte address to CPU I/O register `0x6a`.
Write the two six-bit RGB222 colors to `0x70` and `0x71`, then write `1` to
`0x6b` to enable display. Expect alternating colored horizontal bands, each four
output lines high. Register `0x10` supplies a frame counter; reading `0x6b`
returns enable in bit 0, sticky underrun in bit 1, fetch error in bit 2 and
refill activity in bit 3. Writing bit 1 of `0x6b` clears the sticky faults;
also set bit 0 if display should remain enabled.

Use the application to read controller state at `0x20` while pressing buttons,
and GPIO state at `0x2a` while changing the two GPIO inputs. To test audio,
configure a pulse voice at `0x30` or `0x31` with a nonzero period in bits 15:0,
nonzero volume in bits 19:16 and enable in bit 20. Write zero to `0x33` to
unmute and listen through a filtered audio interface. `0x32` configures noise.

## External hardware

A compatible Tiny Tapeout demo board with a 50 MHz clock, a cartridge PMOD with
compatible quad-SPI Flash and PSRAM, and a VGA resistor-DAC PMOD with a compatible
display are required for the memory and display tests. Use the dual-NES gamepad
PMOD and controllers for input tests. Audio requires a suitable PWM low-pass
filter and amplifier or line-level interface; do not connect a speaker directly
to the chip output. PS/2 hardware is not required.
