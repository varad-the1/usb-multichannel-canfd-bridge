# USB multichannel CAN FD bridge

An FPGA bridge between a PC and one or more CAN FD controllers, over a single USB link (FT2232H).

- **PC to CAN:** the PC sends a frame over USB with a channel number in front. The bridge sends it to that channel's CAN FD core and transmits it on the bus.
- **CAN to PC:** frames received by a core are returned to the PC over USB, each tagged with the channel number it came from, so frames from different channels can always be told apart, even when they are identical.
- **Standard and extended IDs**, classic CAN and CAN FD, up to 16 data words per frame.
- **Automatic setup:** after reset, each CAN core is configured on its own (bit timing, mode, acceptance filters) and then enabled.
- **Any number of channels:** one CAN FD core per channel, set with the `NUM_CH` parameter.

Built with the AMD CAN FD LogiCORE (PG223) in Vivado 2025.2. The bridge RTL is in `files/`, and the Vivado project is `axi4_block.xpr`.

## USB frame

32-bit words, each sent little-endian, in both directions:

`channel, ID, DLC, N, data...`

`N` is the number of 32-bit data words (0 to 16).
