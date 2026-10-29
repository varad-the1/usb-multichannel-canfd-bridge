# USB multichannel CAN FD bridge

An FPGA bridge between a PC and up to 8 CAN FD controllers, over a single USB link (FT2232H).

- **PC to CAN:** the PC sends a frame over USB with a channel number in front. The bridge sends it to that channel's CAN FD core and transmits it on the bus.
- **CAN to PC:** frames received by a core are returned to the PC over USB, each tagged with the channel number it came from, so frames from different channels can always be told apart, even when they are identical.
- **Standard and extended IDs**, classic CAN and CAN FD, up to 16 data words per frame.
- **Configured over USB:** nothing is built in. The first frame the PC sends is a configuration frame that sets how many channels are active, the bit timing, mode, acceptance filters and other settings. Data sent before that is dropped.
- **One bus master per channel:** each channel has its own AXI4-Lite port, wired straight to its own CAN FD core. The number of channels built is set with the `NUM_CH` parameter (1 to 8).

Built with the AMD CAN FD LogiCORE (PG223) in Vivado 2025.2. The bridge RTL is in `files/`, and the Vivado project is `axi4_block.xpr`.

## USB frame

32-bit words, each sent little-endian, in both directions:

`channel, ID, DLC, N, data...`

`N` is the number of 32-bit data words (0 to 16).

## Configuration frame

Sent first, with `0xFFFFFFFF` in the channel position:

`0xFFFFFFFF, M, (register, value) x M`

A register is `{channel, index}`; channel `0xFF` means all channels.

| Index | Setting |
|---|---|
| 0x00 | number of active channels |
| 0x01 - 0x04 | bit timing (BRPR, BTR, FBRPR, FBTR) |
| 0x05, 0x06 | mode (MSR), interrupt enable (IER) |
| 0x07 - 0x0B | acceptance filters (AFR, AFMR0/1, AFIR0/1) |
| 0x0C | RX FIFO watermark |
| 0x0D | auto-start the channels when the frame ends |
| 0x0E | start the channels now |
| 0x0F | receive on or off, per channel |
| 0x10 | CAN FD transmit buffer, per channel |

A channel starts only once its bit timing has been sent.

## Simulation

Two testbenches: `files/tb_can_fd_usb.v` tests the bridge on its own, and `axi4_block.srcs/sim_1/new/tb_bridge_top.sv` runs it with the real CAN FD cores. The system testbench reads frames and configuration lines from `can_frames.txt` (channel 0) and `can_frames_2.txt` (channel 1), and writes what comes back, tagged by channel, to a text file.
