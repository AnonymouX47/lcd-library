# LCD

My **self-written** libraries for **HD47780-driven** LCDs for PIC MPUs.
All written **from scratch**.

## Contents
The libraries include:
- 4-bit interface mode
- 8-bit interface mode
- 8-bit interface mode via i2c. (Coming Soon)

## Features

- It's written for 8-bit MPUs but can be easily adapted to others (if required).
- Efficient implementations.
- All HD47780 instructions are fully implemented.
- The same library for all interface modes. :smile:
- Basic display operations implemented.
- A bunch of error-checking.
- Easy-to-use interface.
- Relatable names of utilities.
- Unnecessary details hidden from main program.
- Custom Character Generation (coming soon).

## Functionalities

- Single Character display
- Character String display
- Integer dislay
- Floating-point display
- Backspace
- Cursor movements
  - Up & Down
  - Left & Right (With option for number of positions)
  - Home & End (within same line)
  - Home (first line beginning)
  - Abitrary Positon
- Reverse Cursor Direction
- Display Shift/Scroll: Left & Right (With option for number of positions)
- Scroll animation with following parameters:
  - Number of repetitions
  - Start and End Positions
  - Scroll speed

**NOTE:** All operations are **aware of display bounds** and employ a number of **input "error-checking"** methods.

