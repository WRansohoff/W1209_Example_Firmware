# Overview

Test program for the cheap STM8-based W1209 temperature control board.

Currently just a test firmware. It can toggle the pin connected to the relay and LED, or draw to the 3-digit 7-segment LED displays.

The resistive input and button pins are initialized, but unused.

# Running it

Build with SDCC:

sdcc -lstm8 -mstm8 --out-fmt-ihx --std-sdcc11 main.c

Flash with the 'stm8flash' project and a generic STLink/v2 module:

stm8flash -c stlinkv2 -p stm8s003f3 -w main.ihx

I haven't figured out on-chip debugging with OpenOCD yet.
