# Overview

Test program for the cheap STM8-based W1209 temperature control board.

Currently just a test firmware. It can toggle the pin connected to the relay and LED, draw to the 3-digit 7-segment LED displays, or read the raw ADC value on its resistive input and print the result to the 7-segment display.

The three button pins are initialized as inputs with pull-ups, but they are currently unused.

# Running it

Build with SDCC:

sdcc -lstm8 -mstm8 --out-fmt-ihx --std-sdcc11 main.c

Flash with the 'stm8flash' project and a generic STLink/v2 module:

stm8flash -c stlinkv2 -p stm8s003f3 -w main.ihx

I haven't figured out on-chip debugging with OpenOCD yet.
