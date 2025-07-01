# TrackballMouse
Arduino code for a trackball or mouse with up to 5 buttons and a scroll wheel

This code is intended to allow a person to design and program their own pointing device. The following options are available:
 - trackball or mouse
 - wired or wireless
 - up to 5 buttons
 - optional scroll wheel

The hardware required for this consists of:

 - an Arduino-compatible MCU (I've tested this with Adafruit's RP2040, BLE M0 and BLE 32u4 Feathers)
 - a PMW3610 breakout board such as the ones [here](https://www.tindie.com/products/randware/pmw3610-breakout-board/) and [here](https://www.etsy.com/listing/1776906095/zmk-compatible-trackball-mouse-sensor) or, if you'd rather build your own, use the PCB files from [here](https://github.com/siderakb/pmw3610-pcb)
 - a mechanical scroll wheel encoder such as the [ALPS EC10E series](https://tech.alpsalpine.com/e/products/detail/EC10E1260502/) (assuming scroll wheel is required)
 - a tactile or micro switch for each button
 - a trackball ball and some ball bearings to support it (for trackball)
 - some mouse glides / skates (for mouse)
 - a battery or battery pack (for wireless)
 - a USB cable (for wired)

In addition, you'll need some sort of enclosure. I designed and 3D printed mine.
Some guidelines on this:
 1. For a trackball configuration, the PMW3610 breakout board should be oriented lens up, with the 6-pin J1 connector on the right
 2. For a mouse configuration, the PMW3610 breakout board should be oriented lens down, with the 6-pin J1 connector on the left
 3. In either case, the distance between the lens and the surface / ball is a bit sensitive; the diagrams on page 4 of [this document](https://trackballs.eu/media/Nakabayashi/Digio2/PMW3610DM-SUDU.pdf) should help
**NOTE**: WHen using glides/skates on a mouse, the thickness of those must be taken into account when positioning the PMW3610

One thing I ran into was that the BLE microcontrollers I had contained firmware that required updating; I recommend updating to at least version 0.8.1 *before* programming the microcontroller

I can't guarantee what I've got here is the most elegant code, but it does seem to work. I've used it to build a wired trackball, a wireless mouse, and a wireless scroll wheel.

![image](https://github.com/user-attachments/assets/5490d0b3-3706-4cd9-af40-ee7ead688813)

