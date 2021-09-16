# robotic-arm-mod

This repository is a slightly modified version of [this](https://www.thingiverse.com/thing:3327968) project. I made some modifications.
Firstly, the base motor gearbox was changed to strain wave gear instead of planetary gearbox to reduce losses and backlash I plan to change all the gearboxes eventually.
I also changed the placement and type of motor drivers, they are all in single box with the microcontroller, the wiring is a bit more messy, but I didnt have other options at the time.
The firmware uses SDfat library, it is capable to read from SDcard, txt files that contain Gcode-like instructions. It can also be controlled with the same "G-code" over the serial bus.
those changes made the build slightly less expensive.

If i have time in the future I will overhaul/completely remake this project, but for the time being there is no time for that.
