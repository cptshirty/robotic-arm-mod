# robotic-arm-mod

This repository is a slightly modified version of [this](https://www.thingiverse.com/thing:3327968) project. I made some modifications.
Firstly, there base motor gearbox was changed to strain wave gear instead of planetary gearbox to reduce losses and backlash.
I also changed the placement and type of motor drivers, they are all in single box with the microcontroller, the wiring is a bit more messy, but I didnt have other options at the time.
also the firmware is changed, it is capable to read from SDcard txt files that contain "G-code".
those changes should make the build slightly less expensive.
