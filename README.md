# PoC - Dual Wheel, Single Axis, Course Correction.

This repo, consists of the following areas:
- v2: microbit-v2 robot-controller code. It can:
  - return debugging data over USB serial
  - receive commands over BLE
  - course correct using the onboard compass
  - course correct, using a tilt compensated compass
  - tilt-compensate, using the onboard accelerometer and magnetometer
  - run filtering like:
    - Low Pass Filter (LPF)
    - Normalization and value-out-of-bounds rejection
    - Kalman Filter [TODO]
    - Input Compensation (Predictive) Model [TODO]
- controller: [TODO] an android app that:
  - gives me some basic movement controls over the bot
  - can visualize the robot's orientation in 3D space
  - probably will be a separate app

In the commit [#d7d5261](../../tree/d7d5261dfeeb25a5737e7ae32004225df6961dff/.old) the .old directory contains portions of the project I no longer use:
- ~~v1      : microbit-v1 remote-controller code that I use to control the robot-controller~~
- ~~v2-rust : microbit-v2 robot-controller code port to rust.~~
- ~~fz      : flipper zero applet that I use for collecting data from the robot, since it has BLE and a shiny microSD card slot :3~~

Should I overcome my depression plateu I'll update this README properly and include some pictures of the bot.
