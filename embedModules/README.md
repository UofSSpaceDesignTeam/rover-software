Post URC 2016
=============
Some notes on some of the modules:

Arm
---
Communications is buggy. I2C packets seem to be recieved from the rover properly,
but the values aren't translated to motor movement.
Directly controlling the arm with the serial command prompt works fine.
There is likely something wrong with the `parseCommand` function,
so one may try directly changing `g_duty_cycle` in the `recieveCommand` function.
I'm not sure this will help however, as during debugging,
the duty cycle manager was setting the motors correctly, but the arm didn't move.
There may be a flag somewhere that was not set/reset properly or something.

Drill
-----
Was working in the lab, but failed at competition. Likely a fried Teensy. :frowning:


StorageBin
----------
Not tested in Utah, but should work.

Science sensors
---------------
Doesn't work properly. (If someone else can elaborate, that would be cool...)
