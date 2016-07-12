Post URC 2016
=============
Some notes on the state of the rover modules:


ArmProcess
----------
The controls were threwn together the hour before the competition,
and while joystick input from the game pad seems to work, the buttons do not.
I2C packets seem to send properly and are recieved by the Arduino on the other end.
The arduino code for comunications is also buggy; see /embedModules/README.md for details.

CameraProcess
-------------
Not implemented. Would be a bridge between the Python and Teensy.

{Drill,StorageBin}Process
-------------------------
Not tested in the field. Drill seemed to work fine in the lab before we left for Utah.
The Teensy on the drill may be fried.

DriveProcess
------------
Some tweaks were made before the compute module board got swapped out,
but the current code works well enough to drive.
