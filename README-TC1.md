# CS452 TC1

There are no special requirements to build the program beyond running `make` on the `linux.student.cs` environment, which generates the image at `kitty_kernel.img`.

The interface shows a digital clock, switch states, recent sensor events, command history, a command prompt, the idle time percentage, and, new in TC1, a train state display that displays estimated speed, the last triggered sensor, the current estimated location, the distance to the destination, the error at the last tripped sensor, and a prediction for the next tripped sensor.

The new commands are:
- `goto <train id> <speed level> <location> <offset>` Send train to a track node (e.g. A5, BR15) with offset in mm.
- `st <a|b>` Set the track to track A or track B.
