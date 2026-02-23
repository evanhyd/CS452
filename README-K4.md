# CS452 K4

There are no special requirements to build the program beyond running `make` on the `linux.student.cs` environment, which generates the image at `kitty_kernel.img`.

Once the program is loaded and executed on the RPi, the kernel starts the first task that creates all other user tasks, reimplementing the behaviour of A0 using the facilities of the microkernel. The interface shows a digital clock, switch states, recent sensor events, command history, a command prompt, and, new in K4, the idle time percentage, updated every second.

As in A0, the available commands are:
- `tr <train number> <train speed>` Set train speed (0-14, 0 = stop)
- `rv <train number>` Reverse train direction
- `sw <switch number> <switch direction>` Throw switch to straight (S) or curved (C)
- `q` Quit program and reboot
