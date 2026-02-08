# CS452 K3

There are no special requirements to build the program beyond running `make` on the `linux.student.cs` environment, which generates the image at `kitty_kernel.img`.

Once the program is loaded and executed on the RPi, a welcome message starting with `Kitty kernel version` is printed from the kernel.

The kernel then starts the first task that creates tasks for the name server, the clock server, and the four client tasks required by the assignment. Each client performs a send to the first task, which replies with the corresponding delay interval and count. Each client task then prints a message of the form `tid: <tid>, delay interval: <delay interval>, delays completed: <count>` after each delay.

Every 500ms, before the next context switch the kernel prints the fraction of time spent in the idle task since the last print, in the format `Idle: <percent>%`.
