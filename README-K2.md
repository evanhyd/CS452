# CS452 K2

There are no special requirements to build the program beyond running `make` on the `linux.student.cs` environment, which generates the image at `kitty_kernel.img`.

Once the program is loaded and executed on the RPi, a welcome message starting with `Kitty kernel version` is printed from the kernel.

The kernel then starts a task that initializes the name server and the RPS server. It then goes through three tests:
1. 2 interactive clients: The user is prompted twice (once per client) to enter an RPS move.
2. Low-priority automatic clients: 32 non-interactive clients (that choose moves according to current timestamp) with priority lower than the server.
3. High-priority automatic clients: 32 non-interactive clients with the same high priority as the server.
Before each test, a message is printed, and it waits for the user to press a key to continue.

Upon the exit of all user tasks, the kernel prints `All tasks exited. Press any key to reboot...` and waits for a key press to reboot.
