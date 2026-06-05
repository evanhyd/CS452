# `CS 452: TC2 Documentation`

## `Evan He, e7he, 20946651`

## `Ian Zhao, i6zhao, 20988818`

## 

`Abstract	4`

[`Commit and SHA	4`](#commit-and-sha)

[`Building	4`](#building)

[`Train Controlling System	5`](#train-controlling-system)

[`Command	5`](#command)

[`TrainTrackState	6`](#traintrackstate)

[`Train	7`](#train)

[NavigationSystem	7](#navigationsystem)

[KinematicsSystem	7](#kinematicssystem)

[`PathFindingSystem	7`](#pathfindingsystem)

[Routing	8](#routing)

[Stopping At The Target	9](#stopping-at-the-target)

[Calibration Data	9](#calibration-data)

[Sensor Ownership And Tracking	9](#sensor-ownership-and-tracking)

[`Tasks	11`](#tasks)

[`TrainTrackTask (Priority 2)	11`](#traintracktask-\(priority-2\))

[`MarklinEventListenerTask (Priority 0)	11`](#marklineventlistenertask-\(priority-0\))

[`MarklinDispatchTask (Priority 2)	12`](#marklindispatchtask-\(priority-2\))

[`UIViewTask (Priority 2)	12`](#uiviewtask-\(priority-2\))

[`UIControllerTask (Priority 0)	12`](#uicontrollertask-\(priority-0\))

[Overall Design	12](#overall-design)

[`Kernel Architecture	13`](#kernel-architecture)

[`Components	13`](#components)

[`K4	13`](#k4)

[`MCP2515	13`](#mcp2515)

[`GPIO	13`](#gpio)

[`UART	13`](#uart)

[`IO Server (for UART)	13`](#io-server-\(for-uart\))

[`CAN Server	14`](#can-server)

[`K3	17`](#k3)

[`StaticPriorityQueue	17`](#staticpriorityqueue)

[`ClockServer	17`](#clockserver)

[`ClockNotifier	17`](#clocknotifier)

[`EventBlockedQueue[]	17`](#eventblockedqueue[])

[`K2	18`](#k2)

[`RingBuffer	18`](#ringbuffer)

[`StaticStack	18`](#staticstack)

[`TaskIdAllocator	18`](#taskidallocator)

[`MessageControlBlock	18`](#messagecontrolblock)

[`NameServer	18`](#nameserver)

[`K1	19`](#k1)

[`TaskDescriptor	19`](#taskdescriptor)

[`TaskScheduler	19`](#taskscheduler)

[`StackContext	19`](#stackcontext)

[`RoundRobinQueue	19`](#roundrobinqueue)

[`MultiLevelQueue<T>	19`](#multilevelqueue\<t\>)

[`SlabAllocator<T, SIZE>	19`](#slaballocator\<t,-size\>)

[`Control Flow	20`](#control-flow)

[`K4	20`](#k4-1)

[`CAN Interface	20`](#can-interface)

[`K3	21`](#k3-1)

[`GICC and GICD configuration	21`](#gicc-and-gicd-configuration)

[`AwaitEvent	21`](#awaitevent)

[`preIrqEntry	21`](#preirqentry)

[`irqEntry(StackContext* sp)	21`](#irqentry\(stackcontext*-sp\))

[`Idle Task	22`](#idle-task)

[`K2	22`](#k2-1)

[`Message Queue Design	22`](#message-queue-design)

[`Send	22`](#send)

[`Receive	23`](#receive)

[`Reply	23`](#reply)

[`K1	24`](#k1-1)

[`Task Creation	24`](#task-creation)

[`Context Switching	24`](#context-switching)

[`Kernel State	25`](#kernel-state)

## 

# `Abstract` {#abstract}

`TC2 contains:`  
	`A sophisticated train control system with a reservation system that plans multiple train’s paths while avoiding collision at real time. Multiple trains can run on the same train track simultaneously with random destinations assigned.` 

# `Commit and SHA` {#commit-and-sha}

`Repo: https://git.uwaterloo.ca/e7he/cs452_kernel`  
`Commit: 12d92499209b75b350cb4f9b847de49694481701`

# `Building` {#building}

`The kernel image at “kitty_kernel.img” is built by running “make”.`  
`There are no special requirements to build the project in the linux.student.cs environment.`

`The raw calibration data (stopping distance and speed measurements) is in the data/ subdirectory.`

# `Train Controlling System` {#train-controlling-system}

## `Command` {#command}

`tr <TrainId> <SpeedLevel>`  
	`Set the train speed.`

`st <TrackId>`  
	`Switch to track A or track B. This command must be performed at the beginning of the program. The track is defaulted to track B as track A is very finicky.`

`goto <TrainId> <SpeedLevel> <LocationName> <DistanceOffset>`  
	`Move the train to the location with an offset in millimeters. The location name is case sensitive. If the destination is a branch with positive distance offset, then the direction of the train depends on the switch direction.`

`wa <TrainId> <SpeedLevel>`  
	`Set the train into “Wandering” mode that travels to random destinations repeatedly.`

## 

## `TrainTrackState` {#traintrackstate}

`struct TrainTrackState {`  
`private:`  
  `uint32_t currentTrack;`  
  `std::array<Train, MAX_TRAIN_ID> trains{};`  
  `TrackSet trackA{};`  
  `TrackSet trackB{};`  
  `std::array<SwitchState, NUM_SWITCHES> switches{};`

`public:`  
  `TrainTrackState();`

  `void reset();`

  `Train& getTrain(TrainId id);`  
  `SwitchState getSwitchState(SwitchId id) const;`  
  `void setSwitchState(SwitchId id, SwitchState switchState);`  
  `void setCurrentTrack(TrackId id);`  
  `TrackId getCurrentTrackId() const;`

  `TrackNode& getTrackNodeById(TrackNodeId id);`  
  `TrackNode* getTrackNodeByName(const char* name);`  
`};`  
`A TrainTrackState is a central state manager that keeps track of the state of track nodes, switch directions, and train’s kinematics and navigation states.`

## 

## `Train` {#train}

`struct Train {`  
  `NavigationSystem navigation{};`  
  `KinematicsSystem kinematics{};`  
  `SensorPredictionSystem prediction{};`  
`};`

`Each train contains a kinematic state representing the motion of the train, a navigation state represents the intent of the train, and a prediction system that predicts the next triggered sensor.`

### `NavigationSystem` {#navigationsystem}

`Manual: The train is controlled by the user.`  
`FindingPath: The train is finding a path to the destination.`  
`Routed: The train is following the path.`  
`Reversing: The train is reversing its direction.`

### `KinematicsSystem` {#kinematicssystem}

`Lost: The train’s kinematics state is unknown.`  
`Tracked: THe train’s kinematics state is known.`

## `PathFindingSystem` {#pathfindingsystem}

`PathFindingSystem is responsible for reserving the track nodes and changing the switches’ directions for the train every tick.`

## `Routing` {#routing}

`When a routing request arrives, the train enters FindingPath state in src/user_tasks/train_track_handler.h. Once the train has a tracked position, the path planner in src/marklin/marklin_pathfinding.h runs a Dijkstra search from the train's current estimated location to the destination. The search:`

- `explores all legal outgoing edges from each node,`  
- `treats nodes reserved as the destination of another train as impassable,`  
- `adds a penalty for nodes with queued reservation waiters, so the planner biases away from congested regions,`  
- `accepts either the requested destination orientation or its reverse orientation, whichever is reached first.`

`If a path is found, the planner reconstructs it from the parent map, stores the ordered path nodes, and reserves each node. The reservation system consists of a FIFO queue for each node. A node is considered enterable by a train if the train is the next in the queue for that node. The destination node is locked separately and considered impassable for other trains.`

`If no forward path is found, the train sets a needToReverse flag, stops, reverses direction once its estimated speed reaches zero, re-seeds its tracked position on the reverse node, and tries to plan again. If no path exists even after reversing, the request fails unless the train is in wandering mode, in which case a new random destination is chosen.`

`While the train is moving, the planner continuously updates the route state. It also proactively sets branch directions for the next few path nodes ahead of the train, which avoids flipping a switch underneath the train.`

## `Stopping At The Target` {#stopping-at-the-target}

`Stopping is handled by the path planner's runtime state machine, which moves each routed train between Moving, Yielding, Arriving, and Idling.`  
`The planner recomputes the enterableDistance based on how many track nodes ahead it can enter every tick to decide when the train should yield or arrive at the destination.`

- `In Moving, the train keeps its commanded cruise speed.`  
- `If the reserved distance ahead becomes too short to stop safely before an unenterable track, the train enters Yielding and is commanded to speed 0.`  
- `If the destination itself is within stopping range, the train enters Arriving and is also commanded to speed 0.`  
- `Once the estimated speed reaches zero in Arriving state, the train releases its reservations and returns to Idling.`

`There are also small overshoot and undershoot corrections near dangerous topology, to avoid directly stopping the train over a switch.`

## `Calibration Data` {#calibration-data}

`Calibration data is stored in:`

- `data/speeds.txt`  
- `data/stopping_distance.txt`  
- `src/marklin/marklin_measured_data.h (computed from other data)`

## `Sensor Ownership And Tracking` {#sensor-ownership-and-tracking}

`When a sensor event arrives, the train-control server must decide which train caused it. The ownership heuristic in src/user_tasks/train_track_handler.h uses three levels:`

1. `The train is currently holding the reservation on that sensor's node.`  
2. `A train whose predicted next sensor matches the event.`  
3. `A train currently in Lost kinematics state.`

`After ownership is resolved, the server logs the event, updates the train's kinematics from the measured inter-sensor distance and elapsed ticks, and finally predicts the next sensor and when it should occur.`

## 

## `Tasks` {#tasks}

`![][image1]`

### `TrainTrackTask (Priority 2)` {#traintracktask-(priority-2)}

`Train track server is the central train control system that manages the track and train states. It is responsible for all the train track state updates and path finding.`

### `MarklinEventListenerTask (Priority 0)` {#marklineventlistenertask-(priority-0)}

`Marklin event listener task listens to the CAN receive buffer, and decodes the CAN event data into user defined event type, and forwards the event data to the train track server and the marklin dispatch server..`

### `MarklinDispatchTask (Priority 2)` {#marklindispatchtask-(priority-2)}

`Marklin dispatch server is responsible for managing all the network CAN messages. This includes waiting for the CAN response, and tracks the network latency.`

### `UIViewTask (Priority 2)` {#uiviewtask-(priority-2)}

`The UI view server receives UI updating request messages from other tasks, and updates the screen pixel on demand. The UI prints out the system timer, switch states, sensor events, command history, and train states such as last triggered sensor name, estimated position, estimated speed, and estimated remaining distance to the destination.` 

### `UIControllerTask (Priority 0)` {#uicontrollertask-(priority-0)}

`The UI controller task receives keyboard inputs and parses the inputs into commands. If the command is valid, it forwards to the train track server for execution.` 

## `Overall Design` {#overall-design}

`The design is intentionally centralized. The train-control server is the single owner of train state and makes all routing and stopping decisions. This avoids excessive message patching to communicate shared state. The dispatcher isolates CAN protocol details. The event listener isolates raw Marklin feedback handling. The clock helper gives the control loop a regular tick. This keeps the logic easy to reason about:`

* `sensor events correct the model,`  
* `timer ticks advance the model and reevaluate actions,`  
* `the path planner decides whether the train should continue, yield, arrive, or reverse,`  
* `the dispatcher converts those decisions into Marklin commands.`

# `Kernel Architecture` {#kernel-architecture}

## `Components` {#components}

## `K4` {#k4}

#### `MCP2515` {#mcp2515}

`MCP 2515 controller is a wrapper around the CAN bus protocol, responsible for translating the signals. CanIO interrupt ID 145 is enabled in the GIC.`

#### `GPIO` {#gpio}

`General Purpose Input Output. It provides hardware pins that trigger interrupts when certain events occur. For instance, MCP2515 interrupt sets PIN 17 to low and triggers the exception handler.`

#### `UART` {#uart}

`Universal Asynchronous Receiver/Transmitter. It allows the user to communicate with the terminal. UartIO Interrupt ID 153 is enabled in the GIC.`

#### `IO Server (for UART)` {#io-server-(for-uart)}

`IO Servers consist of an ioServerTask, a getcNotifierTask, and a putCNotifierTask following the server-notifier pattern.`

`ioServerTask contains a getcBuffer that stores all the inputs from the UART device to the client, a putcBuffer that stores all the output from the client to the UART device, and a getcWaitingQueue to store all the blocked waiting tasks that request to receive a char from the UART.`

`There are 4 IoServerMessageTypes: GetcRequest, PutcRequest, GetcNotify, PutcNotify.`  
`GetcRequest:`  
	`The client requests to get the input from the UART device. If there’s a char in the getcBuffer, then reply with the getcBuffer’s char, otherwise it blocks and adds to the getcWaitingQueue.`

`PutcRequest:`  
	`The client requests to send an input char to the UART device. If the device is available, then it sends to the UART device immediately. Otherwise, the ioServerTask stores the input at the putcBuffer and processes it later.`

`GetcNotify:`  
	`The getcNotifierTask awaits the UART_RX event to wait for the receive buffer available. If yes, it sends the GetcNotify{input} message to the ioServerTask. The ioServerTask checks if there’s any task waiting in the getcWaitingQueue. If yes, then it replies to the task with the input value, otherwise, it adds to the getcBuffer.`

`PutcNotify:`  
	`The putcNotifierTask awaits the UART_TX event to wait for the transmit buffer available. If yes, it sends the PutcNotify{} message to the ioServerTask. The ioServerTask checks if there’s any input waiting in the putcBuffer. If yes, it sends the input from the putcBuffer to the UART device, otherwise, it marks the UART device ready for input.`

#### `CAN Server` {#can-server}

`Main challenges in implementing the CAN servers.`  
`Problem:`  
`If task 1 calls a MCP operation that involves a SPI transaction, then no other concurrent task should call an MCP operation involving a SPI transaction, or else risk corrupting the SPI busline. This design constraint implies the kernel and the CAN notifier must not accidentally preempt the CAN servers’s SPI transaction with another SPI transaction such as read MCP status or enable/disable the MCP interrupt. It is possible to bypass such limitations by carefully scheduling the tasks and managing the interrupt states, but such a solution leads to extreme complexity and it is difficult to reason about the correctness of the program.`

`Solution:`   
`To avoid falling into the complexity trap, only the CAN server can call a MCP operation involving a SPI transaction. The program controls the CAN interrupt via GPIO interrupt registers instead of MCP interrupt registers, which bypasses the SPI transaction limitation, and hence can be used outside of the CAN server.`

`Question:`  
`Why is there only a single CAN notifier in contrast to UART’s input and output notifiers?`  
	  
`Answer:`  
`There’s no convenient way to determine the MCP receive and transmit buffer status without using SPI transactions. If we want to avoid falling into the complexity trap, then the MCP server is the only place where we can check the status.`  
`In addition, both the read and transmit share the same SPI bus line to exchange bytes bidirectionally under the hood. Therefore, the CAN server needs only 1 CAN notifier as only either receive or transmit operations, but not both, can happen at a given time.`

`Implementation:`  
`CAN servers consist of a canServerTask and a notifierTask with implementation similar to the IO Server. However, instead of checking the MCP IO status in the notifier task like the UART ioNotifierTask, the MCP IO status must be checked inside the canServerTask.`

`There are 3 CanServerMessageTypes: ReceiveRequest, TransmitRequest, ReadyNotify.`

`ReceiveRequest:`  
	`The client requests to get the input from the MCP2515. If there’s a message in the receiveBuffer, then it replies with the receiveBuffer's message; otherwise, it blocks and adds the client tid to the receiveWaitingQueue.`

`TransmitRequest:`  
	`The client requests to send a message to the MCP2515. If the transmit buffer is available, then it sends to the transmit buffer immediately. Otherwise, the canServerTask stores the input message at the transmitBuffer and processes it later.`

`ReadyNotify:`  
	`The notifier awaits the CanIO event from the GPIO PIN 17. If an event occurs, the irq_handler disables the GPIO event, and then the notifier sends a ReadyNotify{} message to the canServerTask. The canServerTask checks the MCP2515 interrupt flag to see if TX0, RX0, or RX1 is available. If any are true, it performs the corresponding action, similar to the IO server, and acknowledges the MCP2515 interrupt.`

### 

### `K3` {#k3}

#### `StaticPriorityQueue` {#staticpriorityqueue}

`A min priority queue with fixed buffer size. The user can push a new item to the priority queue and pop the item with the lowest priority. Internally, the priority queue uses binary heap implementation with a O(log(n)) complexity of push and pop, where n is the number of items in the priority queue.`

#### `ClockServer` {#clockserver}

`A clock server is a task that handles time interface functions. Internally, it consists of a loop that Receive() a serialized Message object. A Message object consists of an enum type and a string buffer. The clock server checks the Message.type, which can be a tick update from the notifier, a Time request, or a Delay/DelayUntil request. In the last case, the server does not immediately reply to the task, instead maintaining delayed tasks in a priority queue sorted by the tick deadlines sent by the task. When the clock server receives a tick update, it keeps popping the priority queue until all the requests with expired deadlines have been replied to. The priority queue can contain a maximum of 128 tasks at a time.`

#### `ClockNotifier` {#clocknotifier}

`A helper server that awaits the timer interrupt event, and sends the tick update message to its parent task, the clock server. This allows the clock server to “multiplex” timer interrupts and client requests by combining them both into a single Receive() call.`

#### `EventBlockedQueue[]` {#eventblockedqueue[]}

`An array of cyclic queues indexed by the interrupt event id. All the tasks awaiting the same interrupt event type are enqueued into the same queue.`

### `K2` {#k2}

#### `RingBuffer` {#ringbuffer}

`A FIFO cyclic queue with static size. The user can push a new item to the back, and pop an item from the front.`

#### `StaticStack` {#staticstack}

`A LIFO stack data structure with static size.`

#### `TaskIdAllocator` {#taskidallocator}

`The kernel task manager has gone through a complete overhaul compared to K1. Task id now consists of a generation number and an index number instead of a shared globalTidCounter. The index ranges from 0 - 127 that can be used to index into an array of TaskDescriptor in constant time. This eliminates the needs of slab allocators to manage the TaskDescriptors and task stack frames. To avoid task id duplication, each index has a generation counter that increments when the corresponding task id is freed.`

#### `MessageControlBlock` {#messagecontrolblock}

`A message control block contains: the message to send, the size of the message to send, the receive buffer, the size of the receive buffer. Receive buffer can be used for both receiving the sending messages and receiving the replying message. Both the message to send and the receive buffer are non-owning pointers.`

#### `NameServer` {#nameserver}

`A name server is a task that maps a string name to a task id and provides utilities for name lookup. Internally, it consists of a loop that Receive() a serialized Message object. A Message object consists of an enum type and a string buffer. The name server checks the Message.type, invokes the corresponding handler, then finally Reply() with the query result. We chose to limit the number of entries to 128, and the maximum name length to 64. Since there should realistically be few tasks that need to act as servers and thus have names, this is sufficient.`

### `K1` {#k1}

#### `TaskDescriptor` {#taskdescriptor}

`TaskDescriptor is the handle to an allocated task. It contains all the meta data such as the taskId, priority, parent, runState, stackPointer, etc. Added in K2: messageControlBlock and sendWaitQueue. The latter holds all the tasks that are waiting to be sent to this task.`

#### `TaskScheduler` {#taskscheduler}

	`TaskScheduler is a singleton class that provides operations to manipulate the task queues.`

#### `StackContext` {#stackcontext}

	`A placeholder class to define the saved context layout of each user task.`

#### `RoundRobinQueue` {#roundrobinqueue}

	`` A circular linked list that iterates through all the items in a round robin format. Internally, it stores a set of TaskDescriptor, and uses TaskDescriptor’s intrusive data structure’s link field to to form an order.` ``

#### `MultiLevelQueue<T>` {#multilevelqueue<t>}

	`A template class that holds an array of task queues indexed by task’s priority. It is a wrapper class that has the same operations as the internal queue, and selects the corresponding queue based on task’s priority.`

#### `SlabAllocator<T, SIZE>` {#slaballocator<t,-size>}

	`A template class that allocates an array of fixed size memory blocks. Internally it uses a free list to track the available memory blocks. Each freed memory block is an intrusive data structure, and are linked together via an internal linkage. When the memory block is allocated, it is reinterpreted as an array of unused bytes.`

## `Control Flow` {#control-flow}

### `K4` {#k4-1}

`The K4 kernel does not require significant changes in control flow from K3, as we only implement new event/interrupt types. The burden of using these new interrupt types is on the I/O and CAN servers, which were described in the K4 subsection of the “Components” section above.`

`The three new event types are “UartRX”, “UartTX”, and “CanIO”. “UartRX” and “UartTX” are tied to the UART interrupts. RX and TX are enabled/disabled separately by setting/clearing the appropriate bits of the UART IMSC register. When a UART interrupt arrives, the kernel determines if it was an RX or TX event, unblocks the corresponding tasks, and disables the corresponding interrupt type.`

#### `CAN Interface` {#can-interface}

`There are a couple of reasons why there is only 1 event type, “CanIO”, for CAN bus I/O. As described before, there is only 1 notifier for the CAN server, whose only job is to notify the CAN server when anything changes state. It is up to the server, not the kernel or the notifier, to check the status, see what changed, and what needs to be done. This is to ensure that SPI I/O only happens in one place, to prevent interleaving of transactions. Another consequence of this is that the kernel cannot selectively enable/disable RX and TX interrupts, as this involves SPI transactions. Instead, we choose to toggle interrupts at the GPIO level, which toggles all CAN I/O interrupts. Therefore, there is only 1 event type.`

#### 

* `int ReceiveCAN(int tid, marklin::MMessage& msg)`  
  * `Wrapper for Send-ing a ReceiveRequest to the CAN server`  
* `int TransmitCAN(int tid, const marklin::MMessage& msg)`  
  * `Wrapper for Send-ing a TransmitRequest to the CAN server`

`Since our kernel and user tasks are both C++, we opt to use references to the marklin::MMessage struct in our interface.`

`One other change from K3 is how idle time is displayed. Previously, it was printed by the kernel. However, to prevent interleaving of ANSI escape sequences, this was changed in K4 into a dedicated GetIdle syscall, so that user tasks can query for the idle percentage and print it at their own accord.`

### `K3` {#k3-1}

#### `GICC and GICD configuration` {#gicc-and-gicd-configuration}

`The kernel main entry initializes GICD_CTLR and GICC_CTLR to enable the interrupt controllers as a sanity check. It then routes the interrupt to CPU core 0 by masking the GICD_ITARGETSR register. Finally, it enables the GICC interrupt interface through the GICD_ISENABLER register.`

#### `AwaitEvent` {#awaitevent}

`When task 1 calls the AwaitEvent(id) syscall, the program switches to the kernel mode.`  
`If the id is not recognized by the kernel as corresponding to a valid event, then it immediately returns -1.`  
`If the id is recognized by the kernel, then the TaskManager moves the task from readyQueue to the eventBlockedQueue[id].`

#### `preIrqEntry` {#preirqentry}

`A new entry to the vector table is added alongside the existing syscall entry for IRQs. This performs the exact same context saving as its counterpart for syscalls, except it branches to irqEntry(StackContext*) instead of syscallEntry(StackContext*).`

#### `irqEntry(StackContext* sp)` {#irqentry(stackcontext*-sp)}

`The irqEntry performs the following operations:`

1. `Updating the current TaskDescriptor’s stack pointer to the sp arguments similar to the system call handler.`  
2. `irqEntry reads GICC_IAR register to get the interrupt event id and set the interrupt state to active.`  
3. `irqEntry compares the interrupt id and calls the corresponding handler. For instance, timer interrupts wake up all the timer-event blocked tasks to the readyQueue, and resets the timer channel to the next absolute tick. The next tick is computed as an offset from the previous tick, not the time of handling, so our timer does not lose time.`  
4. `irqEntry deactivates the interrupt via the GICC_EOIR register, and then switches to the next task as determined by the scheduler.`

#### `Idle Task` {#idle-task}

`The idle task is created by the kernel and simply executes the “wfi” instruction in an infinite loop. Its tid is saved by the kernel, and on context switch, if the previous task was the idle task, it adds the time since the last context switch to an accumulator. Once a 500ms window elapses, on the next context switch, the kernel prints the percent of time spent by the idle task.`

### `K2` {#k2-1}

#### `Message Queue Design` {#message-queue-design}

`At any given time, a task can only be in one of the following queues: the kernel readyQueue, or other task’s sendWaitQueue. If the task is in the RECEIVE_BLOCKED or REPLY_BLOCKED state, it will simply be removed from the readyQueue.`

#### `Send` {#send}

`When task 1 sends a message to task 2, it enters the kernel, and the kernel sets task 1’s MessageControlBlock to Send()’s arguments.` 

`If task 2 is in the RECEIVE_BLOCKED state, then the kernel copies task 1’s message to task 2’s receive buffer, removes task 1 from the readyQueue and sets its state to REPLY_BLOCKED, and re-adds task 2 to the readyQueue and sets its state to READY.`

`If task 2 is not in the RECEIVE_BLOCKED state, the kernel moves task 1 from the readyQueue to task 2’s sendWaitQueue.`

#### `Receive` {#receive}

`When task 2 calls receive, it enters the kernel, and the kernel sets task 2’s MessageControlBlock to Receive()’s arguments.`

`If task 2’s sendWaitQueue is empty, then the kernel removes task 2 from the readyQueue and sets its state to RECEIVE_BLOCKED.`

`If task 2’s sendWaitQueue is not empty, say task 1 is at its head, then the kernel copies the task 1’s message to task 2’s receive buffer. It removes the first task (task 1) from the sendWaitQueue and sets its state to REPLY_BLOCKED.`

#### `Reply` {#reply}

`When task 2 calls reply(), it enters the kernel, and the kernel sets task 2’s MessageControlBlock to Reply()’s arguments.`

`If the tid refers to a task 1 that is in the REPLY_BLOCKED state, then the kernel copies task 2’s message to task 1’s receive buffer, and re-adds task 1 to the readyQueue and sets its state to READY.`

`If the tid does not reference a valid task in the REPLY_BLOCKED state, an error is returned.`

### 

### `K1` {#k1-1}

#### `Task Creation` {#task-creation}

`The kernel create a task with the following procedure:`

1. `Allocate a task id using TidAllocator. Internally it uses a stack data structure to track the freed task id.`  
2. `Allocate a stack frame implicitly by indexing into an array of pre-reserved stack frames with the allocated task id. Each stack frame size is 1 MB.`  
3. `Allocate a TaskDescriptor implicitly by indexing into an array of pre-reserved TaskDescriptors with the allocated task id.`  
4. `Pretend the beginning of the stack frame already contains the task context, including registers x0 - x30, ESR_EL1, SPSR_EL1, ELR_EL1.`  
5. `Initialize the link register to the Exit() system call, ELR_EL1 to the function entry, and all other registers to 0.`  
6. `Initialize the TaskDescriptor’s fields, and enqueue into the scheduler queue.`

#### `Context Switching` {#context-switching}

1. `In the boot loader code, the boot loader sets the exception vector to a dedicated vector with synchronized exception pointing to a function named preSyscallEntry.`  
     
2. `When a system call is called, the system call calls SVC #N, where N depends on the type of the system call.`  
     
3. `SVC #N looks up the exception vector, and jumps to preSyscallEntry and switches to the EL1 mode.`  
     
4. `preSyscallEntry temporarily switches to EL0 stack pointer, and pushes the current task’s context into its dedicated stack. It then switches back to EL1 stack pointer, and calls the syscallEntry(taskStackPointer).`  
     
5. `syscallEntry(taskStackPointer) is a C function that checks ESR_EL1 to see the svc code, and performs the corresponding system calls operations such as Create, MyTid, Yield etc. ESR_EL1 and any arguments are read from the user stack. At the end of the syscallEntry, it schedules the next task in a round robin format.`  
     
6. `Context switches to the next scheduled task by temporarily switching to EL0 stack pointer, and pops the context registers from the task’s stack. Set ELR_EL1 to the PC to jump to the address with ERET.`

`![][image2]`

#### `Kernel State` {#kernel-state}

`We adopted the “temporary stack” kernel design, where the kernel acts as an event handler. As such, the function call that context switches to a task never returns. And, in svcEntry, the assembly resets the kernel stack pointer back to its initial value. All kernel states that persist between kernel entries are kept as static variables.`

[image1]: <data:image/png;base64,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>

[image2]: <data:image/png;base64,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>
