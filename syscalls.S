.global Create
.global MyTid
.global MyParentTid
.global Yield
.global Exit
.global Send
.global Receive
.global Reply
.global AwaitEvent

Create:
    svc #1
    ret

MyTid:
    svc #2
    ret

MyParentTid:
    svc #3
    ret

Yield:
    svc #4
    ret

Exit:
    svc #5
    ret

Send:
    svc #6
    ret

Receive:
    svc #7
    ret

Reply:
    svc #8
    ret

AwaitEvent:
    svc #9
    ret
