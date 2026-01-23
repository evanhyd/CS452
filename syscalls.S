.global Create
.global MyTid
.global MyParentTid
.global Yield
.global Exit

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
