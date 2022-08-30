.thumb
.syntax unified

.text

.globl  sem_post32
.type   sem_post32, %function

sem_post32:
1:
        ldrex           r2, [r0]
        add             r2, r2, r1
        strex           r3, r2, [r0]
        teq             r3, #0
        bne             1b
        mov             r0, r2
        bx              lr


.globl  sem_wait32
.type   sem_wait32, %function

sem_wait32:
1:
        ldrex           r2, [r0]
        sub             r2, r2, r1
        strex           r3, r2, [r0]
        teq             r3, #0
        bne             1b
        mov             r0, r2
        bx              lr
