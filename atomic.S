.thumb
.syntax unified

.text

.globl  atomic_inc32
.type   atomic_inc32, %function

atomic_inc32:
1:      ldrex           r2, [r0]
        add             r2, r2, r1
        strex           r3, r2, [r0]
        teq             r3, #0
        bne             1b
        mov             r0, r2
        bx              lr


.globl  atomic_dec32
.type   atomic_dec32, %function

atomic_dec32:
2:      ldrex           r2, [r0]
        sub             r2, r2, r1
        strex           r3, r2, [r0]
        teq             r3, #0
        bne             2b
        mov             r0, r2
        bx              lr
