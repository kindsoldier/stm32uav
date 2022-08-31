.thumb
.syntax unified

.text

.globl  atom_inc32
.type   atom_inc32, %function

atom_inc32:
1:      ldrex           r2, [r0]
        add             r2, r2, r1
        strex           r3, r2, [r0]
        teq             r3, #0
        bne             1b
        mov             r0, r2
        bx              lr


.globl  atom_dec32
.type   atom_dec32, %function

atom_dec32:
2:      ldrex           r2, [r0]
        sub             r2, r2, r1
        strex           r3, r2, [r0]
        teq             r3, #0
        bne             2b
        mov             r0, r2
        bx              lr
