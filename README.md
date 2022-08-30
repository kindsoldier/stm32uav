
# Minimal STM32 F4 task sheduler

NanoOS: "Make less makes no sense" =)

## Register frame description

```
/*
 * ABI: r0..r3 are caller-saved (scratch registers), R4..R12 are callee-saved.
 * It is appropriate to use R12 for a system call opcode (saved by NVIC). The
 * stack pointer points to the current extent of the stack -- it is decremented
 * before being used as index in a store. The stack grows downwards, to lower
 * addresses. When an interrupt is processed, 8 registers are stored. LR is set
 * to a special value that makes an ordinary function return into a return from
 * interrupt. The LR value indicates which stack is going to be used (process
 * or main) and can be modified before return.
 *
 *                  ____________________
 *           Stack |                    |
 *                 |                    |
 *    higher       |        R4          | <-- SP saved in TCB (64B context)
 *  addresses      |        R5          |   ^
 *      |  ^       |        R6          |   |
 *      |  |       |        R7          |   | 8 registers pushed by handler:
 *      |  |       |        R8          |   | R4..R11
 *      |  |       |        R9          |   | Full task context is now stored
 *      V  |       |        R10         |   |
 *         |       |        R11         |   |
 *     direction   |        R0          | <-- SP when SVC handler gets control
 *     of growth   |        R1          |   ^
 *                 |        R2          |   |
 *                 |        R3          |   | 8 registers are pushed by
 *                 |        R12         |   | the NVIC hardware:
 *                 |        LR (R14)    |   | xPSR, PC, LR, R12, R3..R0
 *                 |        PC (R15)    |   |
 *                 |       xPSR         |   |
 *                 |                    | <-- SP before SVC
 *                 |      (stuff)       |
 *       Stack +   |                    |
 *       StackSize |____________________|
 *
 */

```
