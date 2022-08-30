/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 *
 */

#include <libopencm3/cm3/scb.h>

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include "scheduler.h"


static scheduler_t* m_scheduler;

#define TASK_COUNT_REG  8

typedef struct __attribute__((packed)) {
    struct {
        uint32_t registers[TASK_COUNT_REG];
    } sw_frame;
    struct {
        uint32_t r0;
        uint32_t r1;
        uint32_t r2;
        uint32_t r3;
        uint32_t r12;
        uint32_t lr;
        uint32_t pc;
        uint32_t psr;
    } nvic_frame;
} stack_frame_t;

static void task_exit_func(void) {
    while (true);
}

#define RETURN_PSR 0x21000000
#define STACK_FILL 0xA5

void scheduler_init(scheduler_t *scheduler) {

    for (int i = 0; i < SCHEDULER_NUM_TASKS; i++) {
        task_t *task = &(scheduler->tasks[i]);
        task->stack = NULL;
        task->stack_size = 0;
        task->sp = (uint32_t)NULL;
        task->runnable = false;
    }
    scheduler->current_task = 0;
    m_scheduler = scheduler;
}

void scheduler_task(scheduler_t *scheduler, int task_no, void (*entry)(void)) {

    task_t *task = &(scheduler->tasks[task_no]);
    task->stack = (void*)malloc(TASK_STACK_SIZE);
    task->stack_size = TASK_STACK_SIZE;
    memset(task->stack, STACK_FILL, task->stack_size);

    task->sp = (uint32_t)(task->stack + task->stack_size);
    task->sp -= sizeof(stack_frame_t);

    stack_frame_t *frame = (stack_frame_t*)task->sp;
    frame->nvic_frame.lr = (uint32_t)task_exit_func;
    frame->nvic_frame.pc = (uint32_t)entry;
    frame->nvic_frame.psr = RETURN_PSR;

    task->runnable = true;
}

static void scheduler_switch(scheduler_t *scheduler) {
    do {
        scheduler->current_task = (scheduler->current_task + 1) % SCHEDULER_NUM_TASKS;
    } while (!scheduler->tasks[scheduler->current_task].runnable);
}

#define EXC_RETURN_MODE_THREAD  0x00000004
#define RETURN_ON_PSP_THREAD    0xFFFFFFFD

void __attribute__((naked)) pend_sv_handler(void) {

    const uint32_t RETURN_ON_PSP = RETURN_ON_PSP_THREAD;

    __asm__("cpsid if":::"memory");
    uint32_t lr;
    __asm__("mov %0, lr\n":"=r"(lr));

    if (lr & EXC_RETURN_MODE_THREAD) {
        uint32_t psp;
        __asm__(
            "mrs %0, psp\n"
            "stmdb %0!, {r4-r11}\n"
            "msr psp, %0\n" : "=r"(psp)
        );
        m_scheduler->tasks[m_scheduler->current_task].sp = psp;
    } else {
        __asm__(
           "stmdb sp!, {r4-r11}\n");
    }

    scheduler_switch(m_scheduler);

    uint32_t psp = (uint32_t)m_scheduler->tasks[m_scheduler->current_task].sp;
    __asm__(
            "ldmfd %0!, {r4-r11}\n"
            "msr psp, %0\n"::"r"(psp)
    );

    __asm__("cpsie if" ::: "memory");
    __asm__("bx %0\n"::"r"(RETURN_ON_PSP));
}

void scheduler_yield(void) {
    SCB_ICSR |= SCB_ICSR_PENDSVSET;
    __asm__(
        "nop\n"
        "nop\n"
        "nop\n"
        "nop\n"
    );
}
