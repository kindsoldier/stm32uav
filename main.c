/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>


#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "usartu.h"

void delay(uint32_t n) {
    for (volatile int i = 0; i < n * 800; i++)
        __asm__("nop");
}

static void clock_setup(void) {
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);
}

static void usart_setup(void) {

    usart_disable(USART1);
    nvic_disable_irq(NVIC_USART1_IRQ);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);

    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);

    usart_disable_rx_interrupt(USART1);

    usart_enable(USART1);
}

static void systick_setup(void) {
    systick_set_frequency(10000, rcc_ahb_frequency);
    systick_interrupt_enable();
    systick_counter_enable();
}


void scheduler_yield(void);
int g_uptime;

void sys_tick_handler(void) {
    g_uptime++;
    scheduler_yield();
}

#define SCHEDULER_NUM_TASKS 4
#define TASK_STACK_SIZE     2048

typedef struct {
    void (*entry)(void);
    void *stack;
    unsigned stack_size;
    uint32_t sp;
    bool runnable;
} task_t;


#define TASK_COUNT_REG  9

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

typedef struct {
    task_t tasks[SCHEDULER_NUM_TASKS];
    uint8_t current_task;
} scheduler_t;

static scheduler_t g_scheduler = {
    .current_task = 0
};


static void task_exit_func(void) {
    while (true);
}

#define RETURN_PSR 0x21000000
#define STACK_FILL 0xA5

#define SCHEDULER_MAX_TASKS 16

void scheduler_init(scheduler_t *scheduler) {
    for (int i = 0; i < SCHEDULER_NUM_TASKS; i++) {
        task_t *task = &(scheduler->tasks[i]);
        task->stack = NULL;
        task->stack_size = 0;
        task->sp = (uint32_t)NULL;
        task->runnable = false;
    }
    scheduler->current_task = 0;
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
            "stmdb %0!, {r2,r4-r11}\n"
            "msr psp, %0\n" : "=r"(psp)
        );
        g_scheduler.tasks[g_scheduler.current_task].sp = psp;
    } else {
        __asm__(
           "stmdb sp!, {r2,r4-r11}\n");
    }

    scheduler_switch(&g_scheduler);

    uint32_t psp = (uint32_t)g_scheduler.tasks[g_scheduler.current_task].sp;
    __asm__(
            "ldmfd %0!, {r2,r4-r11}\n"
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

void task1(void) {
    while (true) {
        printf("task 1 %d\r\n", g_uptime);
        delay(3000);
    };
}

void task2(void) {
    while (true) {
        printf("task 2 %d\r\n", g_uptime);
        delay(3100);
    };
}

void task3(void) {
    while (true) {
        printf("task 3 %d\r\n", g_uptime);
        delay(3200);
    };
}

void task4(void) {
    while (true) {
        printf("task 4 %d\r\n", g_uptime);
        delay(3300);
    };
}

int main(void) {
    g_uptime        = 0;
    clock_setup();
    usart_setup();
    scheduler_init(&g_scheduler);
    scheduler_task(&g_scheduler, 0, task1);
    scheduler_task(&g_scheduler, 1, task2);
    scheduler_task(&g_scheduler, 2, task3);
    scheduler_task(&g_scheduler, 3, task4);

    systick_setup();

    while (true);
}
