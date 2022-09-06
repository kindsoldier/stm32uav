/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 *
 */

#ifndef SCHEDULER_H_QWERTY
#define SCHEDULER_H_QWERTY

#define SCHEDULER_NUM_TASKS 4
#define TASK_STACK_SIZE     2048

typedef struct {
    void (*entry)(void);
    void *stack;
    unsigned stack_size;
    uint32_t sp;
    bool runnable;
} task_t;

typedef struct {
    task_t tasks[SCHEDULER_NUM_TASKS];
    uint8_t current_task;
} scheduler_t;


void scheduler_init(scheduler_t *scheduler);
void scheduler_task(scheduler_t *scheduler, int task_no, void (*entry)(void));
void scheduler_yield(void);

#endif
