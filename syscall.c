/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#include <libopencm3/stm32/usart.h>

#include <stdlib.h>
#include <reent.h>
#include <sys/stat.h>
#include <sys/errno.h>
#include <errno.h>

#include <usartu.h>

#undef errno
extern int errno;

char *__env[1] = { 0 };

char **environ = __env;

int _execve(char *name, char **argv, char **env) {
    errno = ENOMEM;
    return -1;
}

int _fork(void) {
    errno = EAGAIN;
    return -1;
}

int _getpid(void) {
    return 1;
}

int _kill(int pid, int sig) {
    errno = EINVAL;
    return -1;
}

void _exit(int i) {
    while (1);
}

int _isatty(int file) {
    return 1;
}

int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _link(char *old, char *new) {
    errno = EMLINK;
    return -1;
}

int _lseek(int file, int ptr, int dir) {
    return 0;
}

int _open(const char *name, int flags, int mode) {
    return -1;
}

#define STDIN   0
#define STDOUT  1
#define STDERR  3


int _read(int file, char *ptr, int len) {
    int i = 0;
    while (i < len) {
        ptr[i++] = 0;
    }
    return i;
}


int _write(int file, char *ptr, int len) {
    int i;

    if ((file == STDOUT) || (file == STDERR)) {
        for (i = 0; i < len; i++) {
            usart_putc(USART1, ptr[i]);
        }
        return len;
    }
    return 0;
}

int _stat(char *file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _close(int file) {
    return -1;
}


int _times(struct tms *buf) {
    return -1;
}

int _unlink(char *name) {
    errno = ENOENT;
    return -1;
}

int _wait(int *status) {
    errno = ECHILD;
    return -1;
}

void *xxx_sbrk(int incr) {

    extern unsigned char *_end;
    static unsigned char *heap = NULL;
    unsigned char *prev_heap;

    if (heap == NULL) {
        heap = (unsigned char*)&_end;
    }
    prev_heap = heap;

    heap += incr;
    return prev_heap;
}

register char* stack_ptr __asm__ ("sp");

caddr_t __attribute__((weak)) _sbrk (int incr) {
    extern char   end __asm__ ("_end");
    static char * heap_end;
    char *        prev_heap_end;

    if (heap_end == NULL) {
        heap_end = &end;
    }

    prev_heap_end = heap_end;

    if (heap_end + incr > stack_ptr) {
#if 0
       extern void abort (void);
        _write (1, "_sbrk: Heap and stack collision\n", 32);
       abort ();
#else
       errno = ENOMEM;
       return (caddr_t) -1;
#endif
     }

    heap_end += incr;
    return (caddr_t) prev_heap_end;
}
