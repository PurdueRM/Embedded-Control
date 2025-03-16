#include <sys/stat.h>
#include <errno.h>
#undef errno
extern int errno;

int _close(int file) {
    return -1;
}

int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file) {
    return 1;
}

int _lseek(int file, int ptr, int dir) {
    return 0;
}

int _read(int file, char *ptr, int len) {
    return 0;
}

void *_sbrk(int incr) {
    extern char _end;        /* Defined by the linker */
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_end;
    }
    prev_heap_end = heap_end;
    heap_end += incr;

    return (void *)prev_heap_end;
}

void _exit(int status) {
    while (1) { }  // Hang if exit is called
}

int _kill(int pid, int sig) {
    errno = EINVAL;
    return -1;  // Error, signal handling not implemented
}

int _getpid(void) {
    return 1;  // Return a dummy process ID
}