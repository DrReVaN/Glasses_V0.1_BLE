/* No filesystem/UART is used. Keep newlib's optional stdio hooks explicit. */
#include <errno.h>
#include <stddef.h>
#include <sys/stat.h>
#include <stdint.h>
extern char _end[], _heap_limit[];
void *_sbrk(ptrdiff_t increment) {
    static char *next; char *previous;
    if (!next) next = _end;
    previous = next;
    if (increment < 0 || increment > 512 || (uintptr_t)next + (uintptr_t)increment > (uintptr_t)_heap_limit) { errno = ENOMEM; return (void *)-1; }
    next += increment; return previous;
}
int _write(int file, char *data, int len) { (void)file; (void)data; return len; }
int _read(int file, char *data, int len) { (void)file;(void)data;(void)len;errno=ENOSYS;return -1; }
int _close(int file) { (void)file;errno=EBADF;return -1; }
int _fstat(int file, struct stat *s) { (void)file;s->st_mode=S_IFCHR;return 0; }
int _isatty(int file) { (void)file;return 1; }
int _lseek(int file,int offset,int whence) { (void)file;(void)offset;(void)whence;errno=ESPIPE;return -1; }
