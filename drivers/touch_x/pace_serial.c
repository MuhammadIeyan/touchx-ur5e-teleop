/* Experimental diagnostic only: pace Touch X serial protocol writes.
 * Does not change packet contents or bypass driver error checks.
 * Build: gcc -shared -fPIC -O2 -Wall -Wextra pace_serial.c -ldl -pthread -o pace_serial.so
 * Use through run_touch_x.sh; start force-disabled. Bounded force opt-in and
 * validation limits are documented in docs/TOUCH_X_UBUNTU_SETUP.md.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <time.h>
#include <unistd.h>

static ssize_t (*original_write)(int, const void *, size_t);
static ssize_t (*original_read)(int, void *, size_t);
static _Atomic int64_t last_rx_ns;
int64_t touch_last_rx_ns(void) { return atomic_load(&last_rx_ns); }
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static struct timespec last;
static long gap_ns = 100000; /* 100 us; ordinary 1 ms traffic is unaffected. */
__attribute__((constructor)) static void setup(void) {
    original_write = dlsym(RTLD_NEXT, "write");
    original_read = dlsym(RTLD_NEXT, "read");
    const char *setting = getenv("TOUCH_WRITE_GAP_US");
    if (setting) {
        long us = strtol(setting, NULL, 10);
        if (us >= 0 && us <= 500) gap_ns = us * 1000;
    }
}
ssize_t read(int fd, void *buffer, size_t length) {
    ssize_t result = original_read(fd, buffer, length);
    int saved_errno = errno;
    struct stat info;
    if (result > 0 && fstat(fd, &info) == 0 && S_ISCHR(info.st_mode) &&
        major(info.st_rdev) == 166 && minor(info.st_rdev) == 0) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        atomic_store(&last_rx_ns, (int64_t)now.tv_sec * 1000000000 + now.tv_nsec);
    }
    errno = saved_errno;
    return result;
}
ssize_t write(int fd, const void *buffer, size_t length) {
    const unsigned char *bytes = buffer;
    struct stat info;
    int saved_errno = errno;
    if (length >= 4 && bytes[0] == 0xa5 && bytes[1] == 0x5a &&
        fstat(fd, &info) == 0 && S_ISCHR(info.st_mode) &&
        major(info.st_rdev) == 166 && minor(info.st_rdev) == 0) {
        pthread_mutex_lock(&lock);
        struct timespec deadline = last;
        deadline.tv_nsec += gap_ns;
        if (deadline.tv_nsec >= 1000000000) {
            deadline.tv_nsec -= 1000000000; ++deadline.tv_sec;
        }
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (last.tv_sec && gap_ns && (now.tv_sec < deadline.tv_sec ||
            (now.tv_sec == deadline.tv_sec && now.tv_nsec < deadline.tv_nsec))) {
            int rc;
            do { rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL); }
            while (rc == EINTR);
        }
        errno = saved_errno;
        ssize_t result = original_write(fd, buffer, length);
        saved_errno = errno;
        clock_gettime(CLOCK_MONOTONIC, &last);
        pthread_mutex_unlock(&lock);
        errno = saved_errno;
        return result;
    }
    errno = saved_errno;
    return original_write(fd, buffer, length);
}
