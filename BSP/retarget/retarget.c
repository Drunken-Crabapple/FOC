#include <errno.h>
#include <stdint.h>
#include <string.h>
#include "retarget.h"
#include "usbd_cdc_if.h"
#include "sys_public.h"

#if !defined(OS_USE_SEMIHOSTING)

#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2
#define RX_QUEUE_SIZE 128U

static uint8_t rx_queue[RX_QUEUE_SIZE];
static uint16_t rx_head;
static uint16_t rx_tail;

static int rx_is_empty(void) {
    return rx_head == rx_tail;
}

static int rx_is_full(void) {
    return (uint16_t)((rx_head + 1U) % RX_QUEUE_SIZE) == rx_tail;
}

static void rx_enqueue(uint8_t data) {
    if (rx_is_full()) return;
    rx_queue[rx_head] = data;
    rx_head = (uint16_t)((rx_head + 1U) % RX_QUEUE_SIZE);
}

static int rx_dequeue(uint8_t *data) {
    if (rx_is_empty()) return 0;
    *data = rx_queue[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % RX_QUEUE_SIZE);
    return 1;
}

void RetargetInit(void) {
#if USE_TinyPrintf == 0 && STDIO_SUPPORT == 1
    setvbuf(stdout, NULL, _IONBF, 0);
#endif
}

void CDC_Receive_FS_Callback(uint8_t *Buf, uint32_t *Len) {
    uint32_t length = *Len;
    while (length--) {
        rx_enqueue(*(Buf++));
    }
}

void CDC_TransmitCplt_FS_Callback(void) {}

signed short shellRead(char *data, unsigned short len) {
    signed short i = 0;
    for (i = 0; i < (signed short)len; ++i) {
        uint8_t byte;
        if (!rx_dequeue(&byte)) break;
        data[i] = (char)byte;
    }
    delay(1);
    return i;
}

signed short shellWrite(char *data, unsigned short len) {
    return CDC_Transmit_FS((uint8_t *)data, len) == USBD_OK ? 0 : -1;
}

#if USE_TinyPrintf == 1
void _putchar(char character) {
    shellWrite(&character, 1);
}
#endif

#if STDIO_SUPPORT == 1
int _write(int fd, char *ptr, int len) {
    if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
        return shellWrite(ptr, (unsigned short)len) == 0 ? len : EIO;
    }
    errno = EBADF;
    return -1;
}

int _read(int fd, char *ptr, int len) {
    if (fd == STDIN_FILENO) {
        return shellRead(ptr, (unsigned short)len);
    }
    errno = EBADF;
    return -1;
}

int _close(int fd) {
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) return 0;
    errno = EBADF;
    return -1;
}

int _fstat(int fd, struct stat *st) {
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) {
        st->st_mode = S_IFCHR;
        return 0;
    }
    errno = EBADF;
    return -1;
}

int _isatty(int fd) {
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) return 1;
    errno = EBADF;
    return 0;
}

int _lseek(int fd, int ptr, int dir) {
    (void)fd;
    (void)ptr;
    (void)dir;
    errno = EBADF;
    return -1;
}
#endif

#endif
