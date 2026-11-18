/* 串口实现：POSIX termios。仅依赖标准库。 */

#include "uart.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static speed_t baud_const(int baud)
{
    switch (baud) {
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 921600:
        return B921600;
    default:
        return 0;
    }
}

int uart_open(uart_t *u, const char *device, int baud)
{
    u->fd = -1;
    speed_t speed = baud_const(baud);
    if (!speed) {
        errno = EINVAL;
        return -1;
    }
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return -1;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) {
        close(fd);
        return -1;
    }
    cfmakeraw(&tio); /* 原始模式：无行编辑、无信号、无软件流控 */
    tio.c_cflag |= CLOCAL | CREAD;
#ifdef CRTSCTS
    tio.c_cflag &= ~(unsigned)CRTSCTS; /* 硬件流控关闭 */
#endif
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0) {
        close(fd);
        return -1;
    }
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    u->fd = fd;
    return 0;
}

int uart_read(uart_t *u, uint8_t *data, size_t max)
{
    struct pollfd p = {.fd = u->fd, .events = POLLIN};
    if (poll(&p, 1, 0) < 0 && errno != EINTR)
        return -1;
    if (p.revents & (POLLHUP | POLLERR | POLLNVAL)) {
        errno = EIO;
        return -1;
    }
    ssize_t n = read(u->fd, data, max);
    if (n >= 0) {
        return (int)n;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        return 0;
    }
    return -1;
}

int uart_write_all(uart_t *u, const uint8_t *data, size_t n)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    const double end = t.tv_sec + t.tv_nsec * 1e-9 + .020;
    size_t off = 0;
    while (off < n) {
        clock_gettime(CLOCK_MONOTONIC, &t);
        double remaining = end - (t.tv_sec + t.tv_nsec * 1e-9);
        if (remaining <= 0) {
            errno = ETIMEDOUT;
            return -1;
        }
        ssize_t w = write(u->fd, data + off, n - off);
        if (w > 0) {
            off += (size_t)w;
        } else if (w < 0 && errno == EINTR) {
            continue;
        } else if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            struct pollfd p = {.fd = u->fd, .events = POLLOUT};
            int ms = (int)(remaining * 1000.);
            int r = poll(&p, 1, ms > 0 ? ms : 1);
            if (r < 0 && errno != EINTR)
                return -1;
            if (r > 0 && (p.revents & (POLLHUP | POLLERR | POLLNVAL))) {
                errno = EIO;
                return -1;
            }
        } else {
            return -1;
        }
    }
    return 0;
}

int uart_try_write_all(uart_t *u, const uint8_t *data, size_t n)
{
    ssize_t written = write(u->fd, data, n);
    if (written == (ssize_t)n)
        return 0;
    if (written > 0) {
        errno = EIO;
        return -2;
    }
    if (written == 0 || errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
        return 1;
    return -1;
}

void uart_close(uart_t *u)
{
    if (u->fd >= 0) {
        close(u->fd);
        u->fd = -1;
    }
}
