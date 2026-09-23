/* 串口实现：POSIX termios。仅依赖标准库。 */

#include "uart.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
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
        return B460800;
    }
}

int uart_open(uart_t *u, const char *device, int baud)
{
    u->fd = -1;
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return -1;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) {
        close(fd);
        return -1;
    }
    cfmakeraw(&tio);                 /* 原始模式：无行编辑、无信号、无软件流控 */
    tio.c_cflag |= CLOCAL | CREAD;
#ifdef CRTSCTS
    tio.c_cflag &= ~(unsigned)CRTSCTS; /* 硬件流控关闭 */
#endif
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (cfsetispeed(&tio, baud_const(baud)) != 0 || cfsetospeed(&tio, baud_const(baud)) != 0) {
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
    size_t off = 0;
    while (off < n) {
        ssize_t w = write(u->fd, data + off, n - off);
        if (w > 0) {
            off += (size_t)w;
        } else if (w < 0 && (errno == EAGAIN || errno == EINTR)) {
            continue;
        } else {
            return -1;
        }
    }
    return 0;
}

void uart_close(uart_t *u)
{
    if (u->fd >= 0) {
        close(u->fd);
        u->fd = -1;
    }
}
