
#define _XOPEN_SOURCE 600
/* 无硬件边界测试：非阻塞socket模拟写端被接收方阻塞。 */
#include "uart.h"
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
int main(void)
{
    int master = posix_openpt(O_RDWR | O_NOCTTY);
    assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
    const char *slave = ptsname(master);
    assert(slave);
    uart_t serial;
    assert(uart_open(&serial, slave, 460800) == 0);
    struct termios tio;
    assert(tcgetattr(serial.fd, &tio) == 0 && cfgetispeed(&tio) == B460800);
    uart_close(&serial);
    assert(uart_open(&serial, slave, 12345) == -1 && errno == EINVAL && serial.fd == -1);
    close(master);
    int fd[2];
    assert(socketpair(AF_UNIX, SOCK_STREAM, 0, fd) == 0);
    assert(fcntl(fd[0], F_SETFL, O_NONBLOCK) == 0);
    int send_buffer = 4096;
    assert(setsockopt(fd[0], SOL_SOCKET, SO_SNDBUF, &send_buffer, sizeof send_buffer) == 0);
    uart_t u = {.fd = fd[0]};
    assert(uart_try_write_all(&u, (const uint8_t *)"VEL 10\n", 7) == 0);
    char reply[16];
    assert(read(fd[1], reply, sizeof reply) == 7);
    char data[65536] = {0};
    assert(uart_try_write_all(&u, (const uint8_t *)data, sizeof data) == -2);
    while (write(fd[0], data, sizeof data) > 0) {
    }
    assert(errno == EAGAIN);
    assert(uart_try_write_all(&u, (const uint8_t *)"VEL 10\n", 7) == 1);
    alarm(2);
    assert(uart_write_all(&u, (const uint8_t *)"VEL 0\n", 6) == -1);
    alarm(0);
    uart_close(&u);
    close(fd[1]);
    puts("edge UART timeout passed");
}
