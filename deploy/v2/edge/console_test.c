/* console_parse 的主机单元测试（纯函数，不接触串口与 NPU）。 */
#include <stdio.h>
#include <string.h>

#include "console.h"

static int failures;

#define CHECK(cond)                                                \
    do {                                                           \
        if (!(cond)) {                                             \
            failures++;                                            \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        }                                                          \
    } while (0)

static void expect_ok(const char *line, console_cmd_t cmd)
{
    console_req_t req;
    if (console_parse(line, &req) != 0) {
        failures++;
        printf("FAIL expected valid '%s'\n", line);
        return;
    }
    CHECK(req.cmd == cmd);
}

static void expect_err(const char *line)
{
    console_req_t req;
    int rc = console_parse(line, &req);
    CHECK(rc == -1);
    if (rc == -1) {
        CHECK(req.err[0] != '\0');
    }
}

int main(void)
{
    console_req_t req;

    /* 无参命令与别名 */
    expect_ok("", CONSOLE_CMD_NONE);
    expect_ok("   \t ", CONSOLE_CMD_NONE);
    expect_ok("home", CONSOLE_CMD_HOME);
    expect_ok("  center  ", CONSOLE_CMD_CENTER);
    expect_ok("state", CONSOLE_CMD_STATE);
    expect_ok("query", CONSOLE_CMD_QUERY);
    expect_ok("reset", CONSOLE_CMD_RESET);
    expect_ok("help", CONSOLE_CMD_HELP);
    expect_ok("quit", CONSOLE_CMD_QUIT);
    expect_ok("exit", CONSOLE_CMD_QUIT);

    /* vel：合法值 */
    CHECK(console_parse("vel 200", &req) == 0 && req.cmd == CONSOLE_CMD_VEL &&
          req.vel_mm_s == 200.0);
    CHECK(console_parse("vel -150.5", &req) == 0 && req.vel_mm_s == -150.5);
    CHECK(console_parse("vel 0", &req) == 0 && req.vel_mm_s == 0.0);
    /* vel：非法输入 */
    expect_err("vel");
    expect_err("vel abc");
    expect_err("vel nan");
    expect_err("vel inf");
    expect_err("vel 1e");      /* 尾部残缺 */
    expect_err("vel 200 extra");

    /* config：dump、设值 */
    CHECK(console_parse("config", &req) == 0 && req.cmd == CONSOLE_CMD_CONFIG && req.dump);
    CHECK(console_parse("config VEL_MAX 800", &req) == 0 && !req.dump &&
          strcmp(req.name, "VEL_MAX") == 0 && req.value == 800);
    CHECK(console_parse("config X_LIMIT -1", &req) == 0 && req.value == -1);
    CHECK(console_parse("config A_1 5", &req) == 0);
    CHECK(console_parse("config AAAAAAAAAAAAAAA 5", &req) == 0); /* 15 字符，最长合法 */
    /* config：非法输入 */
    expect_err("config VEL_MAX");
    expect_err("config VEL_MAX 800 extra");
    expect_err("config vel_max 800"); /* 小写 */
    expect_err("config VEL_MAX 0x10"); /* 非十进制 */
    expect_err("config VEL_MAX 1.5");
    expect_err("config AAAAAAAAAAAAAAAA 5"); /* 16 字符，超出 name 容量 */

    /* 未知命令、多余参数、大小写敏感 */
    expect_err("foo");
    expect_err("HOME");
    expect_err("home extra");
    expect_err("quit now");
    expect_err("a b c d e");

    if (failures == 0) {
        printf("console_test: all passed\n");
    }
    return failures != 0;
}
