/* proto.c 的主机端单元测试：只测解析与格式化的行为分支，不接触串口与硬件。
 *
 * 编译运行：make test （见 Makefile 的 host-test 目标）
 * 约定：每条断言对应 protocol.md 的一个可证伪契约分支。
 */
#include "proto.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* 行缓冲测试用的记录器 */
static char last_line[160];
static int cb_count;
static int received_types[8];
static void cb(const char *line, size_t len, void *user)
{
    (void)user;
    memcpy(last_line, line, len < sizeof(last_line) ? len : sizeof(last_line) - 1);
    last_line[len < sizeof(last_line) ? len : sizeof(last_line) - 1] = '\0';
    assert(cb_count < 8);
    received_types[cb_count++] = proto_parse_line(last_line, &(proto_msg_t){0});
}

static void test_state_valid(void)
{
    proto_msg_t m;
    assert(proto_parse_line("STATE 3 -265000 -3141592 -1500 -10000", &m) == PROTO_STATE);
    assert(m.x_um == -265000 && m.theta_urad == -3141592 && m.vx_mm_s == -1500 && m.vth_mrad_s == -10000);
    assert(proto_parse_line("STATE 3 0 0 0 0", &m) == PROTO_STATE);
    assert(m.x_um == 0 && m.theta_urad == 0 && m.vx_mm_s == 0 && m.vth_mrad_s == 0);
    /* 正式 STATE 必须使用单空格分隔。 */
    assert(proto_parse_line("STATE  3 123 456 78 9", &m) == PROTO_UNKNOWN);
}

static void test_state_invalid(void)
{
    proto_msg_t m;
    assert(proto_parse_line("STATE 1 2 3", &m) == PROTO_UNKNOWN);        /* 缺字段 */
    assert(proto_parse_line("STATE 3 1 2 3 4 5", &m) == PROTO_UNKNOWN);    /* 多余字段 */
    assert(proto_parse_line("STATE 1 2 x 4", &m) == PROTO_UNKNOWN);      /* 非数字 */
    assert(proto_parse_line("STATE", &m) == PROTO_UNKNOWN);              /* 只有头 */
}

static void test_mode_valid(void)
{
    proto_msg_t m;
    assert(proto_parse_line("MODE READY NONE", &m) == PROTO_MODE);
    assert(m.mode == FW_READY && m.reason == FW_REASON_NONE);
    assert(proto_parse_line("MODE FAULT STEP_LOSS", &m) == PROTO_MODE);
    assert(m.mode == FW_FAULT && m.reason == FW_REASON_STEP_LOSS);
    assert(proto_parse_line("MODE FAULT AS5600", &m) == PROTO_MODE);
    assert(m.reason == FW_REASON_AS5600);
    assert(proto_parse_line("MODE FAULT MT6835", &m) == PROTO_MODE);
    assert(m.reason == FW_REASON_MT6835);
    assert(proto_parse_line("MODE DISABLED NONE", &m) == PROTO_MODE);
    assert(m.mode == FW_DISABLED);
    /* HOMING 模式与 HOME_TIMEOUT 原因已随自动回零退役（docs/archive/legacy-v2/calib-spec.md §5） */
    assert(proto_parse_line("MODE HOMING NONE", &m) == PROTO_UNKNOWN);
    assert(proto_parse_line("MODE FAULT HOME_TIMEOUT", &m) == PROTO_UNKNOWN);
}

static void test_mode_invalid(void)
{
    proto_msg_t m;
    assert(proto_parse_line("MODE READY", &m) == PROTO_UNKNOWN);        /* 缺 reason */
    assert(proto_parse_line("MODE RUNNING NONE", &m) == PROTO_UNKNOWN); /* 未知模式词 */
    assert(proto_parse_line("MODE FAULT GLITCH", &m) == PROTO_UNKNOWN); /* 未知原因词 */
    assert(proto_parse_line("MODE READY NONE EXTRA", &m) == PROTO_UNKNOWN);
}

static void test_config_replies(void)
{
    proto_msg_t m;
    assert(proto_parse_line("OK VEL_MAX 1000", &m) == PROTO_OK);
    assert(strcmp(m.param, "VEL_MAX") == 0 && m.value == 1000);
    assert(proto_parse_line("ERR ACC_MAX", &m) == PROTO_ERR);
    assert(strcmp(m.param, "ACC_MAX") == 0);
    /* 标定回执名单是单 token（docs/archive/legacy-v2/calib-spec.md §2），走同一 OK <NAME> <value> 语法 */
    assert(proto_parse_line("OK MARK_LEFT 123", &m) == PROTO_OK);
    assert(strcmp(m.param, "MARK_LEFT") == 0 && m.value == 123);
    assert(proto_parse_line("OK MARK_RIGHT 530000", &m) == PROTO_OK);
    assert(strcmp(m.param, "MARK_RIGHT") == 0 && m.value == 530000);
    assert(proto_parse_line("ERR ENABLE", &m) == PROTO_ERR);
    assert(strcmp(m.param, "ENABLE") == 0);
    assert(proto_parse_line("OK", &m) == PROTO_UNKNOWN);       /* OK 缺参数名/值 */
    assert(proto_parse_line("ERR", &m) == PROTO_UNKNOWN);      /* ERR 缺参数名 */
}

static void test_garbage(void)
{
    proto_msg_t m;
    assert(proto_parse_line("", &m) == PROTO_UNKNOWN);
    assert(proto_parse_line("   ", &m) == PROTO_UNKNOWN);
    assert(proto_parse_line("garbage", &m) == PROTO_UNKNOWN);
    assert(proto_parse_line("VEL 200", &m) == PROTO_UNKNOWN); /* 方向相反的行不归解析器管 */
}

static void test_format(void)
{
    char buf[64];
    proto_command_t command;
    assert(proto_format_vel(buf, 200.0) > 0);
    assert(proto_parse_command(buf, &command) && command.kind == PROTO_COMMAND_VELOCITY && command.value == 200);
    assert(proto_format_vel(buf, -12.6) > 0);
    assert(proto_parse_command(buf, &command) && command.kind == PROTO_COMMAND_VELOCITY && command.value == -13); /* 四舍五入 */
    assert(proto_format_vel(buf, NAN) == 0);                                   /* NaN 拒发 */
    assert(proto_format_vel(buf, 1.0e12) == 0);                                /* 超界拒发 */
    assert(proto_format_simple(buf, "MARK LEFT") > 0);
    assert(proto_parse_command(buf, &command) && command.kind == PROTO_COMMAND_MARK_LEFT);
    assert(proto_format_simple(buf, "ENABLE") > 0);
    assert(proto_parse_command(buf, &command) && command.kind == PROTO_COMMAND_ENABLE);
    assert(proto_format_config(buf, "X_MARGIN_UM", 10000) > 0);
    assert(proto_parse_command(buf, &command));
    assert(command.kind == PROTO_COMMAND_CONFIGURE);
    assert(strcmp(command.parameter, "X_MARGIN_UM") == 0 && command.value == 10000);
}

static void test_command_parser_contract(void)
{
    /* 独立字面帧，避免只用格式化器生成输入形成同错自证。 */
    proto_command_t command = {.kind = PROTO_COMMAND_RESET, .value = 77};
    assert(proto_parse_command("VEL -900\n", &command));
    assert(command.kind == PROTO_COMMAND_VELOCITY && command.value == -900);
    assert(proto_parse_command("CONFIG ACC_MAX 10000\n", &command));
    assert(command.kind == PROTO_COMMAND_CONFIGURE && command.value == 10000);
    assert(strcmp(command.parameter, "ACC_MAX") == 0);
    assert(proto_parse_command("MARK RIGHT\n", &command));
    assert(command.kind == PROTO_COMMAND_MARK_RIGHT);
    const char *invalid[] = {"", "VEL 1", "VEL +1\n", "VEL 01\n", "VEL -0\n",
        "VEL  1\n", "VEL 1 \n", "VEL 1\r\n", "VEL 1000000001\n",
        "VEL 9999999999999999999999999999999999\n", "VEL 1\nQUERY\n",
        "CONFIG acc 1\n", "CONFIG  1\n", "CONFIG X 1 extra\n", "UNKNOWN\n"};
    for (size_t i = 0; i < sizeof invalid / sizeof *invalid; i++) {
        command = (proto_command_t){.kind = PROTO_COMMAND_RESET, .value = 77};
        assert(!proto_parse_command(invalid[i], &command));
        assert(command.kind == PROTO_COMMAND_RESET && command.value == 77);
    }
    assert(!proto_parse_command(NULL, &command));
    assert(!proto_parse_command("QUERY\n", NULL));
}

static void test_rx_feed(void)
{
    proto_rx_t rx;
    proto_rx_init(&rx);
    cb_count = 0;

    /* 一次喂半行，再补齐：两段拼成一行 */
    proto_rx_feed(&rx, (const uint8_t *)"MODE READ", 9, cb, NULL);
    assert(cb_count == 0);
    proto_rx_feed(&rx, (const uint8_t *)"Y NONE\n", 7, cb, NULL);
    assert(cb_count == 1 && strcmp(last_line, "MODE READY NONE") == 0);

    /* \r\n 与连续多行 */
    const char *two = "STATE 1 2 3 4\r\nSTATE 5 6 7 8\n";
    proto_rx_feed(&rx, (const uint8_t *)two, strlen(two), cb, NULL);
    assert(cb_count == 3 && strcmp(last_line, "STATE 5 6 7 8") == 0);

    /* 超长行：截断为一条垃圾行（回调仍触发，解析器按非法行丢弃），后续行恢复正常 */
    uint8_t longline[300];
    memset(longline, 'x', sizeof(longline));
    longline[299] = '\n';
    proto_rx_feed(&rx, longline, sizeof(longline), cb, NULL);
    proto_rx_feed(&rx, (const uint8_t *)"STATE 3 9 9 9 9\n", 16, cb, NULL);
    assert(cb_count == 5 && strcmp(last_line, "STATE 3 9 9 9 9") == 0);
    assert(received_types[3] == PROTO_UNKNOWN); /* 检查实际溢出回调 */
    assert(received_types[4] == PROTO_STATE); /* 恢复帧能被真实解析器接收 */
}

int main(void)
{
    test_state_valid();
    test_state_invalid();
    test_mode_valid();
    test_mode_invalid();
    test_config_replies();
    test_garbage();
    test_format();
    test_command_parser_contract();
    test_rx_feed();
    printf("proto_test: all passed\n");
    return 0;
}
