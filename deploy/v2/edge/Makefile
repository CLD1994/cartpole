# 原生无硬件测试。生产目标仅由 CMake + build.sh 构建，可选真实 NPU 后端。
CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra -Werror
CPPFLAGS += -D_DEFAULT_SOURCE -I. -Inpu/include
PYTHON ?= python3
TEST_BUILD := .test-build
UNIT := proto_test policy_test console_test $(TEST_BUILD)/control_test $(TEST_BUILD)/session_test $(TEST_BUILD)/edge_proto_test
.PHONY: test test-unit test-integration clean

test: test-unit

test-unit: $(UNIT)
	@set -e; for test in $(UNIT); do ./$$test; done
	$(PYTHON) tests/unit/test_prepare_manifest.py
	$(PYTHON) tests/unit/test_cpu_export.py

$(TEST_BUILD):
	mkdir -p $@

proto_test: proto_test.c proto.c proto.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ proto_test.c proto.c -lm

policy_test: policy_test.c policy.c policy_npu.c cpu_policy.c policy.h npu/include/cartpole_npu.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -DEDGE_ENABLE_NPU=1 -ffp-contract=off -o $@ policy_test.c policy.c policy_npu.c cpu_policy.c -lm

console_test: console_test.c console.c console.h rig.c rig.h uart.c uart.h proto.c proto.h config.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ console_test.c console.c rig.c uart.c proto.c -lm

$(TEST_BUILD)/control_test: tests/unit/test_control.c control.c control.h | $(TEST_BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ tests/unit/test_control.c control.c -lm

$(TEST_BUILD)/session_test: tests/unit/test_edge_session.c rig.c proto.c rig.h proto.h | $(TEST_BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ tests/unit/test_edge_session.c rig.c proto.c -lm

$(TEST_BUILD)/edge_proto_test: tests/unit/test_edge_proto.c proto.c proto.h | $(TEST_BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ tests/unit/test_edge_proto.c proto.c -lm

$(TEST_BUILD)/uart_test: tests/integration/test_edge_uart.c uart.c uart.h | $(TEST_BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ tests/integration/test_edge_uart.c uart.c -lm

# 只供 PTY 集成测试的可执行文件，名称和输出均标记 TEST_ONLY。
$(TEST_BUILD)/edge_TEST_ONLY: main.c uart.c proto.c rig.c console.c control.c package.c tests/support/policy_stub.c $(wildcard *.h) | $(TEST_BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -DEDGE_TEST_STUB -o $@ main.c uart.c proto.c rig.c console.c control.c package.c tests/support/policy_stub.c -lm

test-integration: $(TEST_BUILD)/uart_test $(TEST_BUILD)/edge_TEST_ONLY
	./$(TEST_BUILD)/uart_test
	$(PYTHON) tests/integration/test_cli.py ./$(TEST_BUILD)/edge_TEST_ONLY
	$(PYTHON) tests/integration/test_cpu_policy.py
	$(PYTHON) tests/integration/test_package.py
	$(PYTHON) tests/integration/test_prepare_package.py

clean:
	rm -f proto_test policy_test console_test
	rm -rf $(TEST_BUILD)
