# 原生无硬件测试。生产目标仅由 CMake + build.sh 构建，可选真实 NPU 后端。
CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra -Werror
CPPFLAGS += -DYYJSON_DISABLE_WRITER=1 -DYYJSON_DISABLE_UTILS=1 -DYYJSON_DISABLE_FILE=1 -DYYJSON_DISABLE_INCR_READER=1 -DYYJSON_READER_DEPTH_LIMIT=16 -D_DEFAULT_SOURCE -I. -Inpu/include -Itests/support
PYTHON ?= python3
TEST_BUILD := .test-build
EDGE_HEADERS := $(wildcard *.h)
SUPPORT_HEADERS := $(wildcard tests/support/*.h)
UNIT := $(TEST_BUILD)/format_test proto_test console_test $(TEST_BUILD)/control_test $(TEST_BUILD)/edge_proto_test $(TEST_BUILD)/device_injection_test $(TEST_BUILD)/transactions_test $(TEST_BUILD)/pole_calibration_test
.PHONY: test test-unit test-integration clean

test: test-unit test-integration test-dependencies

test-unit: $(UNIT)
	@set -e; for test in $(UNIT); do ./$$test; done
	$(PYTHON) tests/unit/test_prepare_manifest.py
	$(PYTHON) tests/unit/test_cpu_export.py
	$(PYTHON) tests/unit/test_cli_report.py

$(TEST_BUILD):
	mkdir -p $@

proto_test: proto_test.c proto.c proto.h $(SUPPORT_HEADERS)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ proto_test.c proto.c -lm

policy_test: policy_test.c policy.c policy_npu.c cpu_policy.c policy.h npu/include/cartpole_npu.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -DEDGE_ENABLE_NPU=1 -ffp-contract=off -o $@ policy_test.c policy.c policy_npu.c cpu_policy.c -lm

# Only console_parse is exercised. Discard unused executor functions rather than
# link real serial/device components into a parser unit test.
console_test: console_test.c console.c console.h $(EDGE_HEADERS)
	$(CC) $(CPPFLAGS) $(CFLAGS) -ffunction-sections -fdata-sections -Wl,--gc-sections -o $@ console_test.c console.c -lm

$(TEST_BUILD)/control_test: tests/unit/test_control.c control.c control.h | $(TEST_BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ tests/unit/test_control.c control.c -lm

$(TEST_BUILD)/session_test: tests/integration/test_edge_session.c tests/support/fake_uart.c cartpole.c cartpole_clock.c cartpole_schedule.c serial_cartpole.c cartpole_format.c cartpole_controller.c cartpole_motion.c control.c proto.c cartpole.h proto.h $(EDGE_HEADERS) $(SUPPORT_HEADERS) | $(TEST_BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ tests/integration/test_edge_session.c tests/support/fake_uart.c cartpole.c cartpole_clock.c cartpole_schedule.c serial_cartpole.c cartpole_format.c cartpole_controller.c cartpole_motion.c control.c proto.c -lm

$(TEST_BUILD)/edge_proto_test: tests/unit/test_edge_proto.c proto.c proto.h | $(TEST_BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ tests/unit/test_edge_proto.c proto.c -lm

$(TEST_BUILD)/uart_test: tests/integration/test_edge_uart.c uart.c uart.h | $(TEST_BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ tests/integration/test_edge_uart.c uart.c -lm

# 只供 PTY 集成测试的可执行文件，名称和输出均标记 TEST_ONLY。
$(TEST_BUILD)/edge_TEST_ONLY: main.c uart.c proto.c cartpole.c cartpole_format.c cartpole_clock.c cartpole_schedule.c serial_cartpole.c cartpole_controller.c cartpole_motion.c cartpole_pole_calibration.c console.c control.c package.c third_party/yyjson/yyjson.c tests/support/policy_stub.c $(wildcard *.h) | $(TEST_BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -DEDGE_TEST_STUB -o $@ main.c uart.c proto.c cartpole.c cartpole_format.c cartpole_clock.c cartpole_schedule.c serial_cartpole.c cartpole_controller.c cartpole_motion.c cartpole_pole_calibration.c console.c control.c package.c third_party/yyjson/yyjson.c tests/support/policy_stub.c -lm

test-integration: policy_test $(TEST_BUILD)/uart_test $(TEST_BUILD)/edge_TEST_ONLY $(TEST_BUILD)/session_test $(TEST_BUILD)/serial_cartpole_test
	./policy_test
	./$(TEST_BUILD)/session_test
	./$(TEST_BUILD)/serial_cartpole_test
	./$(TEST_BUILD)/uart_test
	$(PYTHON) tests/integration/test_cli.py ./$(TEST_BUILD)/edge_TEST_ONLY
	$(PYTHON) tests/integration/test_cpu_policy.py
	$(PYTHON) tests/integration/test_package.py
	$(PYTHON) tests/integration/test_prepare_package.py

clean:
	rm -f proto_test policy_test console_test
	rm -rf $(TEST_BUILD)

$(TEST_BUILD)/device_injection_test: tests/unit/test_device_injection.c tests/support/fake_cartpole_device.c cartpole.c cartpole_clock.c cartpole_schedule.c cartpole_controller.c cartpole_motion.c control.c $(EDGE_HEADERS) $(SUPPORT_HEADERS) | $(TEST_BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $(filter %.c,$^) -lm

$(TEST_BUILD)/serial_cartpole_test: tests/integration/test_serial_cartpole.c cartpole_format.c tests/support/fake_uart.c serial_cartpole.c cartpole_format.c cartpole_clock.c proto.c $(EDGE_HEADERS) $(SUPPORT_HEADERS) | $(TEST_BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $(filter %.c,$^) -lm

$(TEST_BUILD)/format_test: tests/unit/test_cartpole_format.c cartpole_format.c $(EDGE_HEADERS) $(SUPPORT_HEADERS) | $(TEST_BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $(filter %.c,$^) -lm

$(TEST_BUILD)/transactions_test: tests/unit/test_cartpole_transactions.c tests/support/fake_cartpole_device.c cartpole.c cartpole_clock.c $(EDGE_HEADERS) $(SUPPORT_HEADERS) | $(TEST_BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $(filter %.c,$^) -lm

$(TEST_BUILD)/pole_calibration_test: tests/unit/test_pole_calibration.c tests/support/fake_cartpole_device.c cartpole.c cartpole_clock.c cartpole_pole_calibration.c $(EDGE_HEADERS) $(SUPPORT_HEADERS) | $(TEST_BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $(filter %.c,$^) -lm

.PHONY: test-dependencies
test-dependencies:
	CC="$(CC)" $(PYTHON) tests/integration/test_dependencies.py
