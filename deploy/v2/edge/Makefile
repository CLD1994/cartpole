# 主机端测试：只编译并运行单元测试（不依赖串口、NPU 与 SDK 硬件）。
# 交叉编译请用 CMakeLists.txt。

CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra

.PHONY: test clean

test: proto_test policy_test
	./proto_test
	./policy_test

proto_test: proto_test.c proto.c proto.h
	$(CC) $(CFLAGS) -o $@ proto_test.c proto.c -lm

policy_test: policy_test.c policy.c policy.h npu/include/cartpole_npu.h
	$(CC) $(CFLAGS) -Inpu/include -o $@ policy_test.c policy.c -lm

clean:
	rm -f proto_test policy_test
