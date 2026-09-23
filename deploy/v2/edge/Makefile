# 主机端测试：只编译并运行协议解析器的单元测试（不依赖串口/硬件）。
# 交叉编译请用 CMakeLists.txt。

CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra

.PHONY: test clean

test: proto_test
	./proto_test

proto_test: proto_test.c proto.c proto.h
	$(CC) $(CFLAGS) -o $@ proto_test.c proto.c -lm

clean:
	rm -f proto_test
