CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -pedantic

.PHONY: test clean

test:
	@binary=$$(mktemp /tmp/device-session.XXXXXX); \
	trap 'rm -f "$$binary"' EXIT; \
	$(CC) $(CFLAGS) session.c session_test.c -lm -o "$$binary" && "$$binary"

# test 使用临时文件并在退出时清理，不产生仓库构建物。
clean:
	@:

.PHONY: test-receiver
test: test-receiver
test-receiver:
	@binary=$$(mktemp /tmp/device-receiver.XXXXXX); \
	trap 'rm -f "$$binary"' EXIT; \
	$(CC) $(CFLAGS) receiver.c receiver_test.c -lm -o "$$binary" && "$$binary"

.PHONY: test-state test-integration
test: test-state test-integration
test-state:
	@binary=$$(mktemp /tmp/device-state.XXXXXX); \
	trap 'rm -f "$$binary"' EXIT; \
	$(CC) $(CFLAGS) session.c receiver.c state_protocol_test.c -lm -o "$$binary" && "$$binary"

test-integration:
	@binary=$$(mktemp /tmp/device-state-integration.XXXXXX); \
	trap 'rm -f "$$binary"' EXIT; \
	$(CC) $(CFLAGS) session.c receiver.c state_protocol_test.c -lm -o "$$binary" && "$$binary" integration
