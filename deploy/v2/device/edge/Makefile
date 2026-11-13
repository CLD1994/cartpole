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
