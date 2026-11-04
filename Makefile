
CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -Wpedantic -g -Iinclude -Itest/include
CFLAGS += -D_DEFAULT_SOURCE
CFLAGS += -Iplugins/tinycsocket/include
LDLIBS = -lm

BUILD_DIR = build

CODE_SRC = src/*.c
CODE_SRC += src/math/*.c
CODE_SRC += src/satellite/*.c
CODE_SRC += src/simulation/*.c
CODE_SRC += src/receiver/*.c
CODE_SRC += src/receiver/solver/*.c
CODE_SRC += src/receiver/estimator/*.c
CODE_SRC += src/receiver/estimator/kalman_filter/*.c

CODE_SRC += src/network/*.c
CODE_SRC += plugins/tinycsocket/src/*.c

TARGET = $(BUILD_DIR)/gnss_poc
SRC = $(CODE_SRC) src/main.c 

TEST_TARGET = $(BUILD_DIR)/gnss_poc_test
TEST_SRC = $(CODE_SRC) test/src/*.c

all: $(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET): $(SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

$(TEST_TARGET): $(TEST_SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

build_test: $(TEST_TARGET)

test: $(TEST_TARGET)
	./$(TEST_TARGET) $(filter-out test,$(MAKECMDGOALS))

clean:
	rm -f $(TARGET) $(TEST_TARGET)

.PHONY: all run clean build_test test
