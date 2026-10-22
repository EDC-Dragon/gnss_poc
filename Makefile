
CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -Wpedantic -g -Iinclude -Itest/include
LDLIBS = -lm

BUILD_DIR = build

CODE_SRC = src/physical_types.c

TARGET = $(BUILD_DIR)/gnss_poc
SRC = $(CODE_SRC) src/main.c 

TEST_TARGET = $(BUILD_DIR)/gnss_poc_test
TEST_SRC = $(CODE_SRC) test/src/test.c test/src/test_physical_types.c

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
