# Makefile for kalman-embedded and linear-algebra-embedded tests

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -g

# Include directories (-I tells the compiler where to look for .h files)
INCLUDES = -Iinclude -Ilib/linear-algebra-embedded/include

# Library source files
LA_SRCS = lib/linear-algebra-embedded/src/matrix_math.c \
          lib/linear-algebra-embedded/src/vector_math.c
KALMAN_SRCS = src/kalman_filter.c

ALL_LIB_SRCS = $(LA_SRCS) $(KALMAN_SRCS)

# Output directory for executables
BIN_DIR = bin

# Executable targets
TARGET_KALMAN = $(BIN_DIR)/test_kalman
TARGET_MATRIX = $(BIN_DIR)/matrix_math_tests
TARGET_VECTOR = $(BIN_DIR)/vector_math_test

# Phony targets (they don't represent physical files)
.PHONY: all clean prepare

# Default target: builds everything
all: prepare $(TARGET_KALMAN) $(TARGET_MATRIX) $(TARGET_VECTOR)

# Create the binary directory if it doesn't exist
prepare:
	@mkdir -p $(BIN_DIR)

# Rule to build the Kalman Filter test
# Links the math library (-lm) at the end
$(TARGET_KALMAN): tests/test_kalman.c $(ALL_LIB_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -o $@ -lm

# Rule to build the Matrix Math test
# Only requires linear algebra sources, not kalman_filter.c
$(TARGET_MATRIX): lib/linear-algebra-embedded/tests/matrix_math_tests.c $(LA_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -o $@ -lm

# Rule to build the Vector Math test
$(TARGET_VECTOR): lib/linear-algebra-embedded/tests/vector_math_test.c $(LA_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -o $@ -lm

# Clean up generated files
clean:
	rm -rf $(BIN_DIR)