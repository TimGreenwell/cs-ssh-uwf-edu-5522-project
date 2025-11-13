CC        ?= gcc
CFLAGS    ?= -std=c11 -O3 -Wall -Wextra -Wpedantic -Wformat=2 -march=native
LDFLAGS   ?= -lm
OMPFLAGS  ?= -fopenmp

# Sources
SRC_COMMON := common.c
OBJ_COMMON := common.o

SRC_SERIAL := find_closest_serial.c
BIN_SERIAL := find_closest_serial

SRC_OMP := find_closest_omp.c
BIN_OMP := find_closest_omp

BINS := $(BIN_SERIAL) $(BIN_OMP)

.PHONY: all clean dirs

all: $(BINS)

dirs:
	@mkdir -p input output

$(OBJ_COMMON): $(SRC_COMMON) common.h | dirs
	$(CC) $(CFLAGS) $(OMPFLAGS) -c $(SRC_COMMON) -o $(OBJ_COMMON)

$(BIN_SERIAL): $(SRC_SERIAL) $(OBJ_COMMON) | dirs
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

$(BIN_OMP): $(SRC_OMP) $(OBJ_COMMON) | dirs
	$(CC) $(CFLAGS) $(OMPFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	@rm -f $(BINS) $(OBJ_COMMON)

