SDEMU   := ../../components/sdemu
CC      ?= cc
CFLAGS  ?= -O2 -g -std=c11 -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion \
           -fsanitize=address,undefined
CFLAGS  += -I$(SDEMU)/include -I$(SDEMU)/src

SRCS := test_main.c sim_host.c \
        $(SDEMU)/src/sdemu_engine.c $(SDEMU)/src/sdemu_regs.c $(SDEMU)/src/sdemu_crc.c \
        $(SDEMU)/src/sdemu_backend.c $(SDEMU)/src/sdemu_fault.c

sim_test: $(SRCS) $(wildcard $(SDEMU)/include/*.h $(SDEMU)/src/*.h) sim_host.h
	$(CC) $(CFLAGS) -o $@ $(SRCS)

test: sim_test
	./sim_test

clean:
	rm -f sim_test

.PHONY: test clean
