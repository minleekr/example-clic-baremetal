# Copyright 2019 SiFive, Inc #
# SPDX-License-Identifier: Apache-2.0 #

PROGRAM ?= example-clic-baremetal

override CFLAGS += -Xlinker --defsym=__stack_size=0x800
override CFLAGS += -Xlinker --defsym=__heap_size=0x0
override CFLAGS += -fomit-frame-pointer

$(PROGRAM): $(wildcard *.c) $(wildcard *.h) $(wildcard *.S)

clean:
	rm -f $(PROGRAM) $(PROGRAM).hex

