# Makefile

SRC = pp3r.c
CFLAGS = -Wall

# Cross compilers (overridable from environment)
CC_X86_64 = x86_64-w64-mingw32-gcc
CC_I686   = i686-w64-mingw32-gcc
CC_LINUX  = gcc

# Output names
OUT_X86_64 = pp3r_x86_64
OUT_I686   = pp3r
OUT_LINUX  = pp3r_linux

.PHONY: all windows clean

all: windows $(OUT_LINUX)

windows: $(OUT_X86_64) $(OUT_I686)

$(OUT_X86_64): $(SRC)
	$(CC_X86_64) $(CFLAGS) $(SRC) -o $@

$(OUT_I686): $(SRC)
	$(CC_I686) $(CFLAGS) $(SRC) -o $@

$(OUT_LINUX): $(SRC)
	$(CC_LINUX) $(CFLAGS) $(SRC) -o $@

clean:
	rm -f $(OUT_X86_64).exe $(OUT_I686).exe $(OUT_LINUX)
