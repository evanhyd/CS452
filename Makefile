FILENAME=kitty_kernel
XDIR:=/u/cs452/public/xdev
TRIPLE=aarch64-none-elf
XBINDIR:=$(XDIR)/bin
CXX:=$(XBINDIR)/$(TRIPLE)-g++
OBJCOPY:=$(XBINDIR)/$(TRIPLE)-objcopy
OBJDUMP:=$(XBINDIR)/$(TRIPLE)-objdump

MMU?=on
OPT?=-O3 -flto=auto

# COMPILE OPTIONS
ifeq ($(MMU),on)
MMUFLAGS:=-DMMU
else
MMUFLAGS:=-mstrict-align -mgeneral-regs-only
endif
WARNINGS:=-Wall -Wextra -Wpedantic -Wno-unused-const-variable -Werror=shadow -Wconversion \
        -Wsign-conversion -Wcast-align -Wstrict-aliasing -Wreorder -Wuninitialized -Wdouble-promotion -Wvirtual-move-assign
CXXFLAGS:= -std=c++23 -g -pipe -static -ffreestanding -fno-exceptions -fno-rtti -fno-use-cxa-atexit -march=armv8-a -mcpu=cortex-a72 $(OPT) $(MMUFLAGS) $(WARNINGS) \
		-nostdlib -fno-threadsafe-statics -fno-zero-initialized-in-bss

# -Wl,option tells gcc to pass 'option' to the linker with commas replaced by spaces
# doing this rather than calling the linker directly simplifies the compilation procedure
LDFLAGS :=-Wl,-nmagic -Wl,-Tlinker.ld -Wl,--no-warn-rwx-segments -nostartfiles

# Source files and include dirs
SOURCES := $(wildcard *.S) $(wildcard *.cpp)
# Create .o and .d files for every .cpp and .S (hand-written assembly) file
OBJECTS := $(patsubst %.S, %.o, $(patsubst %.cpp, %.o, $(SOURCES)))
DEPENDS := $(patsubst %.S, %.d, $(patsubst %.cpp, %.d, $(SOURCES)))

.PHONY: all clean binary

all: $(FILENAME).img

clean:
	rm -f $(OBJECTS) $(DEPENDS) $(FILENAME).elf $(FILENAME).img

binary: all
	rm -f $(OBJECTS) $(DEPENDS) $(FILENAME).elf

$(FILENAME).img: $(FILENAME).elf
	$(OBJCOPY) -S -O binary $< $@; sync

$(FILENAME).elf: $(OBJECTS) linker.ld
	$(CXX) $(CXXFLAGS) $(filter-out %.ld, $^) -o $@ $(LDFLAGS)
ifneq ($(MMU),on)
	@$(OBJDUMP) -d $@ | grep -Fq q0 && printf "\n***** WARNING: SIMD DETECTED! *****\n\n" || true
endif

%.o: %.S Makefile
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

%.o: %.cpp Makefile
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

-include $(DEPENDS)
