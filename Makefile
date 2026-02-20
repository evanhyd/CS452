FILENAME=kitty_kernel
XDIR:=/u/cs452/public/xdev
TRIPLE=aarch64-none-elf
XBINDIR:=$(XDIR)/bin
CXX:=$(XBINDIR)/$(TRIPLE)-g++
OBJCOPY:=$(XBINDIR)/$(TRIPLE)-objcopy
OBJDUMP:=$(XBINDIR)/$(TRIPLE)-objdump

SRCDIR:=src
OBJDIR:=build

MMU?=on
OPT?=-O3

CACHE?=b

# COMPILE OPTIONS
ifeq ($(MMU),on)
MMUFLAGS:=-DMMU
else
MMUFLAGS:=-mstrict-align -mgeneral-regs-only
endif

ifeq ($(CACHE),b)
MMUFLAGS+= -DENABLE_ICACHE -DENABLE_DCACHE
else ifeq ($(CACHE),i)
MMUFLAGS+= -DENABLE_ICACHE
else ifeq ($(CACHE),d)
MMUFLAGS+= -DENABLE_DCACHE
endif

INCDIRS:=-I$(SRCDIR)

# Flags
WARNINGS:=-Wall -Wextra -Wpedantic -Wno-unused-const-variable -Werror=shadow -Wconversion \
        -Wsign-conversion -Wcast-align -Wstrict-aliasing -Wreorder -Wuninitialized -Wdouble-promotion -Wvirtual-move-assign
CXXFLAGS:= -std=c++23 -g -pipe -static -ffreestanding -fno-exceptions -fno-rtti -fno-use-cxa-atexit -march=armv8-a -mcpu=cortex-a72 $(OPT) $(MMUFLAGS) $(WARNINGS) \
		-nostdlib -fno-threadsafe-statics -fno-zero-initialized-in-bss $(INCDIRS)

# -Wl,option tells gcc to pass 'option' to the linker with commas replaced by spaces
# doing this rather than calling the linker directly simplifies the compilation procedure
LDFLAGS :=-Wl,-nmagic -Wl,-Tlinker.ld -Wl,--no-warn-rwx-segments -nostartfiles

# Source files and include dirs
SOURCES := $(shell find $(SRCDIR) -name '*.cpp' -o -name '*.S')
# Create .o and .d files for every .cpp and .S (hand-written assembly) file
OBJECTS := $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.o, $(patsubst $(SRCDIR)/%.S, $(OBJDIR)/%.o, $(SOURCES)))
DEPENDS := $(OBJECTS:.o=.d)

.PHONY: all clean binary k2_perf_test

all: $(FILENAME).img

clean:
	rm -rf $(OBJDIR) $(FILENAME).elf $(FILENAME).img

binary: all
	rm -rf $(OBJDIR) $(FILENAME).elf

k2_perf_test: CXXFLAGS += -DK2_PERF_TEST
k2_perf_test: binary

$(FILENAME).img: $(FILENAME).elf
	$(OBJCOPY) -S -O binary $< $@; sync

$(FILENAME).elf: $(OBJECTS) linker.ld
	$(CXX) $(CXXFLAGS) $(filter-out %.ld, $^) -o $@ $(LDFLAGS)
ifneq ($(MMU),on)
	@$(OBJDUMP) -d $@ | grep -Fq q0 && printf "\n***** WARNING: SIMD DETECTED! *****\n\n" || true
endif

$(OBJDIR)/%.o: $(SRCDIR)/%.S Makefile
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp Makefile
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

-include $(DEPENDS)
