#include <cstdint>
#include <cstddef>
#include "memory.h"

// Set up linkers, BSS sections, and constructors.
extern "C" void setup_mmu(); // in mmu.S
using ConstructorType = void(*)();
extern ConstructorType __init_array_start, __init_array_end; // defined in linker script

extern "C" {
    int kmain() {
    #if defined(MMU)
        setup_mmu();
    #endif
        // C++ constructors.
        for (ConstructorType* ctr = &__init_array_start; ctr < &__init_array_end; ++ctr) {
            (*ctr)();
        }

        return 0;
    }
}
