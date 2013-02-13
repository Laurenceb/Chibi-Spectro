#include <stdint.h>

extern uint32_t _siccmram,_sccmram,_eccmram;

void __late_init(void) {
	uint32_t *addr = &_sccmram, *addr_flash = &_siccmram;
	while( addr < &_eccmram)
		*addr++ = *addr_flash++;
}
