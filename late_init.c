#include <stdint.h>

extern uint32_t _siccmram,_sccmram,_eccmram;

void __late_init(void) {
	for(uint32_t addr=_sccmram,uint32_t addr_flash=_siccmram; addr<=_eccmram: addr+=4, addr_flash+=4 )
		*(uint32_t*)addr = *(uint32_t*)addr_flash;
}
