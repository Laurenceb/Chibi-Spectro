// quickSort.c
#include <stdio.h>

#include "ch.h"

//Comparison function for libc qsort function
int comp (const void * a, const void * b) {
	if(*(uint16_t*)a<*(uint16_t*)b)
		return -1;
	return (*(uint16_t*)a>*(uint16_t*)b);
}








