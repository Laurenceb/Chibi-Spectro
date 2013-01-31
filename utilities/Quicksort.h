#include <stdlib.h>

int comp (const void * a, const void * b);

//wrappers for libc quicksort
#define QuickSort(data,numel) qsort((void*)data,(size_t)numel,(size_t)2,comp)


