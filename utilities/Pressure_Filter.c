#include "Pressure_Filter.h"

#include "ch.h"

/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
   Command line: /www/usr/fisher/helpers/mkshape -r 4.5000000000e-02 1.0000000000e+00 13 -x -l */

#define NZEROS 12
#define GAIN   1.119332571e+01

static float xv[NZEROS+1];

static const float xcoeffs[] =
  { +0.3351250371, +0.5401012303, +0.7564635992, +0.9605418945,
    +1.1280796826, +1.2381173844, +1.2764680578, +1.2381173844,
    +1.1280796826, +0.9605418945, +0.7564635992, +0.5401012303,
    +0.3351250371,
  };

float filterloop(float input)
  {
	float sum; uint8_t i;
        for (i = 0; i < NZEROS; i++) xv[i] = xv[i+1];
        xv[NZEROS] = input / GAIN;
        sum = 0.0;
        for (i = 0; i <= NZEROS; i++) sum += (xcoeffs[i] * xv[i]);
        return sum;
  }


//float filterloop(float x) {return x;}
/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
   Command line: /www/usr/fisher/helpers/mkfilter -Be -Lp -o 8 -a 5.0000000000e-02 0.0000000000e+00 -l */
/*
#define NZEROS 8
#define NPOLES 8
#define GAIN   7.329895182e+04

static float xv[NZEROS+1], yv[NPOLES+1];

float filterloop(float input)
{
	xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; xv[4] = xv[5]; xv[5] = xv[6]; xv[6] = xv[7]; xv[7] = xv[8]; 
        xv[8] = input / GAIN;
        yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; yv[4] = yv[5]; yv[5] = yv[6]; yv[6] = yv[7]; yv[7] = yv[8]; 
        yv[8] =   (xv[0] + xv[8]) + 8 * (xv[1] + xv[7]) + 28 * (xv[2] + xv[6])
                     + 56 * (xv[3] + xv[5]) + 70 * xv[4]
                     + ( -0.0290335003 * yv[0]) + (  0.3368060458 * yv[1])
                     + ( -1.7373365436 * yv[2]) + (  5.2128930990 * yv[3])
                     + ( -9.9697035249 * yv[4]) + ( 12.4721871960 * yv[5])
                     + ( -9.9931759242 * yv[6]) + (  4.7038706060 * yv[7]);
        return yv[8];
}*/
