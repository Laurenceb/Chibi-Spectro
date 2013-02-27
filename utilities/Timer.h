#include "stm32f4xx.h"
#include "Hardware_Conf.h"

#define PWM_PERIOD_CENTER  7200
#define PWM_PERIOD_TWO	   7100
#define PWM_PERIOD_THREE   7236
#define PWM_PERIOD_FOUR	   7053
#define PWM_PERIOD_FIVE	   7323

#define PWM_CLK_TRIM_ONE   29
#define PWM_CLK_TRIM_TWO   36
#define PWM_CLK_TRIM_THREE  6
#define PWM_CLK_TRIM_FOUR  23

#define PWM_INIT_ONE	   0
#define PWM_INIT_TWO	   0
#define PWM_INIT_THREE	   304
#define PWM_INIT_FOUR	   4562
#define PWM_INIT_FIVE	   5349


/* The timer setup uses two gated chains
 * Expressed in terms of the channels these are:
 * 1->2->3->4->5
 * The frequencies do not stack linearly
 * Order is 4: 2 : 1 : 3 :5
 * Cylces 102:101:100:99:98
 * Timers   3: 4 : 2 : 5 :9
*/

#define DIM_LED		  PWM_PERIOD_CENTER/200

void Setup_PPG_PWM(void);
void Enable_PPG_PWM(uint8_t Mask);
void Disable_PPG_PWM(void);
