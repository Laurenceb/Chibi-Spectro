#include "stm32f4xx.h"
#include "Hardware_Conf.h"

#define PWM_PERIOD_CENTER  14400
#define PWM_PERIOD_TWO	   14200
#define PWM_PERIOD_THREE   14530
#define PWM_PERIOD_FOUR	   14082
#define PWM_PERIOD_FIVE	   14650

#define PWM_CLK_TRIM_ONE   58
#define PWM_CLK_TRIM_TWO   36
#define PWM_CLK_TRIM_FOUR  15
#define PWM_CLK_TRIM_SPARE 43

/* The timer setup uses two gated chains
 * Expressed in terms of the channels these are:
 * 1->2->4->3
 * SPARE->5
 * SPARE is TIM10
 * The frequencies do not stack linearly
 * Order is 4: 2 : 1 : 3 :5
 * Cylces 102:101:100:99:98
 * Timers   3: 4 : 2 : 5 :9
*/

void Setup_PPG_PWM(void);
