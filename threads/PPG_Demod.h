#include "ch.h"

#include "PPG.h"

extern Mailbox PPG_Demod[PPG_CHANNELS];
extern Mailbox Pressures_Output;
extern Mailbox Targets_Reported;


Thread* Spawn_PPG_Thread(void);
msg_t PPG_Thread(void *arg);
