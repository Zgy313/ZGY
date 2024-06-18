#include <seekfree_assistant_interface.h>
#include <zf_common_clock.h>
#include <zf_device_ips200.h>
#include <zf_device_wifi_spi.h>
#include "Cpu0Init.h"


//CPU0用户函数的初始化，大多是中断的初始化
void Cpu0_Init(void)
{





    cpu_wait_event_ready();



}
