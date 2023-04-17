/*
 * @Copyright: © 2021, BeingGod. All rights reserved.
 * @Author: BeingGod
 * @Date: 2021-01-21 21:02:36
 * @LastEditors: BeingGod
 * @LastEditTime: 2021-04-24 13:13:56
 * @Description: 统一跨平台毫秒级计时函数
 */

#ifndef _TIMER_H_
#define _TIMER_H_
#define LINUX

typedef double Systime;

void getSystime(Systime &t);
double getTimeIntervalms(const Systime &now, const Systime &last);

#if defined(LINUX)
    #include <sys/time.h>
    #define ENABLE_CNT_TIME (1)

#elif defined(WINDOWS)
    #include <Windows.h>
    #define ENABLE_CNT_TIME (1)
    
#else
    #error "No supported platform"
    #define ENABLE_CNT_TIME (0)
    
#endif

#endif
