#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

#include "struct_typedef.h"

typedef struct
{
    // Motor
    uint32_t Motor[8];
    
    // CAN Bus Node
    uint32_t PTZnode;
	uint32_t Enemy;
;
    
} OfflineCounter_t;

typedef struct
{
    // Motor
    uint8_t Motor[8];
	//Can bus noode
    uint8_t PTZnode;
    uint8_t Enemy;
} OfflineMonitor_t;



void TimerTaskLoop1000Hz();
void TimerTaskLoop1000Hz();
void TimerTaskLoop500Hz_1();
void TimerTaskLoop500Hz_2();
void TimerTaskLoop100Hz();

extern void DeviceOfflineMonitorUpdate(OfflineMonitor_t *Monitor);
extern int receive_times,send_times;
#endif