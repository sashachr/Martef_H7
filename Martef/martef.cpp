// MARTEF.CPP
// COPYRIGHT 2012 Sasha Chrichov

//#include <stdint.h>
#include "chip.h"

#include "global.h"
#include "thread.h"
#include "adc.h"
#include "communication.h"
#include "pmcuspi.h"
#include "triggerscope.h"
#include "siggen.h"
#include "motion.h"
#include "servo.h"
#include "io.h"
//#include "ethip.h"
#include "pins.h"
#include "systick.h"
#include "command.h"
#include "timer.h"
#include "flash.h"
#include "lwipif.h"
#include "martef.h"

void MartefInit() {
    TimeInit();
    CommunicationInit();
//    IpInit();
    CommandInit();
    Io.Init();
    MotionInit();
    ServoInit();
    Adc.Init();
    Scope.Init();
	ThreadsInit();
    FlashGetInitArea();
	PinsInit();
    PmcuDownload();
    PmcuSpiInit();
	SysTickInit();
    EthInit();
}

void MartefTick() {
    TimeTickStart();
    if (!Time.initialDelayCount) {
        PmcuSpiTickStart();
        ServoTick();
//        Dac.Tick();
//        LedStatus.Tick();
        for (int i = 0; i < 2; i++) Signals[i].Tick();
        Scope.Tick();
//  GPIOF->BSRR = 0x00008000;                  // F15 = 1 
        CommunicationTick();
//  GPIOF->BSRR = 0x80000000;                  // F15 = 0 
//  GPIOF->BSRR = 0x00008000;                  // F15 = 1 
        EthTick();
//  GPIOF->BSRR = 0x80000000;                  // F15 = 0 
        PmcuSpiTickEnd();
    }
    TimeTickEnd();
    // GPIOF->BSRR = 0x80000000;           // Reset TP1
}
