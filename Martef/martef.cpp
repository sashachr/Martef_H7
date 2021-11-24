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
#include "pins.h"
#include "systick.h"
#include "command.h"
#include "timer.h"
#include "flash.h"
#include "martef.h"

void MartefInit() {
    TimeInit();
    CommunicationInit();
    CommandInit();
    Io.Init();
    MotionInit();
    ServoInit();
    Adc.Init();
    Scope.Init();
    PmcuSpiInit();
	ThreadsInit();
    FlashGetInitArea();
	SysTickInit();
	PinsInit();
}

void MartefTick() {
//    GPIOF->BSRR = 0x00000040;           // Set SPI5_NSS
//    GPIOF->BSRR = 0x40008000;           // Set TP1, reset TP2
    TimeTickStart();
    if (!Time.initialDelayCount) {
        PmcuSpiTickStart();
        MotionTick();
        ServoTick();
//        Dac.Tick();
//        LedStatus.Tick();
        for (int i = 0; i < 2; i++) Signals[i].Tick();
        Scope.Tick();
        CommunicationTick();
        PmcuSpiTickEnd();
    }
    TimeTickEnd();
    GPIOF->BSRR = 0x80000000;           // Reset TP1
}
