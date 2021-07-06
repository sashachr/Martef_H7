// Thread.cpp
// Low level thread implementation
// COPYRIGHT 2015 Sasha Chrichov

#pragma once

struct ThreadDesc {
	uint32_t stack;
	uint32_t SP;
	uint8_t previous, next;
	uint8_t state;		// 0xFF - free, 0xFE - reserved, 0 - active (default thread) 1 - active (XMS thread)
};

#define MaxThreads	10

extern struct ThreadDesc threads[MaxThreads];
extern uint8_t runningThread, nextThread, spareThreads;
extern uint32_t defaultStack;

void ThreadsInit();

uint8_t ReserveThread();
void StartThread(uint8_t i, uint32_t entry, uint32_t argument, uint8_t* count);
void KillThread(uint8_t i, uint8_t* count);
void KillXmsThreads();

void SwitchThread();					// called from threads						

// Force saving registers not saved in interrupt frame and switch to default stack
#define SwitchToDefaultStack { \
	asm volatile ("push {r3-r11,lr}");	\
	asm volatile ("vpush {d8-d15}"); \
	if (runningThread != 0xFF) { \
		register uint32_t SP asm("sp"); \
		threads[runningThread].SP = SP; \
		SP = defaultStack;	\
	} \
}

#define SwitchToNextThreadStack { \
	runningThread = nextThread; \
	if (runningThread != 0xFF) { \
		register uint32_t SP asm("sp"); \
		nextThread = threads[runningThread].next; \
		defaultStack = SP; \
		SP = threads[runningThread].SP; \
	} \
	asm volatile ("vpop {d8-d15}"); \
	asm volatile ("pop {r3-r11,pc}");	\
}
