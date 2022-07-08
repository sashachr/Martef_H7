// Thread.cpp
// Low level thread implementation
// COPYRIGHT 2015 Sasha Chrichov

#include "chip.h"     // Processor definitions

#include "Thread.h"

#define Background	(__get_IPSR() == 0)

#define ThreadStackSize	200		// should be even
#define StackAreaSize	(ThreadStackSize * (MaxThreads/* - 1*/))
//#define DefaultThread	(MaxThreads - 1)

uint32_t stackArea[StackAreaSize]; // __attribute__((section(".ccmram")));

//struct ThreadDesc {
	//uint32_t stack;
	//uint32_t SP;
	//uint8_t previous, next;
	//uint8_t state;		// 0xFF - free, 0xFE - reserved, 0 - active (default thread) 1 - active (XMS thread)
//};

ThreadDesc threads[MaxThreads];
uint8_t runningThread, nextThread, spareThreads;

uint32_t defaultStack;

//extern "C" void SwitchToInterruptStack(uint32_t* interruptStack, uint32_t* threadStack);
//extern "C" void SwitchToThreadStack(uint32_t* interruptStack, uint32_t* threadStack);
//extern "C" void Stack();			// software interrupt for thread switch
//extern "C" void StackStarter(uint32_t* to);
//extern "C" void SwapStack(uint32_t* fromStack, uint32_t* toStack)

void ThreadsInit() {
	for (int i = 0; i < StackAreaSize; i++) stackArea[i] = 0xCCCCCCCC;
	uint32_t* s = stackArea;
	ThreadDesc* t = &threads[0];
	for (uint8_t i = 0; i < MaxThreads; i++, t++, s += ThreadStackSize) {
		t->stack = (uint32_t)s;
		t->state = 0xFF;
		t->next = i + 1; 
	}
	threads[MaxThreads - 1].next = 0xFF;			// Terminate spare threads list
	spareThreads = 0;
	runningThread = 0xFF; nextThread = 0xFF;
	register uint32_t SP asm("sp"); 
	defaultStack = SP; 
}

uint8_t ReserveThread() {
	if (Background) __disable_irq();
	uint8_t i = spareThreads;
	if (i != 0xFF) {		// spare thread exists
		spareThreads = threads[i].next;
		threads[i].state = 0xFE;
	}
	if (Background) __enable_irq();
	return i;		
}

// The following constant should correspond to the number of registers saved in the ISR
// additionally to interrupt frame. May change with compiler version/options.
#define nf	25

void StartThread(uint8_t i, uint32_t entry, uint32_t argument, uint8_t* count) {
	ThreadDesc* t = &threads[i];
	uint32_t* frame = (uint32_t*)t->stack + ThreadStackSize - (26 + nf);
	// Fill the frame...
	frame[nf - 1] = 0xffffffe9;			// interrupt return
	frame[nf+0] = argument;			// R0
	frame[nf+5] = 0xffffffff;		// LR
	frame[nf+6] = entry;				// PC
	frame[nf+7] = 0x01000000;		// PSR (Thumb state)
	frame[nf+24] = 0x00000000;		// FPSCR
	t->SP = (uint32_t)frame;
	//if (Background) __disable_irq();
	uint8_t p = runningThread;
	if (p != 0xFF) {
		uint8_t n = threads[p].next;
		t->previous = p; t->next = n;
		threads[p].next = i; threads[n].previous = i;
	} else {
		t->previous = i; t->next = i;
		SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;
	}
	nextThread = i;
	t->state = 1; 
	if (count) ++*count;
	//if (Background) __enable_irq();
 }

//uint8_t CreateThread(uint32_t entry, uint32_t argument) {
	//int i = ReserveThread();
	//if (i) StartThread(i, entry, argument);
	//return i;		
//}

void InactivateThread(uint8_t i) {
	ThreadDesc* t = &threads[i];
	uint8_t p = t->previous, n = t->next;
	if (i == n) {
		nextThread = 0xFF;		// switch to default thread
		SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
	} else {
		threads[p].next = n; threads[n].previous = p;
		if (nextThread == i) nextThread = n;
	}
	t->next = spareThreads; t->state = 0xFF;
	spareThreads = i;
}

void KillThread(uint8_t i, uint8_t* count) {
	if (Background) __disable_irq();
	if (count) --*count;
	InactivateThread(i);
	if (Background) {
		if (runningThread == i) {
			if (nextThread != 0xFF) SwitchThread(); 
		}
		__enable_irq();
		__WFI();
	}
}

//void KillCurrentThread() {
	//KillThread(runningThread);
//}
//
void KillXmsThreads() {	
	if (Background) __disable_irq();
//	uint8_t i = runningThread;
	// uint8_t sw = Background && (threads[i].state == 1);
//	do {
//		uint8_t n = threads[i].next;
//		if (threads[i].state == 1) {
//			InactivateThread(i);
//		}
//		if (i == n) break;
//		i = n;
//	} while (1);
	for (uint8_t i = 0; i < MaxThreads; i++) {
		if (threads[i].state == 1) InactivateThread(i);
	}

	// if (sw) SwitchThread();
//	// if ((__get_IPSR() == 0) && (curThread != i)) {
//	// 	SwitchThread();
//	// 	__enable_irq();
//	// 	while (1) ;	
//	// }
	SCB->SCR |=  SCB_SCR_SLEEPONEXIT_Msk;
	if (Background) {
		__enable_irq();
		__WFI();
	}
}

//void SwapToDefaultStack() {
	//if (runningThread == 0xFF) return;
	//SwitchToInterruptStack(&defaultStack, &threads[runningThread].SP);
//}

//void SwitchThreadISR() {
	//uint8_t cur = runningThread;
	//if (cur == nextThread) return;
	//if (cur != 0xFF) {
		//if (nextThread != 0xFF) {
			//runningThread = nextThread; nextThread = threads[nextThread].next;
			//StackSwap(&threads[cur].SP, &threads[nextThread].SP);
		//} else {
			//SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
			//runningThread = 0xFF;
			//StackSwap(&threads[cur].SP, &defaultStack);
		//}
	//} else {
		//SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;
		//runningThread = nextThread; nextThread = threads[nextThread].next;
		//StackSwap(&defaultStack, &threads[nextThread].SP);
		////StackStarter(&threads[runningThread].SP);
	//}
	////SCB_CleanInvalidateDCache();
	////SCB_DisableDCache();
	////runningThread = nextThread; nextThread = threads[nextThread].next;
	////StackSwap(&threads[cur].SP, &threads[nextThread].SP);
	////SCB_EnableDCache();
//}
//void SwitchThreadISR() {
	//uint8_t next = nextThread;
	//if (next == 0xFF) return;
	//runningThread = next; nextThread = threads[next].next;
	//SwitchToThreadStack(&defaultStack, &threads[next].SP);
//}

void SwitchThread() {
	NVIC->STIR = 9;
}

//extern "C" void ThreadSwitch_Handler();			// software interrupt for thread switch

//__attribute__ ((naked)) 
// __irq __task void ThreadSwitch_Handler() {
// 	if (runningThread == nextThread) {
// 		asm volatile ("bx lr");
// 	}
// 	asm volatile ("push {r4-r11,lr}");
// 	asm volatile ("vpush {d8-d15}");
// 	threads[runningThread].SP = __get_SP(); 
// 	runningThread = nextThread; nextThread = threads[runningThread].next; 
// 	__set_SP(threads[runningThread].SP);
// 	asm volatile ("vpop {d8-d15}");
// 	asm volatile ("pop {r4-r11,pc}");
// }

// Call from background only
//__attribute__ ((naked)) void SwitchThread() {
	//__disable_irq();
	//if (runningThread == nextThread) {
		//__enable_irq();
		//asm volatile ("bx lr");
	//}
	//register uint32_t SP asm("sp");
	//if (runningThread != 0xFF) {
		////asm volatile ("sub sp,sp,#36");
		//// Imitate interrupt frame
		//register uint32_t* frame asm("r0");
		//register uint32_t LR asm("lr");
		//register uint32_t R12 asm("r12");
		//frame = (uint32_t*)SP - (nf + (SP & 0x00000007)? 27 : 26);
		////if (SP & 0x00000007) {
			////frame =  - (27 + nf);
		////} else {
			////frame = (uint32_t*)SP - 35;
		////}
		//asm volatile ("stmia r0,{r4-r11}");
		//frame[nf - 1] = 0xffffffe9;	// interrupt return
		//frame[nf+4] = R12;			// R0
		//frame[nf+5] = LR;			// LR
		//frame[nf+6] = LR;			// return address
		//frame[nf+7] = 0x01000000;		// PSR (Thumb state)
		//frame[nf+24] = 0x00000000;		// FPSCR
		//threads[runningThread].SP = (uint32_t)frame; 
	//}
	//runningThread = nextThread; 
	//if (runningThread != 0xFF) {
		//nextThread = threads[runningThread].next;
		//SP = threads[runningThread].SP;
		//__enable_irq();
		//asm volatile ("pop {r4-r11,pc}");
	//}
	//__enable_irq();
	//__WFI();
//}
