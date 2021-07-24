// IO.C 
// Configuration of IO pins	
// COPYRIGHT 2012 Sasha Chrichov

#include "chip.h"

#include "io.h"

IoStruct Io;

#define PROCESS_INPUT(input, bit) \
    { uint8_t i = (input) != 0, r = (IoRaw & bit) != 0 ; \
    if (i != r) { \
        if (i) IoRaw |= bit; else IoRaw &= ~bit; \
        if (IoInverse & bit) i = !i; \
        if (i) Io |= bit; else Io &= ~bit; }}

void IoStruct::UpdateInputs() {
	uint16_t DR = GPIOD->IDR;
	PROCESS_INPUT(DR & 0x0200, IO_ENABLE);
	PROCESS_INPUT(DR & 0x0020, IO_LINEAR);
	PROCESS_INPUT(DR & 0x0040, IO_UHR);
	PROCESS_INPUT(DR & 0x0080, IO_DC);
	PROCESS_INPUT(DR & 0x0010, IO_SETOFFSET);
	PROCESS_INPUT(DR & 0x0008, IO_MOTORLOST);
	PROCESS_INPUT(DR & 0x0400, IO_EMERGENCY);
}

void IoStruct::UpdateOutputs() {
    uint8_t o = ((Io ^ IoInverse) & IO_FAULT) != 0, r = (IoRaw  & IO_FAULT) != 0;
    if (o != r) {
        if (o) {
            IoRaw |= IO_FAULT;
            GPIOB->BSRR = 0x00000200;      // Reset FAULT output to one
        } else {
            IoRaw &= ~IO_FAULT;
            GPIOB->BSRR = 0x02000000;      // Set FAULT output to zero             
        }
    }
}

void IoStruct::SetInversion(uint32_t inverse) {
    uint32_t changed = (IoInverse ^ inverse);
    IoInverse ^= changed;
    IoRaw ^= (changed & ~IO_FAULT);   // For inputs only
}

