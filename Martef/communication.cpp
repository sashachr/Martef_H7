#include "chip.h"

#include "Global.h"
#include "Command.h"
//#include "Io.h"
#include "Communication.h"

__attribute__((section(".ramD2"), aligned(32))) uint8_t uartwritebuf[1024], uartreadbuf[1024];

class CommUart : public CommChannelDma {
    USART_TypeDef* hard;
    uint16_t received, readlen, intertime, intercount;
    public: CommUart(uint8_t index, uint8_t periphindex, uint8_t bus, uint8_t dma, uint8_t rxstream, uint8_t rxchannel, uint8_t txstream, uint8_t txchannel, uint32_t rate, uint16_t intertimeout);
    public: virtual void Tick();
    private: void TickRead();
    private: void TickWrite();
    public: virtual int StartRead();
    public: virtual int StartWrite();
};

class CommI2c : public CommChannel {
    I2C_TypeDef* hard;
    uint8_t count;      // bytes to transfer
    int8_t counter;    // current byte to transfer
    uint8_t state;      // 0 - idle, 1 - read, 2 - write
    int timeout;
    int tocounter;
    public: CommI2c(uint8_t index, uint8_t periphindex, uint8_t bus, uint8_t dma, uint8_t rxstream, uint8_t rxchannel, uint8_t txstream, uint8_t txchannel, uint32_t rate);
    public: virtual void Tick();
    private: void TickRead();
    private: void TickWrite();
    public: virtual int StartRead();
    public: virtual int StartWrite();
};

// For AB-07
#define nChannels 1
CommChannel* CommChannels[nChannels] = {
    new CommUart(0, 3, 1, 1, 0, 45, 1, 46, 115200, 2000),
};

void CommunicationInit() {
    RCC->AHB3ENR |= 0x00000001;     // Enable MDMA clock
}
void CommunicationTick() {
    for (int i=0; i<nChannels; i++) {
        CommChannels[i]->Tick();
    }
}

CommChannelDma::CommChannelDma(uint8_t index, uint8_t dma, uint8_t rxstream, uint8_t rxchannel, uint8_t txstream, uint8_t txchannel) : CommChannel(index) {
	DmaRegs = (dma == 1) ? DMA1 : DMA2;
	RCC->AHB1ENR |= 1 << (dma - 1);	                // Enable DMA1/2 clock
    RxStream = (DMA_Stream_TypeDef*)((uint8_t*)DmaRegs + 0x0010 + 0x0018 * rxstream);
    TxStream = (DMA_Stream_TypeDef*)((uint8_t*)DmaRegs + 0x0010 + 0x0018 * txstream);
    RxMdma = (MDMA_Channel_TypeDef*)((uint8_t*)MDMA + 0x0040 + 0x0040 * ((dma-1) * 8 + rxstream));
    TxMdma = (MDMA_Channel_TypeDef*)((uint8_t*)MDMA + 0x0040 + 0x0040 * ((dma-1) * 8 + txstream));
    FindFlags(dma, rxstream, &RxStatus, &RxReadyMask, &RxClearMask);
    FindFlags(dma, txstream, &TxStatus, &TxReadyMask, &TxClearMask);
    RxStream->CR = 0x00030400;  // highest priority, byte size, memory increment, peripheral to memory
    TxStream->CR = 0x00030440;  // highest priority, byte size, memory increment, memory to peripheral
    DMAMUX1[rxstream].CCR = rxchannel;
    DMAMUX1[txstream].CCR = txchannel;
    RxStream->FCR = TxStream->FCR = 0x00000003;     // Disable FIFO
}

void CommChannelDma::FindFlags(uint8_t dma, uint8_t stream, uint32_t volatile** flags, uint32_t* completemask, uint32_t* clearmask) {
    DMA_TypeDef* dmar = (dma == 1) ? DMA1 : DMA2;
    *flags = (stream < 4) ? &dmar->LISR : &dmar->HISR;
    uint8_t s = stream & 0x03;
    *completemask = (s == 0) ? 0x00000020 : (s == 1) ? 0x00000800 : (s == 2) ? 0x00200000 : 0x08000000;
    *clearmask = (s == 0) ? 0x0000003D : (s == 1) ? 0x00000F40 : (s == 2) ? 0x003D0000 : 0x0F400000;
}

void CommChannelDma::StartReadDma(void* periphreg, void* buf, int len) {
    RxStream->CR &= ~1;			                // Disable stream, enable configuration
    while (RxStream->CR & 1) ;
    *(RxStatus + 2) = RxClearMask;              // Clear errors
    RxStream->PAR = (uint32_t)periphreg;	    // Peripheral address
    RxStream->M0AR = (uint32_t)buf;	            // Memory initial address
    RxStream->NDTR = len;			            // Buffer size
    RxStream->CR |= 1;			                // Enable
}

void CommChannelDma::StartWriteDma(void* periphreg, void* buf, int len) {
    TxStream->CR &= ~1;			        // Disable stream, enable configuration
    while (TxStream->CR & 1) ;
    *(TxStatus+2) = TxClearMask;			    // Clear errors 
    TxStream->PAR = (uint32_t)periphreg;		// Peripheral address
    TxStream->M0AR = (uint32_t)buf;	            // Memory initial address
    TxStream->NDTR = len; 			            // Buffer size
    TxStream->CR |= 1;			// Enable stream/
}

// UART support

USART_TypeDef* const Uarts[] = { 0, USART1, USART2, USART3, UART4, UART5, USART6 };

CommUart::CommUart(uint8_t index, uint8_t periphindex, uint8_t bus, uint8_t dma, uint8_t rxstream, uint8_t rxchannel, uint8_t txstream, uint8_t txchannel, uint32_t rate, uint16_t intertimeout)
	: CommChannelDma(index, dma, rxstream, rxchannel, txstream, txchannel)
{
    Inbuf = uartreadbuf; InbufSize = 1024;
    Outbuf = uartwritebuf; OutbufSize = 1024;
    intertime = intertimeout;
    hard = Uarts[periphindex];
    if (bus == 1) {
        RCC->APB1LENR |= 0x00020000 << (periphindex - 2);       // Enable UART clock
    } else {
        RCC->APB2ENR |= (periphindex == 1)? 0x00000010 : 0x00000020;       // Enable USART1/6 clock
    }
    hard->CR1 = 0x0000000C; // Disable, oversampling 16, 8 bits, no parity, TX/RX enabled
    hard->CR2 = 0;          // One stop bit
    hard->CR3 = 0;          // No hardware flow control
    //  Baudrate
    uint32_t integerdivider = (((uint64_t)((bus==1) ? APB1_RATE : APB2_RATE) << 1) / rate + 1) >> 1;    
    hard->BRR = integerdivider;
    hard->CR1 |= 0x00000001;    // Enable
    StartRead();
}

void CommUart::TickRead() {
    int n = InbufSize - ReadCount();
    if (n == 0) return;
    if (received == 0) {
        SCB_InvalidateDCache_by_Addr(Inbuf, 1);
        if (Inbuf[0] != 0xFD) {StartRead(); return;}      // Garbage
    }
    if (n != received) {
        received = n;
        intercount = 0;
    } else {
        if (++intercount > intertime) {
            StartRead(); return;  // Internal timeout
        }    
    }
    if (n < 6) return;
    if (readlen == 0) {
        SCB_InvalidateDCache_by_Addr(Inbuf, 6);
        readlen = *(uint16_t*)&Inbuf[4];
        if ((readlen > 128) || (readlen < 8)) {StartRead(); return;}      // Garbage
    }
    if ((n < readlen) || (WriteCount() > 0)) return;
    SCB_InvalidateDCache_by_Addr(Inbuf, readlen);
   	CommandExecute((uint8_t*)Inbuf, readlen, this);
    StartRead();
}

void CommUart::TickWrite() {
}

void CommUart::Tick() {
    TickRead();
    TickWrite();
}

int CommUart::StartRead() {
    hard->CR3 &= (uint16_t)~0x0040; 	// Disable USART Rx DMA request
    uint32_t dummy = hard->ISR;          // Read SR to clear errors
    dummy = hard->RDR;                   // Read DR to clear errors     
    StartReadDma((void*)&hard->RDR, Inbuf, InbufSize);      // Start DMA
    received = readlen = 0;
    hard->CR3 |= (uint16_t)0x0040;		// Enable USART Rx DMA request
	return 0;
}

int CommUart::StartWrite() {
    hard->CR3 &= (uint16_t)~0x0080;		// Disable USART Tx DMA request
    StartWriteDma((void*)&hard->TDR, Outbuf, *(uint16_t*)(Outbuf + 4));
    hard->CR3 |= (uint16_t)0x0080;		// Enable USART Tx DMA requests
    hard->ISR = (uint16_t)~0x0040;		// Clear TC bit in the SR register by writing 0 to it
	return 0;
}
