#include "chip.h"

#include "global.h"
#include "command.h"
#include "lwipif.h"
#include "communication.h"

#define UARTINSIZE  1024
#define UARTOUTSIZE 1024
__attribute__((section(".mdmalink"))) static struct MdmaLink uartlist[10], ethlist[10];     // MDMA linked list
__attribute__((section(".ramD1"), aligned(32))) uint8_t uartwritebuf[UARTOUTSIZE], uartreadbuf[UARTINSIZE];
uint8_t uartwriteheader[UARTOUTSIZE], ethwriteheader[ETH_TX_BUFFER_SIZE];

class CommUart : public CommChannelDma {
    USART_TypeDef* hard;
    uint16_t received, readlen, intertime, intercount;
    public: void Init(uint8_t index, uint8_t periphindex, uint8_t bus, uint8_t dma, uint8_t rxstream, uint8_t rxchannel, uint8_t txstream, uint8_t txchannel, uint32_t rate, uint16_t intertimeout);
    public: virtual void Tick();
    private: void TickRead();
    private: void TickWrite();
    private: int StartRead();
    private: int StartWrite();
    private: int ContinueWrite();
};

class CommEth : public CommChannel {
    uint16_t received, readlen;
    public: void Init(uint8_t index, uint8_t rxstream, uint8_t txstream);
    public: virtual void Tick();
    private: int StartWrite();
    private: int ContinueWrite();
    public: void ExecuteCommand(uint8_t* command, int clen);
};

CommUart commUart;
CommEth commEth;

void CommunicationInit() {
    RCC->AHB3ENR |= 0x00000001;     // Enable MDMA clock
    commUart.Init(0, 3, 1, 1, 0, 45, 1, 46, 115200, 2000);
    commEth.Init(1, 2, 3);
}
void CommunicationTick() {
    commEth.Tick();
    commUart.Tick();
}

void CommChannelDma::Init(uint8_t index, uint8_t dma, uint8_t rxstream, uint8_t rxchannel, uint8_t txstream, uint8_t txchannel) {
    Channel = index;
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

void CommUart::Init(uint8_t index, uint8_t periphindex, uint8_t bus, uint8_t dma, uint8_t rxstream, uint8_t rxchannel, uint8_t txstream, uint8_t txchannel, uint32_t rate, uint16_t intertimeout)
{
    CommChannelDma::Init(index, dma, rxstream, rxchannel, txstream, txchannel);
    Trans.Inb = uartreadbuf; 
    Trans.Mbuf.Bufs[0].Addr = Trans.Outb = uartwriteheader; Trans.Outbl = UARTOUTSIZE;
    Trans.Mbuf.Count = 0;
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
    int n = UARTINSIZE - ReadCount();
    if (n == 0) return;
    if (received == 0) {
        InvalidateDCacheIfUsed(Trans.Inb, 1);
        if (Trans.Inb[0] != 0xFD) {StartRead(); return;}      // Garbage
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
        InvalidateDCacheIfUsed(Trans.Inb, 6);
        readlen = *(uint16_t*)&Trans.Inb[4];
        if ((readlen > 128) || (readlen < 8)) {StartRead(); return;}      // Garbage
    }
    if ((n < readlen) || (WriteCount() > 0)) return;
    InvalidateDCacheIfUsed(Inbuf, readlen);
    Trans.Inbl = readlen;
   	CommandExecute(&Trans);
    StartWrite();
    StartRead();
}

void CommUart::TickWrite() {
    if (Trans.Mbuf.Count && !BusyWrite()) {
        ContinueWrite();
    }
}

void CommUart::Tick() {
    TickRead();
    TickWrite();
}

int CommUart::StartRead() {
    hard->CR3 &= (uint16_t)~0x0040; 	// Disable USART Rx DMA request
//    uint32_t dummy = hard->ISR;          // Read SR to clear errors
//    dummy = hard->RDR;                   // Read DR to clear errors
    hard->ICR = 0x0000103F;				// Clear read flags
    StartReadDma((void*)&hard->RDR, Trans.Inb, UARTINSIZE);      // Start DMA
    received = readlen = 0;
    hard->CR3 |= (uint16_t)0x0040;		// Enable USART Rx DMA request
	return 0;
}

int CommUart::StartWrite() {
    Trans.BuCount = Trans.ByCount = 0;
    ContinueWrite();
}

int CommUart::ContinueWrite() {
    int cnt = 0, li = 0;
    uint8_t final;
    while (1) {
        int chunk = Trans.Mbuf.Bufs[Trans.BuCount].Count - Trans.ByCount;
        int cnt1 = cnt + chunk;
        int8_t full = cnt1 >= UARTOUTSIZE, cmpr = cnt1 <= UARTOUTSIZE; 
        final = cmpr && (Trans.BuCount + 1 == Trans.Mbuf.Count);
        if (full) chunk = UARTOUTSIZE - cnt;
        uint32_t CTCR = (!(cnt & 3) && (full || !(cnt1 & 3))) ? 0x700C0AAA : 0x7000000A;        // Software request, TRGM Full Transfer, 32-bit : byte, no burst, source/target increment 
        uint32_t addr = (uint32_t)(Trans.Mbuf.Bufs[Trans.BuCount].Addr + Trans.ByCount);
        if (full || final) {
    		Mdma::InitLink(uartlist[li++], CTCR, chunk, addr, (uint32_t)(uartwritebuf + cnt), 0, 0, ((addr < 0x20000000) || (addr >= 0x24000000)) ? 0 : 0x00010000);
            if (final) {
                Trans.Mbuf.Count = 0;
            } else if (cmpr) {
                Trans.BuCount++; Trans.ByCount = 0;
            } else {
                Trans.ByCount += chunk;
            }
            cnt += chunk;
            break;
        } else {
    		Mdma::InitLink(uartlist[li], CTCR, cnt1, addr, (uint32_t)(uartwritebuf + cnt), 0, (uint32_t)&uartlist[li+1], ((addr < 0x20000000) || (addr >= 0x24000000)) ? 0 : 0x00010000);
            Trans.BuCount++; Trans.ByCount = 0;
            cnt += chunk;
            li++;
        }
    }
    if (li > 0) {
        Mdma::InitHard(TxMdma, 0x00000040, uartlist[0]);
        Mdma::Start(TxMdma);
        hard->CR3 &= (uint16_t)~0x0080;		// Disable USART Tx DMA request
        StartWriteDma((void*)&hard->TDR, uartwritebuf, cnt);
        hard->CR3 |= (uint16_t)0x0080;		// Enable USART Tx DMA requests
        hard->ISR = (uint16_t)~0x0040;		// Clear TC bit in the SR register by writing 0 to it
    }
	return 0;
}

void CommEth::Init(uint8_t index, uint8_t rxstream, uint8_t txstream)
{
    Channel = index;
    Trans.Mbuf.Bufs[0].Addr = Trans.Outb = ethwriteheader; Trans.Outbl = ETH_TX_BUFFER_SIZE;
    Trans.Mbuf.Count = 0;
    RxMdma = (MDMA_Channel_TypeDef*)((uint8_t*)MDMA + 0x0040 + 0x0040 * rxstream);
    TxMdma = (MDMA_Channel_TypeDef*)((uint8_t*)MDMA + 0x0040 + 0x0040 * txstream);
}

void CommEth::Tick() {
    EthTick();
}

int CommEth::StartWrite() {
    Trans.BuCount = Trans.ByCount = 0;
    ContinueWrite();
}

int CommEth::ContinueWrite() {
    uint8_t* txbuf = EthTxAlloc(ETH_TX_BUFFER_SIZE);            
    if (txbuf == 0) return 0;
    int cnt = 0, li = 0;
    uint8_t final;
    while (1) {
        int chunk = Trans.Mbuf.Bufs[Trans.BuCount].Count - Trans.ByCount;
        int cnt1 = cnt + chunk;
        int8_t full = cnt1 >= ETH_TX_BUFFER_SIZE, cmpr = cnt1 <= ETH_TX_BUFFER_SIZE; 
        final = cmpr && (Trans.BuCount + 1 == Trans.Mbuf.Count);
        if (full) chunk = ETH_TX_BUFFER_SIZE - cnt;
        uint32_t CTCR = (!(cnt & 3) && (full || !(cnt1 & 3))) ? 0x700C0AAA : 0x7000000A;        // Software request, TRGM Full Transfer, 32-bit : byte, no burst, source/target increment 
        uint32_t addr = (uint32_t)(Trans.Mbuf.Bufs[Trans.BuCount].Addr + Trans.ByCount);
        if (full || final) {
    		Mdma::InitLink(ethlist[li++], CTCR, chunk, addr, (uint32_t)cnt, 0, 0, ((addr < 0x20000000) || (addr >= 0x24000000)) ? 0 : 0x00010000);
            if (final) {
                Trans.Mbuf.Count = 0;
            } else if (cmpr) {
                Trans.BuCount++; Trans.ByCount = 0;
            } else {
                Trans.ByCount += chunk;
            }
            cnt += chunk;
            break;
        } else {
    		Mdma::InitLink(ethlist[li], CTCR, cnt1, addr, (uint32_t)cnt, 0, (uint32_t)&ethlist[li+1], ((addr < 0x20000000) || (addr >= 0x24000000)) ? 0 : 0x00010000);
            Trans.BuCount++; Trans.ByCount = 0;
            cnt += chunk;
            li++;
        }
    }
    if (li > 0) {
        for (int i = 0; i < li; i++) ethlist[i].CDAR += (uint32_t)txbuf;
        Mdma::InitHard(TxMdma, 0x00000040, ethlist[0]);
        Mdma::Start(TxMdma);
        EthSend();
        if (final) EthTxEnd();
    }
	return 0;
}

void CommEth::ExecuteCommand(uint8_t* command, int clen) {
    Trans.Inb = command; Trans.Inbl = clen; 
    CommandExecute(&Trans); 
    StartWrite();   
}

void EthCallback(uint8_t* command, int clen) {
    commEth.ExecuteCommand(command, clen);    
}
