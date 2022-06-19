// Timer.h
// Real time interrupt
// COPYRIGHT 2012 Sasha Chrichov

#pragma once

class Dma {
    static const uint32_t tcmasks[];
    static const uint32_t flagmasks[];
    public: static DMA_TypeDef* GetDma(uint8_t ch) { return (ch < 8) ? DMA1 : DMA2; }
    public: static DMA_Stream_TypeDef* GetChannel(uint8_t ch) { return (DMA_Stream_TypeDef*)((ch < 8) ? (uint8_t*)DMA1 + 0x0010 + 0x0018 * ch : (uint8_t*)DMA2 + 0x0010 + 0x0018 * (ch-8)); }
    public: static volatile uint32_t* GetFlagsReg(uint8_t ch) { return (ch < 4) ? &DMA1->LISR : (ch < 8) ? &DMA1->HISR : (ch < 12) ? &DMA2->LISR : (ch < 16) ? &DMA2->HISR : 0; }
    public: static uint32_t GetTcMask(uint8_t ch) { return tcmasks[ch & 3]; }
    public: static uint32_t GetFlagMask(uint8_t ch) { return flagmasks[ch & 3]; }
    public: static void EnableClock(uint8_t ch) { RCC->AHB1ENR |= 4 | ((ch < 8) ? 1 : 2); }
    public: static void SetMux(uint8_t ch, uint8_t inp) { *((uint32_t*)DMAMUX1 + ch) = inp; }
    public: static void Init(DMA_Stream_TypeDef* c, uint32_t CR, uint32_t NDTR, void* PAR, void* MAR) {
        c->NDTR = NDTR; c->PAR = (uint32_t)PAR; c->M0AR = (uint32_t)MAR; c->CR = CR; 
    }
    public: static void ReInit(DMA_Stream_TypeDef* c, uint32_t NDTR, void* MAR) {
         c->CR &= ~0x00000001; c->NDTR = NDTR; c->M0AR = (uint32_t)MAR;
    }
    public: static void Enable(DMA_Stream_TypeDef* c) { c->CR |= 0x00000001; }
    public: static void Disable(DMA_Stream_TypeDef* c) { c->CR &= ~0x00000001; }
};

class Spi {
    static SPI_TypeDef* const spis[];    // All SPIs
    static uint8_t const mux[];                       // SPI RX connection to DMAMUX1
    static volatile uint32_t* const rccregs[];  // Mapping to RCC registers
    static uint32_t const rccbits[];                        // Mapping to RCC bits   
    static uint32_t const clocks[];        // Connection to clock     
    public: static SPI_TypeDef* GetSpi(uint8_t i) { return (i <= 6) ? spis[i] : 0; }
    public: static void EnableClock(uint8_t i) { if (i <= 6) *rccregs[i] |= rccbits[i]; }
    public: static uint8_t GetMuxRxInp(uint8_t i) { return (i <= 5) ? mux[i] : 0; }    
    public: static uint8_t GetMuxTxInp(uint8_t i) { return (i <= 5) ? mux[i]+1 : 0; }    
    public: static uint32_t GetClock(uint8_t i) { return (i <= 6) ? clocks[i] : 0;}
    public: static void Init(SPI_TypeDef* spi, uint32_t CR1, uint32_t CR2, uint32_t CFG1, uint32_t CFG2) {
    	spi->CR1 = CR1 & ~0x00000201;
        spi->CFG1 = CFG1; 
        spi->CFG2 = CFG2; 
        spi->CR2 = CR2; 
        spi->CR1 = CR1; 
    }
    public: static void Stop(SPI_TypeDef* spi) { spi->CR1 &= ~0x00000201; spi->CFG1 & ~0x0000c000; spi->IFCR = 0x00000FF8; }
    public: static void Start(SPI_TypeDef* spi) { uint32_t c = spi->CR1; spi->CR1 = c | 1; spi->CR1 = c | 0x00000201; }
    public: static void EnableDma(SPI_TypeDef* spi) { spi->CFG1 |= 0x0000c000; }
    // low-level operations for firmware downloading
    public: static uint8_t SendReceive(SPI_TypeDef* spi, uint8_t b) { 
        *(uint8_t*)&spi->TXDR = b; 
        Start(spi);
        while ((spi->SR & 0x00001000) == 0) ;   // wait for transfer end
        uint8_t r = *(uint8_t*)&spi->RXDR;
        return r;
    }
    public: static uint8_t SendBlock(SPI_TypeDef* spi, uint8_t* b, int count) {
        uint8_t cs = *b;
        *(uint8_t*)&spi->TXDR = *b++;
        Start(spi);
        for (int i = 1; i < count; i++) {
            while ((spi->SR & 0x00000002) == 0) ;   // wait for TxFIFO space available
            cs ^= *b;
            *(uint8_t*)&spi->TXDR = *b++;
        }
        while ((spi->SR & 0x00000002) == 0) ;   // wait for TxFIFO space available
        *(uint8_t*)&spi->TXDR = cs;
        while ((spi->SR & 0x00001000) == 0) ;   // wait for transfer end
    } 
    public: static uint8_t SendData(SPI_TypeDef* spi, uint8_t* b, int count) {
        *(uint8_t*)&spi->TXDR = count - 1;
        Start(spi);
        uint8_t cs = 0;
        for (int i = 0; i < count; i++) {
            while ((spi->SR & 0x00000002) == 0) ;   // wait for TxFIFO space available
            cs ^= *b;
            *(uint8_t*)&spi->TXDR = *b++;
        }
        while ((spi->SR & 0x00000002) == 0) ;   // wait for TxFIFO space available
        *(uint8_t*)&spi->TXDR = cs;
        while ((spi->SR & 0x00001000) == 0) ;   // wait for transfer end
    } 
};

class ServoStruct;

class PmcuSpi {
    uint32_t index;
    SPI_TypeDef* spi;
    DMA_TypeDef* dma;
    DMA_Stream_TypeDef *rxStream, *txStream; 
    volatile uint32_t *rxFlags, *txFlags;
    uint32_t rxCompleteFlag, rxFlagMask, txCompleteFlag, txFlagMask;
    uint32_t *inBuf[2], *outBuf[2];
    uint32_t iTick, iBuf;
    public: uint8_t Valid;
    public: ServoStruct* servo;
    public: void Init(uint8_t index, uint8_t ispi, uint8_t idma);
    public: uint8_t IsReadComplete() { return (*rxFlags & rxCompleteFlag) != 0; }
    public: uint8_t IsWriteComplete() { return (*txFlags & txCompleteFlag) != 0; }
    public: void TickStart();
    public: void TickEnd();
    public: void DecipherReport();
    public: void EncipherCommand(uint32_t* buf);
    public: void StartTransfer() { Dma::Enable(txStream); }
    uint32_t cnt;
    uint32_t faultcount;

    // Support of downloading PMCU firmware
    void DownInit(uint8_t ind, uint8_t ispi);
    uint8_t DownGetAck();
    uint8_t DownSynchro();
    uint8_t DownCommand(uint8_t com);
    uint8_t DownChunk(uint8_t* b, int count, int addr);
    uint8_t Download();
};

void PmcuSpiInit();
void PmcuSpiTickStart();
void PmcuSpiTickEnd();
void PmcuDownload();


