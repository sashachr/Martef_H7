#include "chip.h"

#include "global.h"
#include "martef.h"
#include "time.h"
#include "pmcuspi.h"

const uint32_t Dma::tcmasks[] = { 0x00000020, 0x00000800, 0x00200000, 0x08000000 };
const uint32_t Dma::flagmasks[] = { 0x0000003d, 0x00000f40, 0x003d0000, 0x0f400000 };
SPI_TypeDef* const Spi::spis[] = {0, SPI1, SPI2, SPI3, SPI4, SPI5, SPI6};    // All SPIs
uint8_t const Spi::mux[] = {0, 37, 39, 61, 83, 85};                       // SPI RX connection to DMAMUX1
volatile uint32_t* const Spi::rccregs[] = { 0, &RCC->APB2ENR, &RCC->APB1LENR, &RCC->APB1LENR, &RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB4ENR };  // Mapping to RCC registers
uint32_t const Spi::rccbits[] = { 0, 0x00001000, 0x00004000, 0x00008000, 0x00002000, 0x00100000, 0x00000020 };                        // Mapping to RCC bits   
uint32_t const Spi::clocks[] = { 0, APB2_RATE, APB1_RATE, APB1_RATE, APB2_RATE, APB2_RATE, APB4_RATE};        // Connection to clock     

__attribute__((section(".ramD2"))) static uint32_t spiBufs[4 *12];
static uint32_t* spiInBuf[] = {spiBufs, spiBufs + 12};
static uint32_t* spiOutBuf[] = {spiBufs + 24, spiBufs + 36};

PmcuSpi pmcu[6];

void PmcuSpi::Init(uint8_t ispi, uint8_t idma) {
    spi = Spi::GetSpi(ispi);
    dma = Dma::GetDma(idma);
    rxStream = Dma::GetChannel(idma); txStream = Dma::GetChannel(idma+1);
    Dma::EnableClock(idma); Spi::EnableClock(ispi);
    Dma::SetMux(idma, Spi::GetMuxRxInp(ispi)); Dma::SetMux(idma+1, Spi::GetMuxTxInp(ispi));
    rxCompleteFlag = Dma::GetTcMask(idma); txCompleteFlag = Dma::GetTcMask(idma+1); 
    rxFlagMask = Dma::GetFlagMask(idma); txFlagMask = Dma::GetFlagMask(idma+1);
    rxFlags = Dma::GetFlagsReg(idma); txFlags = Dma::GetFlagsReg(idma+1);
    Spi::Init(spi, 0x00001001, 0x00000000, 0x3000c02f, 0x04400010); // Enable,start, endless, clock/16, dma, 8-byte fifo, 16-bit data, MSB first, CPOL/CPHA=00, DMA requests, master
    Dma::Init(rxStream, 0x00025400, 10, (void*)&spi->RXDR, spiInBuf[0]);
    Dma::Init(txStream, 0x00025440, 10, (void*)&spi->TXDR, spiOutBuf[0]);
}

void PmcuSpi::Tick() {
    uint32_t valid = IsReadComplete();
    uint32_t index = Time.Tick & 0x0000001;
    Spi::Stop(spi);
    *(rxFlags + 2) = rxFlagMask | txFlagMask;
    Dma::ReInit(rxStream, 10, spiInBuf[index]);
    Dma::ReInit(txStream, 10, spiOutBuf[index]);
    Spi::Start(spi);
}

void PmcuSpiInit() {
    pmcu[0].Init(5, 4); pmcu[1].Init(4, 6);
    spiOutBuf[0][0] = 0xA0A0;
    spiOutBuf[1][0] = 0xA0A0;
    for (int i=1; i<10; i++) {
        spiOutBuf[0][i] = i;
        spiOutBuf[1][i] = i;
    }
}

void PmcuSpiTickStart() {
    GPIOF->BSRR = 0x00000040;                  // F6 = 1 (NSS)
    for (int i=0; i<2; i++) pmcu[i].Tick();
    GPIOF->BSRR = 0x00400000;                  // F6 = 0 (NSS)
    for (int i=0; i<2; i++) pmcu[i].EnableDma();
}
void PmcuSpiTickEnd() {
}

