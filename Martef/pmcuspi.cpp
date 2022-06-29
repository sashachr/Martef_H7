#include "chip.h"

#include "global.h"
#include "martef.h"
#include "timer.h"
#include "flash.h"
#include "adc.h"
#include "servo.h"
#include "pmcuspi.h"

float wdummy, rdummy;

typedef void (*SetServoVar)(ServoStruct* s, float v);

// Servo Parameters
const uint32_t ServoPars[] = {
/*  0 ENR       */ (uint8_t*)&Servo[0].RResolution - (uint8_t*)&Servo[0],
/*  1 ENR1      */ (uint8_t*)&Servo[0].LResolution - (uint8_t*)&Servo[0],
/*  2 ENOFF     */ 0,
/*  3 ENOFF1    */ 0,
/*  4 ENDL      */ (uint8_t*)&Servo[0].EncDiL - (uint8_t*)&Servo[0],
/*  5 PEL       */ (uint8_t*)&Servo[0].PeL - (uint8_t*)&Servo[0],
/*  6 PKP       */ (uint8_t*)&Servo[0].Ploop.Pi.Kp - (uint8_t*)&Servo[0],
/*  7 PKI       */ (uint8_t*)&Servo[0].Ploop.Pi.Ki - (uint8_t*)&Servo[0],
/*  8 PLI       */ (uint8_t*)&Servo[0].Ploop.Pi.Li - (uint8_t*)&Servo[0],
/*  9 VKP       */ (uint8_t*)&Servo[0].Vloop.Pi.Kp - (uint8_t*)&Servo[0],
/* 10 VKI       */ (uint8_t*)&Servo[0].Vloop.Pi.Ki - (uint8_t*)&Servo[0],
/* 11 VLI       */ (uint8_t*)&Servo[0].Vloop.Pi.Li - (uint8_t*)&Servo[0],
/* 12 CKP       */ (uint8_t*)&Servo[0].Cdloop.Pi.Kp - (uint8_t*)&Servo[0],
/* 13 CKI       */ (uint8_t*)&Servo[0].Cdloop.Pi.Ki - (uint8_t*)&Servo[0],
/* 14 CLI       */ (uint8_t*)&Servo[0].Cdloop.Pi.Li - (uint8_t*)&Servo[0],
/* 15 CKP       */ (uint8_t*)&Servo[0].Cdloop.Pi.Kp - (uint8_t*)&Servo[0],
/* 16 CKI       */ (uint8_t*)&Servo[0].Cdloop.Pi.Ki - (uint8_t*)&Servo[0],
/* 17 CLI       */ (uint8_t*)&Servo[0].Cdloop.Pi.Li - (uint8_t*)&Servo[0],
/* 18 BQA1      */ (uint8_t*)&Servo[0].Vloop.Bq[0].sA1 - (uint8_t*)&Servo[0],
/* 19 BQA2      */ (uint8_t*)&Servo[0].Vloop.Bq[0].sA2 - (uint8_t*)&Servo[0],
/* 20 BQB0      */ (uint8_t*)&Servo[0].Vloop.Bq[0].sB0 - (uint8_t*)&Servo[0],
/* 21 BQB1      */ (uint8_t*)&Servo[0].Vloop.Bq[0].sB1 - (uint8_t*)&Servo[0],
/* 22 BQB2      */ (uint8_t*)&Servo[0].Vloop.Bq[0].sB2 - (uint8_t*)&Servo[0],
/* 23 BQ1A1     */ (uint8_t*)&Servo[0].Vloop.Bq[1].sA1 - (uint8_t*)&Servo[0],
/* 24 BQ1A2     */ (uint8_t*)&Servo[0].Vloop.Bq[1].sA2 - (uint8_t*)&Servo[0],
/* 25 BQ1B0     */ (uint8_t*)&Servo[0].Vloop.Bq[1].sB0 - (uint8_t*)&Servo[0],
/* 26 BQ1B1     */ (uint8_t*)&Servo[0].Vloop.Bq[1].sB1 - (uint8_t*)&Servo[0],
/* 27 BQ1B2     */ (uint8_t*)&Servo[0].Vloop.Bq[1].sB2 - (uint8_t*)&Servo[0],
/* 28 BQ2A1     */ (uint8_t*)&Servo[0].Vloop.Bq[2].sA1 - (uint8_t*)&Servo[0],
/* 29 BQ2A2     */ (uint8_t*)&Servo[0].Vloop.Bq[2].sA2 - (uint8_t*)&Servo[0],
/* 30 BQ2B0     */ (uint8_t*)&Servo[0].Vloop.Bq[2].sB0 - (uint8_t*)&Servo[0],
/* 31 BQ2B1     */ (uint8_t*)&Servo[0].Vloop.Bq[2].sB1 - (uint8_t*)&Servo[0],
/* 32 BQ2B2     */ (uint8_t*)&Servo[0].Vloop.Bq[2].sB2 - (uint8_t*)&Servo[0],
/* 33 CURL      */ (uint8_t*)&Servo[0].CurL - (uint8_t*)&Servo[0],
/* 34 PWML      */ (uint8_t*)&Servo[0].PwmL - (uint8_t*)&Servo[0],
/* 35 COMP      */ (uint8_t*)&Servo[0].CommutPeriod - (uint8_t*)&Servo[0],
/* 37           */ 0,
/* 36           */ 0,
/* 38           */ 0,
/* 39           */ 0,
};

// Servo report variables
const SetServoVar ServoReps[] = {
/*  0 POUT      */ [](ServoStruct* s, float v) {if (s->RState & SM_POSITIONLOOP) s->POut = v;},
/*  1 VOUT      */ [](ServoStruct* s, float v) {if (s->RState & SM_VELOCITYLOOP) s->VOut = v;},
/*  2 CIN       */ [](ServoStruct* s, float v) {if (s->RState & SM_VELOCITYLOOP) s->CIn = v;},
/*  3 CDOUT     */ [](ServoStruct* s, float v) {if (s->RState & SM_CURRENTLOOP) s->CdOut = v;},
/*  4 CQOUT     */ [](ServoStruct* s, float v) {if (s->RState & SM_CURRENTLOOP) s->CqOut = v;},
/*  5 FCD       */ [](ServoStruct* s, float v) {s->FCd = v;},
/*  6 FCQ       */ [](ServoStruct* s, float v) {s->FCq = v;},
/*  7 OUTA      */ [](ServoStruct* s, float v) {s->OutA = v;},
/*  8 OUTB      */ [](ServoStruct* s, float v) {s->OutB = v;},
/*  9 OUTC      */ [](ServoStruct* s, float v) {s->OutC = v;},
/* 10 IPOS      */ [](ServoStruct* s, float v) {s->IndPos = v;},
/* 11 IPOS1     */ [](ServoStruct* s, float v) {s->IndPos1 = v;},
/* 12           */ [](ServoStruct* s, float v) {Adc.Ain[(s->Index << 2) + 0] = v; s->FCa = v;},
/* 13           */ [](ServoStruct* s, float v) {Adc.Ain[(s->Index << 2) + 1] = v; s->FCb = v;},
/* 14           */ [](ServoStruct* s, float v) {Adc.Ain[(s->Index << 2) + 2] = v;},
/* 15           */ [](ServoStruct* s, float v) {Adc.Ain[(s->Index << 2) + 3] = v;},
/* 16 COMA      */ [](ServoStruct* s, float v) {if (s->RState & SM_COMMUTATION) s->Teta = v;},
/* 17           */ [](ServoStruct* s, float v) {},
/* 18           */ [](ServoStruct* s, float v) {},
/* 19           */ [](ServoStruct* s, float v) {},
/* 20 POUT      */ [](ServoStruct* s, float v) {if (s->RState & SM_POSITIONLOOP) s->POut = v;},
/* 21 VOUT      */ [](ServoStruct* s, float v) {if (s->RState & SM_VELOCITYLOOP) s->VOut = v;},
/* 22 CIN       */ [](ServoStruct* s, float v) {if (s->RState & SM_VELOCITYLOOP) s->CIn = v;},
/* 23 CDOUT     */ [](ServoStruct* s, float v) {if (s->RState & SM_CURRENTLOOP) s->CdOut = v;},
/* 24 CQOUT     */ [](ServoStruct* s, float v) {if (s->RState & SM_CURRENTLOOP) s->CqOut = v;},
/* 25 FCD       */ [](ServoStruct* s, float v) {s->FCd = v;},
/* 26 FCQ       */ [](ServoStruct* s, float v) {s->FCq = v;},
/* 27 OUTA      */ [](ServoStruct* s, float v) {s->OutA = v;},
/* 28 OUTB      */ [](ServoStruct* s, float v) {s->OutB = v;},
/* 29 OUTC      */ [](ServoStruct* s, float v) {s->OutC = v;},
/* 30 IPOS      */ [](ServoStruct* s, float v) {s->IndPos = v;},
/* 31 IPOS1     */ [](ServoStruct* s, float v) {s->IndPos1 = v;},
/* 32           */ [](ServoStruct* s, float v) {Adc.Ain[(s->Index << 2) + 0] = v; s->FCa = v;},
/* 33           */ [](ServoStruct* s, float v) {Adc.Ain[(s->Index << 2) + 1] = v; s->FCb = v;},
/* 34           */ [](ServoStruct* s, float v) {Adc.Ain[(s->Index << 2) + 2] = v;},
/* 35           */ [](ServoStruct* s, float v) {Adc.Ain[(s->Index << 2) + 3] = v;},
/* 36 COMA      */ [](ServoStruct* s, float v) {if (s->RState & SM_COMMUTATION) s->Teta = v;},
/* 37           */ [](ServoStruct* s, float v) {},
/* 38           */ [](ServoStruct* s, float v) {},
/* 39           */ [](ServoStruct* s, float v) {},
};

const uint32_t Dma::tcmasks[] = { 0x00000020, 0x00000800, 0x00200000, 0x08000000 };
const uint32_t Dma::flagmasks[] = { 0x0000003d, 0x00000f40, 0x003d0000, 0x0f400000 };
SPI_TypeDef* const Spi::spis[] = {0, SPI1, SPI2, SPI3, SPI4, SPI5, SPI6};    // All SPIs
uint8_t const Spi::mux[] = {0, 37, 39, 61, 83, 85};                       // SPI RX connection to DMAMUX1
volatile uint32_t* const Spi::rccregs[] = { 0, &RCC->APB2ENR, &RCC->APB1LENR, &RCC->APB1LENR, &RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB4ENR };  // Mapping to RCC registers
uint32_t const Spi::rccbits[] = { 0, 0x00001000, 0x00004000, 0x00008000, 0x00002000, 0x00100000, 0x00000020 };                        // Mapping to RCC bits   
uint32_t const Spi::clocks[] = { 0, APB2_RATE, APB1_RATE, APB1_RATE, APB2_RATE, APB2_RATE, APB4_RATE};        // Connection to clock     

__attribute__((section(".ramD1"))) static uint32_t spiBufs[4 * 12 * NAX];
//static uint32_t* spiInBuf[] = {spiBufs, spiBufs + 12};
//static uint32_t* spiOutBuf[] = {spiBufs + 24, spiBufs + 36};

PmcuSpi pmcu[NAX];

static uint32_t trace[1024];
static uint32_t itrace;
static uint32_t nexti(uint32_t i) { if (++i == 1024) i = 0; return i; }
static uint32_t previ(uint32_t i) { if (i-- == 0) i = 1023; return i; }
static void Trace(uint32_t t) {trace[itrace] = (uint32_t)t; itrace = nexti(itrace);}

void PmcuSpi::Init(uint8_t ind, uint8_t ispi, uint8_t idma) {
    index  = ind;
    servo = &Servo[ind];
    inBuf[0] = spiBufs + 4 * 12 * index;
    inBuf[1] = inBuf[0] + 12; outBuf[0] = inBuf[1] + 12; outBuf[1] = outBuf[0] + 12;
    spi = Spi::GetSpi(ispi);
    dma = Dma::GetDma(idma);
    rxStream = Dma::GetChannel(idma); txStream = Dma::GetChannel(idma+1);
    Dma::EnableClock(idma); Spi::EnableClock(ispi);
    Dma::SetMux(idma, Spi::GetMuxRxInp(ispi)); Dma::SetMux(idma+1, Spi::GetMuxTxInp(ispi));
    rxCompleteFlag = Dma::GetTcMask(idma); txCompleteFlag = Dma::GetTcMask(idma+1); 
    rxFlagMask = Dma::GetFlagMask(idma); txFlagMask = Dma::GetFlagMask(idma+1);
    rxFlags = Dma::GetFlagsReg(idma); txFlags = Dma::GetFlagsReg(idma+1);
    // Spi::Init(spi, 0x00001001, 0x00000014, (ispi == 2) ? 0x404fc02f : 0x304fc02f, 0x04400010); // Enable,start, 20 frames, clock/16 (or 32 for SPI2, not clear why), dma, 8-byte fifo, 16-bit data, MSB first, CPOL/CPHA=00, DMA requests, master
    Spi::Init(spi, 0x00001000, 0x00000000, (ispi == 2) ? 0x4000c02f : 0x3000c02f, 0x04400010); // Enable,start, endless, clock/16, dma, 8-byte fifo, 16-bit data, MSB first, CPOL/CPHA=00, DMA requests, master
    Dma::Init(rxStream, 0x00025400, 10, (void*)&spi->RXDR, inBuf[0]);
    Dma::Init(txStream, 0x00025440, 10, (void*)&spi->TXDR, outBuf[0]);
    iTick = iBuf = 0;
}

void PmcuSpi::TickStart() {
	uint32_t flg, rndtr, tndtr;
	if (index == 0) {
		flg = *rxFlags; rndtr = rxStream->NDTR; tndtr = txStream->NDTR;
		cnt++;
	}
    Valid = IsReadComplete();
    if (!Valid) {
        Trace(cnt);
        cnt = 0;
    }
    // uint32_t index = Time.Tick & 0x0000001;
    Dma::Disable(rxStream); Dma::Disable(txStream); 
    *(rxFlags + 2) = rxFlagMask | txFlagMask;
    Spi::Stop(spi);
    Dma::ReInit(rxStream, 10, inBuf[iBuf]);
    Dma::ReInit(txStream, 10, outBuf[iBuf]);
    Dma::Enable(rxStream);
    Spi::EnableDma(spi);
    Spi::Start(spi);
    iBuf ^= 1;
}

void PmcuSpi::TickEnd() {
    EncipherCommand(outBuf[iBuf]);
}

void PmcuSpi::DecipherReport() {
	if (servo->Index > 1) return;
    uint32_t* buf = inBuf[iBuf];
    servo->FState = buf[0];
    if (((uint16_t*)&servo->FState)[1]) faultcount++; else faultcount = 0;
    if (faultcount < 3) ((uint16_t*)&servo->FState)[1] = 0; 
    servo->FPos = *(float*)(buf+1);
    servo->FPos1 = *(float*)(buf+2);
    servo->FCd = *(float*)(buf+3);
    servo->FCq = *(float*)(buf+4);
//    servo->RPosF = *(float*)(buf+5);
    int i = iTick - 4;
    if (i < 0) i += 10;
    const SetServoVar* rep = ServoReps+(i<<2);
    float* v = (float*)(buf+6);
    for (int i=0; i<4; i++) (*rep++)(servo, *v++);
}

void PmcuSpi::EncipherCommand(uint32_t* buf) {
    uint32_t stat = servo->RState;
    buf[0] = stat;
    if (stat & SM_SETFPOS) {
    	*(float*)(buf+1) = servo->FPos;
        servo->RState &= ~SM_SETFPOS;
    } else if (stat & SM_POSITIONLOOP) {
    	*(float*)(buf+1) = servo->RPos;
        *(float*)(buf+2) = servo->RVel;
    } else if (stat & SM_VELOCITYLOOP) {
    	*(float*)(buf+1) = servo->VIn;
        *(float*)(buf+2) = 0;
    } else if (stat & SM_CURRENTLOOP) {
    	*(float*)(buf+1) = servo->CIn;
        *(float*)(buf+2) = servo->Teta;
    } else if (stat & SM_PWM) {
    	*(float*)(buf+1) = servo->CqOut;
        *(float*)(buf+2) = servo->Teta;
    }
    *(buf+5) = iTick;
    uint32_t *b = buf+6, *par = (uint32_t*)(ServoPars+(iTick<<2));
    for (int i=0; i<4; i++) *b++ = *(uint32_t*)((uint8_t*)servo + *par++);
    if (++iTick >= 10) iTick = 0;
}

void PmcuSpiTickStart() {
    GPIOF->BSRR = 0x00004040;                  // F6/14 = 1 (NSS)
    for (int i=0; i<2; i++) pmcu[i].TickStart();
    GPIOF->BSRR = 0x40400000;                  // F6/14 = 0 (NSS)
    for (int i=0; i<2; i++) pmcu[i].StartTransfer();
    for (int i=0; i<2; i++) if (pmcu[i].Valid) pmcu[i].DecipherReport();
}
void PmcuSpiTickEnd() {
    for (int i=0; i<2; i++) pmcu[i].TickEnd();
}

// ------------------ Start up validation/downloading

uint8_t PmcuFailure[6];
static uint32_t dllog[1024];
static uint32_t ilog;

void PmcuSpi::DownInit(uint8_t ind, uint8_t ispi) {
    index  = ind;
    spi = Spi::GetSpi(ispi);
    Spi::EnableClock(ispi);
    Spi::Init(spi, 0x00001201, 0x00000000, (ispi == 2) ? 0x70000007 : 0x60000007, 0x04400010); // Enable,start, endless, 1 MHz, 8-byte fifo, 8-bit data, MSB first, CPOL/CPHA=00, master
}
#define TransactonTime  0.000008
uint8_t PmcuSpi::DownGetAck(float timeout) {
    Spi::SendReceive(spi, 0);
    uint32_t i = 0, repeat = timeout ? (uint32_t)ceil(timeout/TransactonTime) : 10;
    uint8_t r = 0;
    while ((r != 0x79) && (r != 0x1F) && (++i < repeat)) {
        r = Spi::SendReceive(spi, 0);
    }
    Spi::SendReceive(spi, 0x79);
    dllog[ilog++] = r;
    if (ilog == 1024) ilog = 0;
    dllog[ilog++] = i;
    if (ilog == 1024) ilog = 0;
    if (r == 0x79)
    	return 1;
    else if (r != 0x1F)
    	return 0;
    else
    	return 0;
}
uint8_t PmcuSpi::DownSynchro() {
    while (1) {
        Spi::SendReceive(spi, 0x5A);
        if (DownGetAck(0)) {
                dllog[ilog++] = 1;
                if (ilog == 1024) ilog = 0;
            return 1;
        }
    }
                dllog[ilog++] = 0;
                if (ilog == 1024) ilog = 0;
    return 0;
}
uint8_t PmcuSpi::DownCommand(uint8_t com) {
    Spi::SendReceive(spi, 0x5A);
    Spi::SendReceive(spi, com);
    Spi::SendReceive(spi, ~com);
    return DownGetAck(0);
}
uint8_t PmcuSpi::DownAddress(uint32_t addr) {
    uint8_t cs = 0, c;
    for (int i = 0; i < 4; i++) {
        uint8_t c = (uint8_t)((addr >> (24 - i*8)) & 0x000000FF);
        Spi::SendReceive(spi, c);
        cs ^= c;
    }
    Spi::SendReceive(spi, cs);
    return DownGetAck(0);
}
uint8_t PmcuSpi::DownBlock(uint8_t* buf, int count, uint8_t cs) {
    for (int i = 0; i < count; i++) {
        Spi::SendReceive(spi, *buf);
        cs ^= *buf++;
    }
    Spi::SendReceive(spi, cs);
}
uint8_t PmcuSpi::DownChunk(uint8_t* buf, int count, uint32_t addr) {
                dllog[ilog++] = 0x31;
                if (ilog == 1024) ilog = 0;
                dllog[ilog++] = addr;
                if (ilog == 1024) ilog = 0;
    if (!DownCommand(0x31)) {
                dllog[ilog++] = 0;
                if (ilog == 1024) ilog = 0;
        return 0;   // write memory
    }
    if (!DownAddress(addr)) {
                dllog[ilog++] = 1;
                if (ilog == 1024) ilog = 0;
        return 0;
    }
    uint8_t c = (uint8_t)((count - 1) & 0x000000FF);
    Spi::SendReceive(spi, c);
    DownBlock(buf, count, c);
    uint8_t r = DownGetAck(1);
                dllog[ilog++] = r;
                if (ilog == 1024) ilog = 0;
    return r;
}
uint8_t PmcuSpi::Download(uint8_t* buf, int count, uint32_t addr) {
    DownSynchro();
    while (count > 0) {
        int c = (count < 256) ? count : 256;
        if (!DownChunk(buf, c, addr))
        	return 0;
        buf += c; addr += c; count -= c;
    }
    return 1;
}
uint8_t PmcuSpi::DownValidate(uint32_t addr, uint32_t count, uint32_t cs) {
    if (!DownCommand(0xA2)) return 0;   // validate
    uint32_t b[] = { addr, count, cs};
    DownBlock((uint8_t*)b, 12, 0);
    uint8_t r = DownGetAck(1);
    return r;
}
uint8_t PmcuSpi::DownStart(uint32_t addr) {
    if (!DownCommand(0x21)) return 0;   // go
    if (!DownAddress(addr)) return 0;
    return 1;
}

void PmcuSpiInit() {
    pmcu[0].Init(0, 2, 4); pmcu[1].Init(1, 5, 6);
}

void PmcuUpgrade() {
    extern int32_t _spmcu, _epmcu;
    uint32_t *_s = (uint32_t*)&_spmcu, *_e = (uint32_t*)&_epmcu;
    // while (*(_e - 1) == 0xFFFFFFFFFFFFFFFF) _e--;
    int len = (_e - _s) << 2;
    uint32_t cs = FlashCalculateCrc(_s, len);
    GPIOF->BSRR = 0x00004040;                  // F6/14 = 1 (NSS)
    pmcu[0].DownInit(0, 2); pmcu[1].DownInit(1, 5);
    GPIOF->BSRR = 0x40400000;                  // F6/14 = 0 (NSS)
    if (!pmcu[0].DownValidate(0x08002000, len, cs)) {
        // pmcu[0].Download((uint8_t*)_s, len, 0x08002000);
        // PmcuFailure[0] = !pmcu[0].DownValidate(0x08002000, len, cs);
    }
    if (!pmcu[1].DownValidate(0x08002000, len, cs)) {
        pmcu[1].Download((uint8_t*)_s, len, 0x08002000);
        PmcuFailure[1] = !pmcu[1].DownValidate(0x08002000, len, cs);
    }
    // pmcu[0].DownStart(0x08002000);
    pmcu[1].DownStart(0x08002000);
    GPIOF->BSRR = 0x00004040;                  // F6/14 = 1 (NSS)
}

