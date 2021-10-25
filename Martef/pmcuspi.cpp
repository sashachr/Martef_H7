#include "chip.h"

#include "global.h"
#include "martef.h"
#include "timer.h"
#include "adc.h"
#include "encoder.h"
#include "servo.h"
#include "pmcuspi.h"

float wdummy, rdummy;

typedef void (*SetServoVar)(ServoStruct* s, float v);

// Servo Parameters
const uint32_t ServoPars[] = {
/*  0 ENR       */ (uint8_t*)&Servo[0].REncoder.Resolution - (uint8_t*)&Servo[0],
/*  1 ENR1      */ (uint8_t*)&Servo[0].LEncoder.Resolution - (uint8_t*)&Servo[0],
/*  2 ENOFF     */ (uint8_t*)&Servo[0].REncoder.Offset - (uint8_t*)&Servo[0],
/*  3 ENOFF1    */ (uint8_t*)&Servo[0].LEncoder.Offset - (uint8_t*)&Servo[0],
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
/* 18 BQA1      */ (uint8_t*)&Servo[0].Vloop.Bq[0].A1 - (uint8_t*)&Servo[0],
/* 19 BQA2      */ (uint8_t*)&Servo[0].Vloop.Bq[0].A2 - (uint8_t*)&Servo[0],
/* 20 BQB0      */ (uint8_t*)&Servo[0].Vloop.Bq[0].B0 - (uint8_t*)&Servo[0],
/* 21 BQB1      */ (uint8_t*)&Servo[0].Vloop.Bq[0].B1 - (uint8_t*)&Servo[0],
/* 22 BQB2      */ (uint8_t*)&Servo[0].Vloop.Bq[0].B2 - (uint8_t*)&Servo[0],
/* 23 BQ1A1     */ (uint8_t*)&Servo[0].Vloop.Bq[1].A1 - (uint8_t*)&Servo[0],
/* 24 BQ1A2     */ (uint8_t*)&Servo[0].Vloop.Bq[1].A2 - (uint8_t*)&Servo[0],
/* 25 BQ1B0     */ (uint8_t*)&Servo[0].Vloop.Bq[1].B0 - (uint8_t*)&Servo[0],
/* 26 BQ1B1     */ (uint8_t*)&Servo[0].Vloop.Bq[1].B1 - (uint8_t*)&Servo[0],
/* 27 BQ1B2     */ (uint8_t*)&Servo[0].Vloop.Bq[1].B2 - (uint8_t*)&Servo[0],
/* 28 BQ2A1     */ (uint8_t*)&Servo[0].Vloop.Bq[2].A1 - (uint8_t*)&Servo[0],
/* 29 BQ2A2     */ (uint8_t*)&Servo[0].Vloop.Bq[2].A2 - (uint8_t*)&Servo[0],
/* 30 BQ2B0     */ (uint8_t*)&Servo[0].Vloop.Bq[2].B0 - (uint8_t*)&Servo[0],
/* 31 BQ2B1     */ (uint8_t*)&Servo[0].Vloop.Bq[2].B1 - (uint8_t*)&Servo[0],
/* 32 BQ2B2     */ (uint8_t*)&Servo[0].Vloop.Bq[2].B2 - (uint8_t*)&Servo[0],
/* 33 CURL      */ (uint8_t*)&Servo[0].CurL - (uint8_t*)&Servo[0],
/* 34 PWML      */ (uint8_t*)&Servo[0].PwmL - (uint8_t*)&Servo[0],
/* 35 COMP      */ (uint8_t*)&Servo[0].Commut.Period - (uint8_t*)&Servo[0],
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

__attribute__((section(".ramD2"))) static uint32_t spiBufs[4 * 12 * NAX];
//static uint32_t* spiInBuf[] = {spiBufs, spiBufs + 12};
//static uint32_t* spiOutBuf[] = {spiBufs + 24, spiBufs + 36};

PmcuSpi pmcu[NAX];

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
    Spi::Init(spi, 0x00001001, 0x00000000, 0x3000c02f, 0x04400010); // Enable,start, endless, clock/16, dma, 8-byte fifo, 16-bit data, MSB first, CPOL/CPHA=00, DMA requests, master
    Dma::Init(rxStream, 0x00025400, 10, (void*)&spi->RXDR, inBuf[0]);
    Dma::Init(txStream, 0x00025440, 10, (void*)&spi->TXDR, outBuf[0]);
    iTick = iBuf = 0;
}

void PmcuSpi::TickStart() {
    uint32_t valid = IsReadComplete();
    // uint32_t index = Time.Tick & 0x0000001;
    Spi::Stop(spi);
    *(rxFlags + 2) = rxFlagMask | txFlagMask;
    Dma::ReInit(rxStream, 10, inBuf[iBuf]);
    Dma::ReInit(txStream, 10, outBuf[iBuf]);
    iBuf ^= 1;
    Spi::Start(spi);
    DecipherReport(inBuf[iBuf]);
}

void PmcuSpi::TickEnd() {
    EncipherCommand(outBuf[iBuf]);
}

void PmcuSpi::DecipherReport(uint32_t* buf) {
	if (servo->Index != 0) return;
    servo->FState = buf[0];
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
    GPIOF->BSRR = 0x00000040;                  // F6 = 1 (NSS)
    for (int i=0; i<2; i++) pmcu[i].TickStart();
    GPIOF->BSRR = 0x00400000;                  // F6 = 0 (NSS)
    for (int i=0; i<2; i++) pmcu[i].EnableDma();
}
void PmcuSpiTickEnd() {
    for (int i=0; i<2; i++) pmcu[i].TickEnd();
}

void PmcuSpiInit() {
    pmcu[0].Init(0, 5, 4); pmcu[1].Init(1, 4, 6);
//    spiOutBuf[0][0] = 0xA0A0;
//    spiOutBuf[1][0] = 0xA0A0;
//    for (int i=1; i<10; i++) {
//        spiOutBuf[0][i] = i;
//        spiOutBuf[1][i] = i;
//    }
}

