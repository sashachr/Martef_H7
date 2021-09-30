#include "chip.h"

#include "adc.h"

AdcStruct Adc;
uint32_t adcBuf[2000];

// Channels mapping
// Ain      Signal          Pin     ADC Channel
// 0        User_Analog_In  PC4	    1   14                ADC12_IN14		
// 1        ID_OUT	        PA2	    2   2                ADC123_IN2		
// 2        OV1	            PA3	    1   3                ADC123_IN3		
// 3        24V_CHECK	    PA5	    2   5                ADC12_IN5		
// 4        3V3_CHECK	    PA6	    1   6                ADC12_IN6		
// 5        5V_CHECK	    PA7	    2   7                ADC12_IN7		
// 6        12V_CHECK	    PC5	    1   15                ADC12_IN15		
// 7        24VCM	        PC2	    2   12                ADC123_IN12		
// 8        24VDC_CM	    PC3	    1   13                ADC123_IN13		
// 9        ADC_AIN_backup	PB0	    2   8                ADC12_IN8		
// 10       Temperature 		    1   16                ADC1_IN16		
// 11       ADC_0.5_VREF	PB1	    2   9                ADC12_IN9		

float AdcStruct::AinMin[24] = {0,   0,   0,  48, 40, 60, 54, 0, 0, 0, 0, 58};
float AdcStruct::AinMax[24] = {100, 100, 50, 73, 60, 91, 82, 100, 72, 100, 100, 66};
float AdcStruct::Filter[24] = {1, 0.2, 0.2, 0.085, 0.085, 0.085, 0.085, 1, 0.085, 1, 1, 0.02};
// float AdcStruct::filter1[12];

void AdcStruct::Init()
{
    RCC->AHB1ENR |= 0x00400000;                            			// Enable DMA2 clocks
    RCC->APB2ENR |= 0x00000300;                            			// Enable ADC1/ADC2 clocks

    DMA2_Stream0->CR &= ~1;			            // Disable stream, enable configuration
    DMA2_Stream0->CR = 0x00025400;              // Channel 0, no burst, high priority, 32-bit size, memory increment, circular mode, peripheral to memory
    DMA2_Stream0->FCR = 0x00000003;		        // Fifo disabled
    DMA2_Stream0->NDTR = 200;			        // Buffer size
    DMA2_Stream0->PAR = (uint32_t)&ADC12_COMMON->CDR;    // Peripheral address
    DMA2_Stream0->M0AR = (uint32_t)adcBuf;      // Memory address
    DMA2_Stream0->CR |= 1;			            // Enable stream
    
    ADC12_COMMON->CCR = 0x00818006;     // Dual mode, prescaler PCLK2/4 (PCLK2=84 MHz), Temperature sensor enabled
    // ADC1->CR1 = 0x00000100; ADC2->CR1 = 0x00000100;		// Scan mode
    // ADC1->CR2 = 0x00000302; ADC2->CR2 = 0x00000002;    // Continuous mode, DMA request
    ADC1->SQR1 = 5 << 20; ADC2->SQR1 = 5 << 20;  // 2*6 conversions in cycle
    ADC1->SQR3 = 14 | (3 << 5) | (6 << 10) | (15 << 15) | (13 << 20) | (16 << 25); // Channels 14,3,6,15,13,16
	ADC2->SQR3 = 2 | (5 << 5) | (7 << 10) | (12 << 15) | (8 << 20) | (9 << 25); // Channels 2,5,7,12,8,9           
    ADC1->SMPR1 = 0; ADC2->SMPR1 = 0;            // 3 cycles sampling time
    // ADC1->CR2 |= 1; ADC2->CR2 |= 1;             // Enable ADC1/ADC2
    // ADC1->CR2 |= 0x40000000;    // Start conversion
    first = 1;
}

int16_t nBuf[100];
int16_t nBufInd;

void AdcStruct::Tick() {
    // ADC1->CR2 &= ~1; ADC2->CR2 &= ~1;
    DMA2_Stream0->CR &= ~1;			            // Disable stream, enable configuration
    int n = 200-DMA2_Stream0->NDTR;
    nBuf[nBufInd]=n;
    if (++nBufInd==100) nBufInd=0;
    if (n >= 48) {
	  	uint32_t sum[6]; 
		uint32_t* psum = sum;
		uint32_t* pbuf = adcBuf + ((n/6)*6 - 48);
		for (int j=0; j<6; j++) *psum++ = 0;
		for (int i=0; i<8; i++) {	// oversampling 8
		  	psum = sum;
			for (int j=0; j<6; j++) *psum++ += *pbuf++;		// 6 pairs of values
		}
		float *ain = Ain, *flt = Filter; //, *f1 = filter1;
        uint16_t* ps = (uint16_t*)sum;
        if (first) {
            first = 0;
            for (int j=0; j<12; j++) {
                float v = *ps++ * 0.00305250305F;   // 100.0/4095/8
                *ain = v;		
                ain++;
            }
        } else {
            for (int j=0; j<12; j++) {
                float v = *ps++ * 0.00305250305F;   // 100.0/4095/8
                float fi = *flt++;
                *ain = (1.0F-fi)*(*ain) + fi*v;		// First order filter
                ain++;
            }
        }
    } else {
        for (int j=0; j<12; j++) Ain[j] = 0;
    }
    DMA2->LIFCR = 0x0000003D;
    DMA2_Stream0->NDTR = 200;			        // Buffer size
    DMA2_Stream0->M0AR = (uint32_t)adcBuf;      // Memory address
    DMA2_Stream0->CR |= 1;			            // Enable stream
    // ADC1->CR2 |= 1; ADC2->CR2 |= 1;            // Enable ADC1
    // ADC1->CR2 |= 0x40000000; ADC2->CR2 |= 0x40000000;    // Start conversion
}

void AdcStruct::SetMaxCurrent24V(float ampers) {
    AinMax[7] = ampers * (0.4f * 100 / 3.3f);
}
