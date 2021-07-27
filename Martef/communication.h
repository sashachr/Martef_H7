#pragma once

// MDMA services
struct MdmaLink {
    uint32_t  CTCR;      // Transfer Configuration register,       Address offset: 0x50
    uint32_t  CBNDTR;    // block number of data register,         Address offset: 0x54
    uint32_t  CSAR;      // source address register,               Address offset: 0x58
    uint32_t  CDAR;      // destination address register,          Address offset: 0x5C
    uint32_t  CBRUR;     // Block Repeat address Update register,  Address offset: 0x60
    uint32_t  CLAR;      // Link Address register,                 Address offset: 0x64
    uint32_t  CTBR;      // Trigger and Bus selection Register,    Address offset: 0x68
    uint32_t  CMAR;      // Mask address register,                 Address offset: 0x70
    uint32_t  CMDR;      // Mask Data register,                    Address offset: 0x74
    uint32_t  fill;      // to provide alignment 8
};
class Mdma {
    public: static void InitLink(MdmaLink& ml, uint32_t CTCR, uint32_t CBNDTR, uint32_t CSAR = 0, uint32_t CDAR = 0, uint32_t CBRUR = 0, uint32_t CLAR = 0, uint32_t CTBR = 0, uint32_t CMAR = 0, uint32_t CMDR = 0) {
        ml.CTCR = CTCR; ml.CBNDTR = CBNDTR; ml.CSAR = CSAR; ml.CDAR = CDAR; ml.CBRUR = CBRUR; ml.CLAR = CLAR; ml.CTBR = CTBR; ml.CMAR = CMAR; ml.CMDR = CMDR;  
    }
    public: static void InitHard(MDMA_Channel_TypeDef* c, uint32_t CCR, uint32_t CTCR, uint32_t CBNDTR, uint32_t CSAR, uint32_t CDAR, uint32_t CBRUR, uint32_t CLAR, uint32_t CTBR, uint32_t CMAR, uint32_t CMDR) {
        c->CCR = CCR; c->CTCR = CTCR; c->CBNDTR = CBNDTR; c->CSAR = CSAR; c->CDAR = CDAR; c->CBRUR = CBRUR; c->CLAR = CLAR; c->CTBR = CTBR; c->CMAR = CMAR; c->CMDR = CMDR;  
    }
    public: static void InitHard(MDMA_Channel_TypeDef* c, uint32_t CCR, MdmaLink& ml) {
        c->CCR &= ~1;       // Disable channel
        while (c->CCR & 1) ;
        c->CIFCR = 0x0000001F;  // Clear flags
        c->CCR = CCR; c->CTCR = ml.CTCR; c->CBNDTR = ml.CBNDTR; c->CSAR = ml.CSAR; c->CDAR = ml.CDAR; c->CBRUR = ml.CBRUR; c->CLAR = ml.CLAR; c->CTBR = ml.CTBR; c->CMAR = ml.CMAR; c->CMDR = ml.CMDR;  
    }
};

class CommChannel {
    public: uint8_t *Inbuf, *Outbuf; 
    public: uint16_t InbufSize, OutbufSize; 
	protected: uint8_t Channel;
    public: MDMA_Channel_TypeDef* RxMdma;
    public: MDMA_Channel_TypeDef* TxMdma;
	protected: CommChannel(uint8_t index) : Channel(index) {}
    public: virtual void Tick() = 0;
    public: virtual int StartRead() {return 0;}
    public: virtual int StartWrite() {return 0;}
};

class CommChannelDma : public CommChannel {
    DMA_TypeDef* DmaRegs;
    DMA_Stream_TypeDef* RxStream;    
    DMA_Stream_TypeDef* TxStream; 
    uint32_t volatile* TxStatus;
    uint32_t TxClearMask, TxReadyMask;
    uint32_t volatile* RxStatus;
    uint32_t RxClearMask, RxReadyMask;

	protected: CommChannelDma(uint8_t index, uint8_t dma, uint8_t rxstream, uint8_t rxchannel, uint8_t txstream, uint8_t txchannel);
    protected: void DmaInit(int dma, int rxstream, int rxchannel, int txstream, int txchannel) {}
    protected: void StartReadDma(void* periphreg, void* buf, int len);
    protected: void StartWriteDma(void* periphreg, void* buf, int len);
    protected: int ReadCount() {return RxStream->NDTR;}
    protected: int WriteCount() {return TxStream->NDTR;}
    private: void FindFlags(uint8_t dma, uint8_t stream, uint32_t volatile** flags, uint32_t* completemask, uint32_t* clearmask);
};

void CommunicationTick();
void CommunicationInit();

extern CommChannel* CommChannels[];