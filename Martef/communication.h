#pragma once

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
};

// Channels
#define CH_COMM     0

class CommChannel {
    public: uint8_t *Inbuf, *Outbuf; 
	protected: uint8_t Channel;
	protected: CommChannel(uint8_t index) : Channel(index) {}
    public: virtual void Tick() = 0;
    public: virtual int StartRead(void* buf, int len) {return 0;}
    public: virtual int StartWrite(void* buf, int len) {return 0;}
};
extern struct MdmaLink MdmaLinkBuf[50];

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