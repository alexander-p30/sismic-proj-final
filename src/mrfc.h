#ifndef SRC_MRFC_H_
#define SRC_MRFC_H_
#include <msp430.h>
#include <stdint.h>

//MF522 - Status
#define MI_OK                 0
#define MI_NOTAGERR           1
#define MI_ERR                2

// Port mapping:
// P3.0 = MOSI
// P3.1 = MISO
// P3.2 = UCB0CLK
// P1.3 = SDA/SS
// P1.4 = RST

// ========================================
// |                  SPI                 |
// ========================================

typedef struct {
    uint8_t polarity;
    uint8_t phase;
    uint8_t MSB_first;
    uint16_t brw;
} spi_config;

void configSPI(const spi_config *c);
uint8_t spiTransfer(uint8_t byte);

// ========================================
// |                 MRFC                 |
// ========================================

struct _MRFC_COMMANDS {
    uint8_t Idle;
    uint8_t Mem;
    uint8_t GenerateRandomID;
    uint8_t CalcCRC;
    uint8_t Transmit;
    uint8_t NoCmdChange;
    uint8_t Receive;
    uint8_t Transceive;
    uint8_t MFAuthent;
    uint8_t SoftReset;
};

extern const struct _MRFC_COMMANDS MRFC_COMMAND;

struct _MRFC_REGISTERS {
    uint8_t Command;
    uint8_t ComIEn;
    uint8_t ComIrq;
    uint8_t Error;
    uint8_t FIFOData;
    uint8_t FIFOLevel;
    uint8_t Control;
    uint8_t BitFraming;
    uint8_t Coll;
    uint8_t Mode;
    uint8_t TxMode;
    uint8_t RxMode;
    uint8_t ModWidth;
    uint8_t TMode;
    uint8_t TxControl;
    uint8_t TxASK;
    uint8_t TPrescaler;
    uint8_t TReloadHB;
    uint8_t TReloadLB;
    uint8_t AutoTest;
    uint8_t Version;
};

extern const struct _MRFC_REGISTERS MRFC_REGISTER;

void confMRFC();
uint8_t MRFCGetRegister(uint8_t reg);
void MRFCSetRegister(uint8_t reg, uint8_t value);
int8_t MRFCRequest();
void MRFCTest();

#endif
