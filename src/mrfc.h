#ifndef SRC_MRFC_H_
#define SRC_MRFC_H_
#include <msp430.h>
#include <stdint.h>

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
    uint8_t Version;
};

extern const struct _MRFC_REGISTERS MRFC_REGISTER;

void configMRFC();
uint8_t MRFCReadRegister(uint8_t reg);
void MRFCWriteRegister(uint8_t reg, uint8_t value);

#endif
