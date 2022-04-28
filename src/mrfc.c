#include "mrfc.h"

// ========================================
// |                  SPI                 |
// ========================================

// P3.0 = MOSI; P3.1 = MISO; P3.2 = UCB0CLK
void configSPI(const spi_config *c) {
    // desliga o modulo
    UCB0CTL1 = UCSWRST;

    UCB0CTL0 = UCMODE_0 | UCSYNC;

    UCB0CTL1 |= UCSSEL__SMCLK;
    UCB0BRW = c->brw;

    UCB0CTL0 |= UCMST;

    if (c->polarity)
        UCB0CTL0 |= UCCKPL;
    if (c->phase)
        UCB0CTL0 |= UCCKPH;
    if (c->MSB_first)
        UCB0CTL0 |= UCMSB;

    P3SEL |= BIT0 | BIT1 | BIT2;

    UCB0CTL1 &= ~UCSWRST;
}

uint8_t spiTransfer(uint8_t byte) {
    while ((UCB0IFG & UCTXIFG) == 0);
    UCB0TXBUF = byte;

    while ((UCB0IFG & UCRXIFG) == 0);
    return UCB0RXBUF;
}

// ========================================
// |                 MRFC                 |
// ========================================

const struct _MRFC_COMMANDS MRFC_COMMAND = { .Idle = 0b0000, .Mem = 0b0001, .GenerateRandomID = 0b0010,
                   .CalcCRC = 0b0011, .Transmit = 0b0100, .NoCmdChange = 0b0111,
                   .Receive = 0b1000, .Transceive = 0b1100, .MFAuthent = 0b1110,
                   .SoftReset = 0b1111 };

const struct _MRFC_REGISTERS MRFC_REGISTER = { .Version = 0x37 };

void configMRFC() {
    // SDA/SS
    P1DIR |= BIT3;
    P1OUT |= BIT3;

    // RST
    P1DIR |= BIT4;
    P1OUT |= BIT4;

    const spi_config masterSPIConfig = { .polarity = 0, .phase = 1, .MSB_first = 1, .brw = 2 };

    configSPI(&masterSPIConfig);
}

void __setSS() {
    P1OUT &= ~BIT3;
}

void __unsetSS() {
    P1OUT |= BIT3;
}

uint8_t MRFCReadRegister(uint8_t reg) {
    __setSS();

    // transfer register address, setting read mode, response is DONT CARE
    spiTransfer(0b10000000 | (reg << 1) & 0b11111110);

    // transfer 0 to get response from previous transfer
    uint8_t regVal = spiTransfer(0);

    __unsetSS();

    return regVal;
}

void MRFCWriteRegister(uint8_t reg, uint8_t value) {
    __setSS();

    // transfer register address, setting write mode, response is DONT CARE
    spiTransfer((reg << 1) & 0b01111110);

    // transfer value to be written to register;
    spiTransfer(value);

    __unsetSS();
}
