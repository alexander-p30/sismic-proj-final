#include "mrfc.h"
#include <stdint.h>

// ========================================
// |                  SPI                 |
// ========================================´

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

#define RX_INTERRUPT BIT5
#define TIMER_INTERRUPT BIT0

uint8_t MRFCSetRegBits(uint8_t reg, uint8_t bits) {
    MRFCSetRegister(reg, MRFCGetRegister(reg) | bits);
    return MRFCGetRegister(reg);
}

uint8_t MRFCUnsetRegBits(uint8_t reg, uint8_t bits) {
    MRFCSetRegister(reg, MRFCGetRegister(reg) & (~bits));
    return MRFCGetRegister(reg);
}

const struct _PICC_COMMANDS PICC_COMMAND = { .ReqA = 0x26, .AnticollisionCL1 = 0x93 };
// internal alias
const struct _PICC_COMMANDS *PCMD = &PICC_COMMAND;

const struct _MRFC_COMMANDS MRFC_COMMAND = { .Idle = 0b0000, .Mem = 0b0001,
                                             .GenerateRandomID = 0b0010,
                                             .CalcCRC = 0b0011, .Transmit =
                                                     0b0100,
                                             .NoCmdChange = 0b0111, .Receive =
                                                     0b1000,
                                             .Transceive = 0b1100, .MFAuthent =
                                                     0b1110,
                                             .SoftReset = 0b1111 };
// internal alias
const struct _MRFC_COMMANDS *CMD = &MRFC_COMMAND;

const struct _MRFC_REGISTERS MRFC_REGISTER = { .Command = 0x01, .ComIEn = 0x02,
                                               .ComIrq = 0x04, .Error = 0x06,
                                               .FIFOData = 0x09, .FIFOLevel =
                                                       0x0A,
                                               .Control = 0x0C, .BitFraming =
                                                       0x0D,
                                               .Coll = 0x0E, .Mode = 0x11,
                                               .TxMode = 0x12, .RxMode = 0x13,
                                               .TxControl = 0x14, .TxASK = 0x15,
                                               .ModWidth = 0x24, .TMode = 0x2A,
                                               .TPrescaler = 0x2B, .TReloadHB =
                                                       0x2C,
                                               .TReloadLB = 0x2D, .AutoTest =
                                                       0x36,
                                               .Version = 0x37 };
// internal alias
const struct _MRFC_REGISTERS *REG = &MRFC_REGISTER;

void confMRFC() {
    // SDA/SS
    P1DIR |= BIT3;
    P1OUT |= BIT3;

    // RST
    P1DIR |= BIT4;
    P1OUT |= BIT4;

    // TODO: test lower brw values
    const spi_config masterSPIConfig = { .polarity = 0, .phase = 1, .MSB_first =
                                                 1,
                                         .brw = 104 };

    configSPI(&masterSPIConfig);

    MRFCSetRegister(REG->Command, CMD->SoftReset);

    MRFCSetRegister(REG->TMode, 0x8D); //Tauto=1; f(Timer) = 6.78MHz/TPreScaler
    MRFCSetRegister(REG->TPrescaler, 0x3E);  //TModeReg[3..0] + TPrescalerReg
    MRFCSetRegister(REG->TReloadLB, 30);
    MRFCSetRegister(REG->TReloadHB, 0);

    MRFCSetRegister(REG->TxASK, 0x40);    //100%ASK
    MRFCSetRegister(REG->Mode, 0x3D);

    MRFCSetRegBits(REG->TxControl, 0x03);
}

void __setSS() {
    P1OUT &= ~BIT3;
}

void __unsetSS() {
    P1OUT |= BIT3;
}

uint8_t MRFCGetRegister(uint8_t reg) {
    __setSS();

    // transfer register address, setting read mode, response is DONT CARE
    spiTransfer(0b10000000 | (reg << 1) & 0b11111110);

    // transfer 0 to get response from previous transfer
    uint8_t regVal = spiTransfer(0);

    __unsetSS();

    return regVal;
}

void MRFCSetRegister(uint8_t reg, uint8_t value) {
    __setSS();

    // transfer register address, setting write mode, response is DONT CARE
    spiTransfer((reg << 1) & 0b01111110);

    // transfer value to be written to register;
    spiTransfer(value);

    __unsetSS();
}

void __prepareTransmission(uint8_t interruptMask) {
    MRFCSetRegister(REG->ComIEn, interruptMask | 0x80);
    MRFCUnsetRegBits(REG->ComIrq, 0x80);
    MRFCSetRegBits(REG->FIFOLevel, 0x80);
    MRFCSetRegister(REG->Command, CMD->Idle);
}

uint8_t MRFCDetectPICC() {
    uint8_t enabledInterrupts = RX_INTERRUPT | TIMER_INTERRUPT;

    __prepareTransmission(enabledInterrupts);

    MRFCSetRegister(REG->FIFOData, PCMD->ReqA);
    MRFCSetRegister(REG->Command, CMD->Transceive);
    MRFCSetRegBits(REG->BitFraming, 0x87);

    uint8_t interruptReg = 0;
    while (!(interruptReg & enabledInterrupts)) {
        interruptReg = MRFCGetRegister(REG->ComIrq);
    }

    MRFCUnsetRegBits(REG->BitFraming, 0x80);

    return interruptReg & RX_INTERRUPT;
}

UID MRFCReadPICC() {
    ANTICOLLISION_CMD cmd = { .SEL = PCMD->AnticollisionCL1, .NVB = 0x20,
                              .UID_CLn = 0 };
    uint8_t enabledInterrupts = RX_INTERRUPT | TIMER_INTERRUPT;

    MRFCUnsetRegBits(REG->Coll, 0x80);

    __prepareTransmission(enabledInterrupts);

    MRFCSetRegister(REG->FIFOData, cmd.SEL);
    MRFCSetRegister(REG->FIFOData, cmd.NVB);

    MRFCSetRegister(REG->Command, CMD->Transceive);
    MRFCSetRegister(REG->BitFraming, 0x80);

    uint8_t interruptReg = 0;
    while (!(interruptReg & enabledInterrupts)) {
        interruptReg = MRFCGetRegister(REG->ComIrq);
    }

    MRFCUnsetRegBits(REG->BitFraming, 0x80);

    const uint8_t responseSize = MRFCGetRegister(REG->FIFOLevel);
    const uint8_t resposeIsValidUID = responseSize >= MIN_UID_SIZE;
    UID uid = { .received = resposeIsValidUID, .size =
            resposeIsValidUID ? responseSize : 0 };

    if (uid.received) {
        uint8_t i;
        for (i = 0; i < responseSize; i++)
            uid.data[i] = MRFCGetRegister(REG->FIFOData);
    }

    return uid;
}

