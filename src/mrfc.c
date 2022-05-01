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

const struct _PICC_COMMANDS PCMD = { .ReqA = 0x26, .AnticollisionCL1 = 0x93 };

const struct _MRFC_COMMANDS CMD = { .Idle = 0b0000, .Transceive = 0b1100, .SoftReset = 0b1111 };

const struct _MRFC_REGISTERS REG = { .Command = 0x01, .ComIEn = 0x02, .ComIrq = 0x04, .FIFOData = 0x09, .FIFOLevel = 0x0A, .BitFraming = 0x0D, .Coll = 0x0E, .Mode = 0x11, .TxControl = 0x14, .TxASK = 0x15, .TMode = 0x2A, .TPrescaler = 0x2B, .TReloadHB = 0x2C, .TReloadLB = 0x2D, .Version = 0x37 };

PICC_UID_DB uid_db;

void init_data() {
  uid_db.nextEmptyLine = 0;
  PICC_UID default_uid = {.size = 5, .data = { 0xEC, 0xC3, 0xAF, 0x16, 0x96 }};
  uid_db.adminPICC_UID = default_uid;
}

void confMRFC() {
    // SDA/SS
    P1DIR |= BIT3;
    P1OUT |= BIT3;

    // RST
    P1DIR |= BIT4;
    P1OUT |= BIT4;

    const spi_config masterSPIConfig = { .polarity = 0, .phase = 1, .MSB_first =
                                                 1,
                                         .brw = 2 };

    configSPI(&masterSPIConfig);

    init_data();

    MRFCSetRegister(REG.Command, CMD.SoftReset);

    MRFCSetRegister(REG.TMode, 0x8D);
    MRFCSetRegister(REG.TPrescaler, 0x3E);
    MRFCSetRegister(REG.TReloadLB, 30);
    MRFCSetRegister(REG.TReloadHB, 0);

    MRFCSetRegister(REG.TxASK, 0x40);
    MRFCSetRegister(REG.Mode, 0x3D);

    MRFCSetRegBits(REG.TxControl, 0x03);
}

// set slave select
void __setSS() {
    P1OUT &= ~BIT3;
}

// unset slave select
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

// sets up interruptions, clears FIFO buffer and puts module at idle state
void __prepareTransmission(uint8_t interruptMask) {
    MRFCSetRegister(REG.ComIEn, interruptMask | 0x80);
    MRFCUnsetRegBits(REG.ComIrq, 0x80);
    MRFCSetRegBits(REG.FIFOLevel, 0x80);
    MRFCSetRegister(REG.Command, CMD.Idle);
}

uint8_t MRFCDetectPICC() {
    const uint8_t enabledInterrupts = RX_INTERRUPT | TIMER_INTERRUPT;

    __prepareTransmission((uint8_t) enabledInterrupts);

    MRFCSetRegister(REG.FIFOData, PCMD.ReqA);
    MRFCSetRegister(REG.Command, CMD.Transceive);
    MRFCSetRegBits(REG.BitFraming, 0x87);

    uint8_t interruptReg = 0;
    while (!(interruptReg & enabledInterrupts)) {
        interruptReg = MRFCGetRegister(REG.ComIrq);
    }

    MRFCUnsetRegBits(REG.BitFraming, 0x80);

    return interruptReg & RX_INTERRUPT;
}

PICC_UID MRFCReadPICC() {
    ANTICOLLISION_CMD cmd = { .SEL = PCMD.AnticollisionCL1, .NVB = 0x20,
                              .PICC_UID_CLn = 0 };
    const uint8_t enabledInterrupts = RX_INTERRUPT | TIMER_INTERRUPT;

    MRFCUnsetRegBits(REG.Coll, 0x80);

    __prepareTransmission((uint8_t) enabledInterrupts);

    MRFCSetRegister(REG.FIFOData, cmd.SEL);
    MRFCSetRegister(REG.FIFOData, cmd.NVB);

    MRFCSetRegister(REG.Command, CMD.Transceive);
    MRFCSetRegister(REG.BitFraming, 0x80);

    uint8_t interruptReg = 0;
    while (!(interruptReg & enabledInterrupts)) {
        interruptReg = MRFCGetRegister(REG.ComIrq);
    }

    MRFCUnsetRegBits(REG.BitFraming, 0x80);

    const uint8_t responseSize = MRFCGetRegister(REG.FIFOLevel);
    const uint8_t resposeIsValidPICC_UID = responseSize >= MIN_PICC_UID_SIZE;
    PICC_UID uid = { .received = resposeIsValidPICC_UID, .size =
            resposeIsValidPICC_UID ? responseSize : 0 };

    if (uid.received) {
        uint8_t i;
        for (i = 0; i < uid.size; i++)
            uid.data[i] = MRFCGetRegister(REG.FIFOData);
    }

    return uid;
}

uint8_t MRFCCheckRegisteredPICC_UID(PICC_UID * uid) {
  if(MRFCCheckAdminPICC_UID(uid))
    return 1;

  uint8_t i;
  for(i = 0; i < uid_db.nextEmptyLine; i++) {
    if(__comparePICC_UID(&uid_db.data[i], uid));
      return 1;
  }

  return 0;
}

uint8_t MRFCCheckAdminPICC_UID(PICC_UID * uid) {
  return __comparePICC_UID(&uid_db.adminPICC_UID, uid);
}

uint8_t __comparePICC_UID(PICC_UID * uid_a, PICC_UID * uid_b) {
  if(uid_a->size != uid_b->size)
    return 0;

  uint8_t i;
  for(i = 0; i < uid_a->size; i++) {
    if(uid_a->data[i] != uid_b->data[i])
      return 0;
  }

  return 1;
}

uint8_t MRFCRegisterPICC_UID(PICC_UID uid) {
  if(uid_db.nextEmptyLine == -1)
    return 0;

  uid_db.data[uid_db.nextEmptyLine++] = uid;

  if(uid_db.nextEmptyLine == DB_SIZE)
    uid_db.nextEmptyLine = -1;

  return 1;
}

