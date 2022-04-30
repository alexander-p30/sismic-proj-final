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
                                               .TReloadLB = 0x2D, .AutoTest = 0x36, .Version =
                                                       0x37 };
// internal alias
const struct _MRFC_REGISTERS *REG = &MRFC_REGISTER;

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

    MRFCSetRegister(REG->TxMode, 0);
    MRFCSetRegister(REG->RxMode, 0);

    MRFCSetRegister(REG->ModWidth, 0x26);

    MRFCSetRegister(REG->Command, CMD->SoftReset);
    MRFCSetRegister(REG->TMode, 0x80); //Tauto=1; f(Timer) = 6.78MHz/TPreScaler
    MRFCSetRegister(REG->TPrescaler, 0xA9);  //TModeReg[3..0] + TPrescalerReg
    MRFCSetRegister(REG->TReloadLB, 0xE8);
    MRFCSetRegister(REG->TReloadHB, 0x03);

    MRFCSetRegister(REG->TxASK, 0x40);    //100%ASK
    MRFCSetRegister(REG->Mode, 0x3D);

    // turn antenna on
    volatile unsigned char temp;

    temp = MRFCGetRegister(REG->TxControl);
    if (!(temp & 0x03)) {
        MRFCSetRegister(REG->TxControl, temp | 0x03);  // set bit mask
    }
}

void MRFCTest() {
    MRFCSetRegister(REG->Command, CMD->SoftReset);

    // 2. Clear the internal buffer by writing 25 bytes of 00h
    uint8_t i = 0;
    MRFCSetRegister(REG->FIFOLevel, 0x80);    // flush the FIFO buffer
    for(i = 0; i < 25; i++) {
        MRFCSetRegister(REG->FIFOData, 0); // write 25 bytes of 00h to FIFO
    }
    MRFCSetRegister(REG->Command, CMD->Mem);   // transfer to internal buffer

    // 3. Enable self-test
    MRFCSetRegister(REG->AutoTest, 0x09);

    // 4. Write 00h to FIFO buffer
    MRFCSetRegister(REG->FIFOData, 0x00);

    // 5. Start self-test by issuing the CalcCRC command
    MRFCSetRegister(REG->Command, CMD->CalcCRC);

    // 6. Wait for self-test to complete
    uint8_t n;
    for (i = 0; i < 0xFF; i++) {
      // The datasheet does not specify exact completion condition except
      // that FIFO buffer should contain 64 bytes.
      // While selftest is initiated by CalcCRC command
      // it behaves differently from normal CRC computation,
      // so one can't reliably use DivIrqReg to check for completion.
      // It is reported that some devices does not trigger CRCIRq flag
      // during selftest.
      n = MRFCGetRegister(REG->FIFOLevel);
      if (n >= 64) {
        break;
      }
    }
    MRFCSetRegister(REG->Command, CMD->Idle);    // Stop calculating CRC for new content in the FIFO.

    // 7. Read out resulting 64 bytes from the FIFO buffer.
    volatile uint8_t result[64];
    for(i = 0; i < 64; i++) {
        result[i] = MRFCGetRegister(REG->FIFOData);
    }

    // Auto self-test done
    // Reset AutoTestReg register to be 0 again. Required for normal operation.
    MRFCSetRegister(REG->AutoTest, 0x00);
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


#define PICC_REQA 0x26
#define PICC_WUPA 0x52
int8_t MRFCRequest() {

    MRFCSetRegister(REG->Coll, 0b10000000);

    uint8_t validBits = 7;
    uint8_t backData[100] = { 0 };
    uint8_t backLen = 100;
    uint8_t sendData = PICC_REQA;
    uint8_t command = CMD->Transceive;
    uint8_t waitIRq = 0x30;    // RxIRq and IdleIRq
    uint8_t checkCRC = 0;

    //MRFCSetRegister(REG->ComIEn, 0x77 | 0x80);
    volatile uint8_t ien = MRFCGetRegister(REG->ComIEn);

    MRFCSetRegister(REG->Command, CMD->Idle);      // Stop any active command.
    MRFCSetRegister(REG->ComIrq, 0x7F); // Clear all seven interrupt request bits
    MRFCSetRegister(REG->FIFOLevel, 0x80); // FlushBuffer = 1, FIFO initialization
    MRFCSetRegister(REG->FIFOData, sendData);  // Write sendData to the FIFO
    MRFCSetRegister(REG->BitFraming, validBits);   // Bit adjustments
    MRFCSetRegister(REG->Command, command);       // Execute the command
        MRFCSetRegister(REG->BitFraming, 0x80); // StartSend=1, transmission of data starts

    // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer
    // automatically starts when the PCD stops transmitting.
    //
    // Wait here for the command to complete. The bits specified in the
    // `waitIRq` parameter define what bits constitute a completed command.
    // When they are set in the ComIrqReg register, then the command is
    // considered complete. If the command is not indicated as complete in
    // ~36ms, then consider the command as timed out.
    uint8_t completed = 0;
    uint16_t counter = 0;
    do {
        uint8_t n = MRFCGetRegister(REG->ComIrq); // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
        if (n & waitIRq) { // One of the interrupts that signal success has been set.
            completed = 1;
            break;
        }
        if (n & 0x01) {           // Timer interrupt - nothing received in 25ms
            return -1;
        }
        __delay_cycles(500);
    } while (counter++ <= 2000);

    volatile uint8_t err_reg = MRFCGetRegister(REG->Error);

    // 36ms and nothing happened. Communication with the MFRC522 might be down.
    if (!completed) {
        return -1;
    }

    // Stop now if any errors except collisions were detected.
    uint8_t errorRegValue = MRFCGetRegister(REG->Error); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
    if (errorRegValue & 0x13) {  // BufferOvfl ParityErr ProtocolErr
        return -1;
    }

    uint8_t n = MRFCGetRegister(REG->FIFOLevel);  // Number of bytes in the FIFO
    if (n > backLen) {
        return -2;
    }
    backLen = n;                     // Number of bytes returned
    uint8_t data[100] = { 0 };
    uint8_t i = 0;
    for (i = 0; i < n; i++) {
        data[i] = MRFCGetRegister(REG->FIFOData); // Get received data from FIFO
    }
    validBits = MRFCGetRegister(REG->Control) & 0x07; // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.

    // Tell about collisions
    if (errorRegValue & 0x08) {   // CollErr
        return -3;
    }

    // Perform CRC_A validation if requested.
    if (backData && backLen && checkCRC) {
        // In this case a MIFARE Classic NAK is not OK.
        if (backLen == 1 && validBits == 4) {
            return -4;
        }
        // We need at least the CRC_A value and all 8 bits of the last byte must be received.
        if (backLen < 2 || validBits != 0) {
            return -5;
        }
        // Verify CRC_A - do our own calculation and store the control in controlBuffer.
         /*uint8_t controlBuffer[2];
         MFRC522::StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
         if (status != STATUS_OK) {
         return status;
         }
         if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
         return STATUS_CRC_WRONG;
         }*/
    }

    return 0;
}

