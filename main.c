#include <msp430.h> 
#include <stdint.h>

#define TRUE 1
#define FALSE 0

#define LED_RED_ON        (P1OUT |= BIT0)
#define LED_RED_OFF       (P1OUT &= ~BIT0)
#define LED_RED_TOGGLE    (P1OUT ^= BIT0)

#define LED_GREEN_ON      (P4OUT |= BIT7)
#define LED_GREEN_OFF     (P4OUT &= ~BIT7)
#define LED_GREEN_TOGGLE  (P4OUT ^= BIT7)

#define MOTION_DETECTED (P2IN & BIT2)

#define S1_PRESSED ((P2IN & BIT1) == 0)

void confS1();
void confLeds();
void confBuzzer();
void confMotionSensor();
void confTimer();
void initAlarmTimer();
void deactivateAlarmTimer();
void soundAlarm();

#define BUFFER_SIZE 8

typedef struct {
    unsigned long idx;
    uint8_t data[BUFFER_SIZE];
} TransientBuffer;

TransientBuffer rxBuff, txBuff;

typedef uint8_t bool;

typedef struct {
    bool enabled;
    bool reading;
} sensor;

sensor motionSensor;

uint8_t counter = 0, secondsPastDetection = 0, isr_flag_ch1 = 0,
        shouldSoundAlarm = 0;

// SPI stuff
typedef struct {
    bool set_polarity;
    uint8_t phase;
    bool MSB_first;
} spi_config;

const spi_config masterSPIConfig = { .set_polarity = TRUE, .phase = 0,
                                     .MSB_first = TRUE };

const struct {
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
} MRFC_COMMAND = { .Idle = 0b0000, .Mem = 0b0001, .GenerateRandomID = 0b0010,
                   .CalcCRC = 0b0011, .Transmit = 0b0100, .NoCmdChange = 0b0111,
                   .Receive = 0b1000, .Transceive = 0b1100, .MFAuthent = 0b1110,
                   .SoftReset = 0b1111 };

/*const struct {

} MRFC_REGS = {

};*/

void configSPI(spi_config *c);
uint8_t spiTransfer(uint8_t byte);

void initSensor(sensor *s, bool enabled);

uint8_t first = 0, second = 0;

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

    initBuffer(&txBuff);
    initBuffer(&rxBuff);

    confBuzzer();
    confS1();
    confLeds();
    confMotionSensor();
    initSensor(&motionSensor, TRUE);
    confTimer();

    __enable_interrupt();

    configSPI(&masterSPIConfig);

    uint16_t i = 0;
    uint8_t data = 0x37;
    volatile uint8_t j = 0;

    P1DIR |= BIT3;
    P1SEL |= BIT3;
    P1OUT |= BIT3;

    while (1) {
        for(i = 0; i < BUFFER_SIZE; i++) {
            P1OUT ^= BIT3;
            spiTransfer(((data << 1) & 0x7E) | 0x80);
            pushToBuffer(&txBuff, UCB0TXBUF);
            pushToBuffer(&rxBuff, UCB0RXBUF);
            spiTransfer(0);
            pushToBuffer(&txBuff, UCB0TXBUF);
            pushToBuffer(&rxBuff, UCB0RXBUF);
            P1OUT ^= BIT3;
        }
         j++;
    }

    while (1) {
        if (MOTION_DETECTED) {
            while (MOTION_DETECTED) {
                LED_RED_ON;
                __delay_cycles(100000);
                LED_RED_OFF;
                __delay_cycles(100000);
            }
            confTimer();
        }
        if (shouldSoundAlarm) {
            soundAlarm();
            shouldSoundAlarm = 0;
        }
    };

    return 0;
}

void confBuzzer() {
    P2DIR |= BIT0;
    P2SEL |= BIT0;
}

void confS1() {
    P2SEL &= ~BIT1;
    P2DIR &= ~BIT1;
    P2REN |= BIT1;
    P2OUT |= BIT1;

    P2IE |= BIT1;
    P2IES |= BIT1;

    do {
        P2IFG = 0;
    } while (P2IFG != 0);
}

void confTimer() {
    TB0CTL = TBSSEL__ACLK | ID__1 | MC__UP | TBCLR;
    TB0CCTL0 = CCIE;
    TB0CCR0 = 32768 >> 1;
}

void initAlarmTimer() {
    TA2CTL = TASSEL__ACLK | ID__1 | MC__UP | TACLR;
    TA2CCTL0 = CCIE;
    TA2CCR0 = 32767;
}

void deactivateAlarmTimer() {
    TA2CTL = TACLR;
}

#pragma vector = TIMER2_A0_VECTOR
__interrupt void timer_a2_isr() {
    if (secondsPastDetection++ == 5) {
        secondsPastDetection = 0;
        deactivateAlarmTimer();
        shouldSoundAlarm = 1;
    }
}

void soundAlarm() {
    TA1CTL = TASSEL__ACLK | ID__1 | MC__UP | TACLR;
    const uint16_t tone = 180;
    const uint16_t highRef = 32768 / tone;
    const uint16_t lowRef = highRef / 2;
    TA1CCR0 = highRef;
    TA1CCR1 = lowRef;
    TA1CCTL1 = OUTMOD_7;
    const int16_t step = 2;
    int16_t modifier = -step;

    while (!isr_flag_ch1) {
        LED_GREEN_TOGGLE;
        __delay_cycles(50000);
        if ((modifier > 0 && TA1CCR0 >= highRef - step)
                || (modifier < 0 && TA1CCR0 <= lowRef + step)) {
            modifier *= -1;
        }
        TA1CCR0 += modifier;
        TA1CCR1 += modifier;
    }
    TA1CTL = TACLR;
    isr_flag_ch1 = 0;
    LED_GREEN_ON;
}

#pragma vector = TIMER0_B0_VECTOR
__interrupt void timer_b0_isr() {
    if (counter++ == 8) {
        TB0CTL = TBCLR;
        LED_GREEN_ON;
        counter = 0;
    }
}

void initSensor(sensor *s, bool enabled) {
    s->enabled = enabled;
    s->reading = FALSE;
}

void confLeds() {
    // LED VERMELHO
    P1SEL &= ~BIT0;
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;

    // LED VERDE
    P4SEL &= ~BIT7;
    P4DIR |= BIT7;
    P4OUT &= ~BIT7;
}

void confMotionSensor() {
    P2SEL &= ~BIT2;
    P2DIR &= ~BIT2;
    P2REN |= BIT2;
    P2OUT |= BIT2;

    P2IE |= BIT2;
    P2IES |= BIT2;

    do {
        P2IFG = 0;
    } while (P2IFG != 0);
}

#pragma vector = PORT2_VECTOR;
__interrupt void s1_isr(void) {
    switch (P2IV) {
    case P2IV_P2IFG1:
        isr_flag_ch1 = 1;
        deactivateAlarmTimer();
        break;
    case P2IV_P2IFG2:
        motionSensor.reading = TRUE;
        LED_GREEN_OFF;
        initAlarmTimer();
        break;
    default:
        break;
    }
}

// P3.0-P3.2
void configSPI(spi_config *c) {
    // desliga o modulo
    UCB0CTL1 = UCSWRST;

    UCB0CTL0 = UCMODE_0 | UCSYNC;

    UCB0CTL1 |= UCSSEL__SMCLK;
    UCB0BRW = 6000;

    UCB0CTL0 |= UCMST;

    if (c->set_polarity)
    UCB0CTL0 |= UCCKPL;
    if (c->phase)
    UCB0CTL0 |= UCCKPH;
    if (c->MSB_first)
    UCB0CTL0 |= UCMSB;

    P3SEL |= BIT0 | BIT1;

    UCB0CTL1 &= ~UCSWRST;
    generateSPIClock();

    //UCB0IE = UCTXIE;// | UCRXIE;
}

void generateSPIClock() {
    P1DIR |= BIT2;
    P1SEL |= BIT2;

    TA0CTL = TASSEL__SMCLK | ID__1 | MC__UP | TACLR;
    TA0CCR0 = 3000;
    TA0CCR1 = TA0CCR0 / 2;
    TA0CCTL1 = OUTMOD_7;
}

uint8_t spiTransfer(uint8_t byte) {
    while ((UCB0IFG & UCTXIFG) == 0);
    UCB0TXBUF = byte;

    while ((UCB0IFG & UCRXIFG) == 0);
    return UCB0RXBUF;
}

void initBuffer(TransientBuffer *buf) {
    buf->idx = 0;

    unsigned long i = 0;
    for (i = 0; i < BUFFER_SIZE; i++)
        buf->data[i] = 0;
}

void pushToBuffer(TransientBuffer *buf, uint8_t val) {
    unsigned long currentIndex = buf->idx;
    buf->data[currentIndex] = val;
    buf->idx = (currentIndex + 1) % BUFFER_SIZE;
}
