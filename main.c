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
void confMotionSensor();
void confTimer();
void initAlarmTimer();
void deactivateAlarmTimer();
void soundAlarm();

typedef uint8_t bool;

typedef struct {
    bool enabled;
    bool reading;
} sensor;

sensor motionSensor;

uint8_t counter = 0, secondsPastDetection = 0, isr_flag_ch1 = 0;

void initSensor(sensor *s, bool enabled);

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

    confS1();
    confLeds();
    confMotionSensor();
    initSensor(&motionSensor, TRUE);
    confTimer();

    __enable_interrupt();

    soundAlarm();

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
    };

    return 0;
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
        soundAlarm();
    }
}

void soundAlarm() {
    P2DIR |= BIT0;
    P2SEL |= BIT0;
    TA1CTL = TASSEL__ACLK | ID__1 | MC__UP | TACLR;
    const uint16_t highRef = 32768 / 264;
    const uint16_t lowRef = highRef / 2;
    TA1CCR0 = highRef;
    TA1CCR1 = lowRef;
    TA1CCTL1 = OUTMOD_7;
    int16_t step = 2;
    int16_t modifier = -step;

    while (!isr_flag_ch1) {
        LED_GREEN_TOGGLE;
        __delay_cycles(50000);
        if (TA1CCR0 == highRef - step || TA1CCR0 == lowRef + step) {
            modifier *= -1;
        }/* else if (TA1CCR0 == lowRef + 1) {
            modifier *= -1;
        }*/
        TA1CCR0 += modifier;
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
