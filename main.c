#include <msp430.h> 
#include <stdint.h>
#include "src/transient_buffer.h"

// Ports used: 1.3, 1.4, 3.0-2
#include "src/mrfc.h"

#define TRUE 1
#define FALSE 0

#define LED_RED_ON        (P1OUT |= BIT0)
#define LED_RED_OFF       (P1OUT &= ~BIT0)
#define LED_RED_TOGGLE    (P1OUT ^= BIT0)

#define LED_GREEN_ON      (P4OUT |= BIT7)
#define LED_GREEN_OFF     (P4OUT &= ~BIT7)
#define LED_GREEN_TOGGLE  (P4OUT ^= BIT7)

void confLeds();

#define S1_PRESSED ((P2IN & BIT1) == 0)

void confS1();

// ========================================
// |                Alarm                 |
// ========================================

void confBuzzer();
void initAlarmTimer();
void deactivateAlarmTimer();
void soundAlarm();

// ========================================
// |            Motion Sensor             |
// ========================================

#define MOTION_DETECTED (P2IN & BIT2)

void confMotionSensor();
void confMotionSensorPrepareTimer();

typedef uint8_t bool;

uint8_t halfSecondsCounter = 0, secondsPastDetection = 0,
        shouldDeactivateAlarm = 0, shouldSoundAlarm = 0;

TransientBuffer rxBuff;

PICC_UID lastReadUID;
PICC_UID samplePICC_UID();

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

    initBuffer(&rxBuff);

    confBuzzer();
    confS1();
    confLeds();
    confMotionSensor();

    confMRFC();

    __enable_interrupt();

    /* while (1) { */
    /*     if (MRFCDetectPICC()) { */
    /*         lastReadUID = MRFCReadPICC(); */
    /*         volatile uint8_t x; */
    /*         if(lastReadUID.received && MRFCCheckAdminPICC_UID(&lastReadUID)) */
    /*             x = lastReadUID.size; */
    /*     } */
    /* } */

    while (1) {
        if (MOTION_DETECTED) {
            while (MOTION_DETECTED) {
                LED_RED_ON;
                __delay_cycles(100000);
                LED_RED_OFF;
                __delay_cycles(100000);
            }
            confMotionSensorPrepareTimer();
        }
        if (shouldSoundAlarm) {
            soundAlarm();
            shouldSoundAlarm = 0;
        }
    };

    return 0;
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

// ========================================
// |                Alarm                 |
// ========================================

void confBuzzer() {
    P2DIR |= BIT0;
    P2SEL |= BIT0;
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
    if (secondsPastDetection++ == 1) {
        secondsPastDetection = 0;
        deactivateAlarmTimer();
        shouldSoundAlarm = 1;
    }
}

void soundAlarm() {
    TA1CTL = TASSEL__ACLK | ID__1 | MC__UP | TACLR;
    const uint16_t tone = 320;
    const uint16_t highRef = 32768 / tone;
    const uint16_t lowRef = highRef / 2;
    TA1CCR0 = highRef;
    TA1CCR1 = lowRef;
    TA1CCTL1 = OUTMOD_7;
    const int16_t step = 1;
    int16_t modifier = -step;

    while (!shouldDeactivateAlarm) {
        __delay_cycles(2000);
        if ((modifier > 0 && TA1CCR0 >= highRef - step)
                || (modifier < 0 && TA1CCR0 <= lowRef + step)) {
            modifier *= -1;
        }
        TA1CCR0 += modifier;
        TA1CCR1 += modifier;
    }
    TA1CTL = TACLR;
    shouldDeactivateAlarm = 0;
    LED_GREEN_ON;
}

void confS1() {
    P2SEL &= ~BIT1;
    P2DIR &= ~BIT1;
    P2REN |= BIT1;
    P2OUT |= BIT1;

    P2IE |= BIT1;

    do {
        P2IFG = 0;
    } while (P2IFG != 0);
}

PICC_UID uidOversampling[100];
#pragma vector = PORT2_VECTOR;
__interrupt void s1_isr(void) {
    switch (P2IV) {
    case P2IV_P2IFG1:
        if(shouldSoundAlarm) {
            if(!shouldDeactivateAlarm) {
                shouldDeactivateAlarm = 1;
                deactivateAlarmTimer();
            }
            break;
        } else {
            while(!MRFCDetectPICC());

            PICC_UID uid = { .received = 0 };

            while(!uid.received)
                uid = MRFCReadPICC();

            if(MRFCCheckAdminPICC_UID(&uid)) {
              LED_RED_ON;
              while(MRFCDetectPICC());
              LED_RED_OFF;
              while(!MRFCDetectPICC());
              uid = samplePICC_UID();
              MRFCRegisterPICC_UID(uid);
            }

        }
        break;
    case P2IV_P2IFG2:
        LED_GREEN_OFF;
        initAlarmTimer();
        break;
    default:
        break;
    }
}

PICC_UID samplePICC_UID() {
  const uint8_t sampleCount = 100;
  uint8_t byteFrequencies[MAX_PICC_UID_SIZE][256] = {{ 0 }};
  uint8_t sizeFrequencies[7] = { 0 };
  uint8_t j, i;

  for(i = 0; i < sampleCount; i++) {
    LED_GREEN_OFF;
    LED_RED_OFF;
    while(MRFCDetectPICC());
    LED_GREEN_ON;
    uidOversampling[i].received = 0;
    while(!uidOversampling[i].received) {
      uidOversampling[i] = MRFCReadPICC();
    }
    LED_RED_ON;

    sizeFrequencies[uidOversampling[i].size]++;

    for(j = 0; j < uidOversampling[i].size; j++)
      byteFrequencies[j][uidOversampling->data[j]]++;
  }

  uint8_t mostFrequentUIDSizeFreq = 0, mostFrequentUIDSize = 0;

  for(i = 0; i < 7; i++) {
    if(sizeFrequencies[i] > mostFrequentUIDSizeFreq)
      mostFrequentUIDSize = i + 4;
  }

  PICC_UID mostFrequentUID = { .received = 1, .size = mostFrequentUIDSize };
  for(j = 0; j < mostFrequentUID.size; j++) {
    uint8_t m;
    uint8_t mostFrequentByteFreq = 0, mostFrequentByte = 0;
    for(m = 0; m < 256; m++) {
      if(byteFrequencies[j][m] > mostFrequentByteFreq)
        mostFrequentByteFreq = byteFrequencies[j][m];
        mostFrequentByte = m;
    }
    mostFrequentUID.data[j] = mostFrequentByte;
  }

  return mostFrequentUID;
}

// ========================================
// |            Motion Sensor             |
// ========================================

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

void confMotionSensorPrepareTimer() {
    TB0CTL = TBSSEL__ACLK | ID__1 | MC__UP | TBCLR;
    TB0CCTL0 = CCIE;
    TB0CCR0 = 32768 >> 1;
}

// turns led green on when motion sensor is ready again
#pragma vector = TIMER0_B0_VECTOR
__interrupt void motion_sensor_readiness() {
    if (halfSecondsCounter++ == 8) {
        TB0CTL = TBCLR;
        LED_GREEN_ON;
        halfSecondsCounter = 0;
    }
}
