#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "ChipConfig.h"
#include "IO.h"
#include "timer.h"
#include "PWM.h"
#include "ADC.h"

int main(void) {

    InitOscillator();
    InitIO();
    InitTimer23();
    InitTimer1();
    InitPWM();
    InitADC1();
    
    //Le PWM est set a 20%
    //Pour 70% TP_M1_PMWH : 14.30 us +Width
    //Pour 70% TP_M1_PMWL : 413.2 ns +Width
   PWMSetSpeedConsigne(0, MOTOR_GAUCHE);
   PWMSetSpeedConsigne(0, MOTOR_DROIT);
    
    LED_BLANCHE_1 = 1;
    LED_BLEUE_1 = 1;
    LED_ORANGE_1 = 1;
    LED_ROUGE_1 = 1;
    LED_VERTE_1 = 1;
    
    LED_BLANCHE_2 = 1;
    LED_BLEUE_2 = 1;
    LED_ORANGE_2 = 1;
    LED_ROUGE_2 = 1;
    LED_VERTE_2 = 1;
    
    while(1){
        if(ADCConversionFinishedFlag){
            ADCClearConversionFinishedFlag();
            unsigned int * result = ADCGetResult();

        }
    }
}

//Observation ossilocope
//LED ORANGE Fréquence: 500 ms Hz
//LED BLANCHE Fréquence: 500 Hz
