/* 
 * File:   main.c
 * Author: E306-PC2
 *
 * Created on September 2, 2026, 12:03 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "ChipConfig.h"
#include "IO.h"
int main (void){

    InitOscillator();

    InitIO();
    LED_BLANCHE_1 = 1;
    LED_BLEUE_1 = 1;
    LED_ORANGE_1 = 1;
    LED_ROUGE_1 = 1;
    LED_VERTE_1 = 1;

while(1)
    {
    
    } // fin main
}


