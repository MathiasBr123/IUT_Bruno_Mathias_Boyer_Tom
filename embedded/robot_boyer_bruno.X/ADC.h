/* 
 * File:   ADC.h
 * Author: E306-PC2
 *
 * Created on 15 septembre 2026, 11:16
 */

#ifndef ADC_H
#define	ADC_H

extern unsigned char ADCConversionFinishedFlag;

void ADC1StartConversionSequence(void);
unsigned int * ADCGetResult(void);
unsigned char ADCIsConversionFinished(void);
void ADCClearConversionFinishedFlag(void);

#endif	/* ADC_H */

