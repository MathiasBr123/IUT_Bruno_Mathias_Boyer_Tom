/* 
 * File:   time.h
 * Author: E306-PC2
 *
 * Created on 7 septembre 2026, 14:17
 */

#ifndef TIME_H
#define	TIME_H

#define FCY 60000000 

extern unsigned long timestamp;

void InitTimer23(void);
void InitTimer1(void);
void InitTimer4(void);
void SetFreqTimer1(float freq); 

#endif	/* TIME_H */

