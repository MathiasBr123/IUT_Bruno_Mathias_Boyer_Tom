/* 
 * File:   PWM.h
 * Author: E306-PC2
 *
 * Created on 7 septembre 2026, 16:20
 */

#ifndef PWM_H
#define	PWM_H

#define MOTOR_GAUCHE 0
#define MOTOR_DROIT 1

void InitPWM(void);
void PWMSetSpeed(float vitesseEnPourcents, uint8_t motor);

#endif	/* PWM_H */

