#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "main.h"
#include "ChipConfig.h"
#include "IO.h"
#include "timer.h"
#include "PWM.h"
#include "ADC.h"
#include "Robot.h"

unsigned char stateRobot;
unsigned int val;
#define MODE 1

int main(void) {

    InitOscillator();
    InitIO();

    InitTimer23();
    InitTimer1();
    InitTimer4();
    InitPWM();
    InitADC1();

    //Le PWM est set a 20%
    //Pour 70% TP_M1_PMWH : 14.30 us +Width
    //Pour 70% TP_M1_PMWL : 413.2 ns +Width
    PWMSetSpeedConsigne(0, MOTOR_GAUCHE);
    PWMSetSpeedConsigne(0, MOTOR_DROIT);

    LED_BLANCHE_1 = 0;
    LED_BLEUE_1 = 0;
    LED_ORANGE_1 = 0;
    LED_ROUGE_1 = 0;
    LED_VERTE_1 = 0;

    LED_BLANCHE_2 = 1;
    LED_BLEUE_2 = 1;
    LED_ORANGE_2 = 1;
    LED_ROUGE_2 = 1;
    LED_VERTE_2 = 1;

    while (1) {
        if (ADCConversionFinishedFlag) {
            ADCClearConversionFinishedFlag();

            unsigned int * result = ADCGetResult();

            float volts = ((float) result [0])* 3.3 / 4096;
            robotState.distanceTelemetreExGauche = 34 / volts - 5;
            volts = ((float) result [1])* 3.3 / 4096;
            robotState.distanceTelemetreGauche = 34 / volts - 5;
            volts = ((float) result [2])* 3.3 / 4096;
            robotState.distanceTelemetreCentre = 34 / volts - 5;
            volts = ((float) result [3])* 3.3 / 4096;
            robotState.distanceTelemetreDroit = 34 / volts - 5;
            volts = ((float) result [4])* 3.3 / 4096;
            robotState.distanceTelemetreExDroit = 34 / volts - 5;
            
            if (robotState.distanceTelemetreExDroit <= 10)
                robotState.distanceTelemetreExDroit = 10;
            
            if (robotState.distanceTelemetreDroit <= 10)
                robotState.distanceTelemetreDroit = 10;
            
            if (robotState.distanceTelemetreCentre <= 10)
                robotState.distanceTelemetreCentre = 10;
            
            if (robotState.distanceTelemetreGauche <= 10)
                robotState.distanceTelemetreGauche = 10;
            
            if (robotState.distanceTelemetreExGauche <= 10)
                robotState.distanceTelemetreExGauche = 10;
            
            
            if (robotState.distanceTelemetreExDroit >= 80)
                robotState.distanceTelemetreExDroit = 80;
            
            if (robotState.distanceTelemetreDroit >= 80)
                robotState.distanceTelemetreDroit = 80;
            
            if (robotState.distanceTelemetreCentre >= 80)
                robotState.distanceTelemetreCentre = 80;
            
            if (robotState.distanceTelemetreGauche >= 80)
                robotState.distanceTelemetreGauche = 80;
            
            if (robotState.distanceTelemetreExGauche >= 80)
                robotState.distanceTelemetreExGauche = 80;
            
            val = result[2];

            LED_BLANCHE_1 = robotState.distanceTelemetreExGauche <= 20;
            LED_BLEUE_1 = robotState.distanceTelemetreGauche <= 20;
            LED_ORANGE_1 = robotState.distanceTelemetreCentre <= 30;
            LED_ROUGE_1 = robotState.distanceTelemetreDroit <= 20;
            LED_VERTE_1 = robotState.distanceTelemetreExDroit <= 20;

            if (MODE) {
                SetNextRobotStateInAutomaticMode();
                OperatingSystemLoop();
            }
        }
    }
}

void OperatingSystemLoop(void) {
    switch (stateRobot) {
        case STATE_ATTENTE:
            timestamp = 0;
            PWMSetSpeedConsigne(0, MOTOR_DROIT);
            PWMSetSpeedConsigne(0, MOTOR_GAUCHE);
            stateRobot = STATE_ATTENTE_EN_COURS;
        case STATE_ATTENTE_EN_COURS:
            if (timestamp > 1000)
                stateRobot = STATE_AVANCE;
            break;
        case STATE_AVANCE:
            PWMSetSpeedConsigne(25, MOTOR_DROIT);
            PWMSetSpeedConsigne(25, MOTOR_GAUCHE);
            stateRobot = STATE_AVANCE_EN_COURS;
            break;
        case STATE_AVANCE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;
        case STATE_TOURNE_GAUCHE:
            PWMSetSpeedConsigne(25, MOTOR_DROIT);
            PWMSetSpeedConsigne(0, MOTOR_GAUCHE);
            stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
            break;
        case STATE_TOURNE_GAUCHE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;
        case STATE_TOURNE_DROITE:
            PWMSetSpeedConsigne(0, MOTOR_DROIT);
            PWMSetSpeedConsigne(25, MOTOR_GAUCHE);
            stateRobot = STATE_TOURNE_DROITE_EN_COURS;
            break;
        case STATE_TOURNE_DROITE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;
        case STATE_TOURNE_SUR_PLACE_GAUCHE:
            PWMSetSpeedConsigne(10, MOTOR_DROIT);
            PWMSetSpeedConsigne(-10, MOTOR_GAUCHE);
            stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS;
            break;
        case STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;
        case STATE_TOURNE_SUR_PLACE_DROITE:
            PWMSetSpeedConsigne(-10, MOTOR_DROIT);
            PWMSetSpeedConsigne(10, MOTOR_GAUCHE);
            stateRobot = STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS;
            break;
        case STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;
        default:
            stateRobot = STATE_ATTENTE;
            break;
    }
}
unsigned char nextStateRobot = 0;

void SetNextRobotStateInAutomaticMode() {
    unsigned char positionObstacle = PAS_D_OBSTACLE;
    //ÈDtermination de la position des obstacles en fonction des ÈÈËtlmtres
    if (robotState.distanceTelemetreExDroit <= 30 &&
            robotState.distanceTelemetreExDroit > 15 &&
            robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreCentre > 30 &&
            robotState.distanceTelemetreGauche > 30 &&
            robotState.distanceTelemetreExGauche > 30) //Obstacle ‡ droite
        positionObstacle = OBSTACLE_A_DROITE;

    else if (robotState.distanceTelemetreExDroit > 30 &&
            robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreCentre > 30 &&
            robotState.distanceTelemetreGauche > 30 &&
            robotState.distanceTelemetreExGauche <= 30 &&
            robotState.distanceTelemetreExGauche > 15) //Obstacle ‡ gauche
        positionObstacle = OBSTACLE_A_GAUCHE;

    else if (robotState.distanceTelemetreExDroit > 30 &&
            robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreCentre <= 30 &&
            robotState.distanceTelemetreGauche <= 30 &&
            robotState.distanceTelemetreGauche > 20 &&
            robotState.distanceTelemetreExGauche <= 30 &&
            robotState.distanceTelemetreExGauche > 15) //Obstacle en face et gauche
        positionObstacle = OBSTACLE_EN_FACE_GAUCHE;

    else if (robotState.distanceTelemetreExDroit <= 30 &&
            robotState.distanceTelemetreExDroit > 15 &&
            robotState.distanceTelemetreDroit <= 30 &&
            robotState.distanceTelemetreDroit > 20 &&
            robotState.distanceTelemetreCentre <= 30 &&
            robotState.distanceTelemetreGauche > 30 &&
            robotState.distanceTelemetreExGauche > 30) //Obstacle en face et gauche
        positionObstacle = OBSTACLE_EN_FACE_DROITE;

    else if (robotState.distanceTelemetreCentre <= 30) //Obstacle en face
        positionObstacle = OBSTACLE_EN_FACE;

    else if (robotState.distanceTelemetreExDroit > 15 &&
            robotState.distanceTelemetreDroit > 20 &&
            robotState.distanceTelemetreCentre > 30 &&
            robotState.distanceTelemetreGauche > 20 &&
            robotState.distanceTelemetreExGauche > 15) //pas d?obstacle
        positionObstacle = PAS_D_OBSTACLE;
    
    
    else if (robotState.distanceTelemetreExDroit <= 15 )
        positionObstacle = OBSTACLE_A_ExDROITE_PROCHE;
    
    else if (robotState.distanceTelemetreDroit <= 20 )
        positionObstacle = OBSTACLE_A_DROITE_PROCHE;
    
    else if (robotState.distanceTelemetreGauche <= 20 )
        positionObstacle = OBSTACLE_A_GAUCHE_PROCHE;
    
    else if (robotState.distanceTelemetreExGauche <= 15 )
        positionObstacle = OBSTACLE_A_ExGAUCHE_PROCHE;

    //ÈDtermination de lÈ?tat ‡venir du robot
    if (positionObstacle == PAS_D_OBSTACLE)
        nextStateRobot = STATE_AVANCE;

    else if (positionObstacle == OBSTACLE_A_DROITE)
        nextStateRobot = STATE_TOURNE_GAUCHE;

    else if (positionObstacle == OBSTACLE_A_GAUCHE)
        nextStateRobot = STATE_TOURNE_DROITE;

    else if (positionObstacle == OBSTACLE_EN_FACE_GAUCHE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;

    else if (positionObstacle == OBSTACLE_EN_FACE_DROITE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;

    else if (positionObstacle == OBSTACLE_EN_FACE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
    
    else if (positionObstacle == OBSTACLE_A_ExDROITE_PROCHE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
    
    else if (positionObstacle == OBSTACLE_A_DROITE_PROCHE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
    
    else if (positionObstacle == OBSTACLE_A_GAUCHE_PROCHE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
    
    else if (positionObstacle == OBSTACLE_A_ExGAUCHE_PROCHE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;

    //Si l?on n?est pas dans la transition de lÈ?tape en cours
    if (nextStateRobot != stateRobot - 1)
        stateRobot = nextStateRobot;
}
