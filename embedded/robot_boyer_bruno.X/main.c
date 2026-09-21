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
#include <math.h>
#include "Toolbox.h"

unsigned char stateRobot;
unsigned int val;

// Si MODE = 0 le robot n'est pas piloter les valeur des capteur sont traiter
// mais aucun traitement n 'est angager.
// Si MODE = 1 le robot utilise le mode de detection iteratif (detection pas cas). 
// Si MODE = 2 le robot utilise le mode de detection par champ de potnetiel simlifier
// (Utilisation d'un distence ponderais puis transformer en un vecteur global au 5 capteur).

#define MODE 1

int main(void) {

    InitOscillator();
    InitIO();

    InitTimer23();
    InitTimer1();
    InitTimer4();
    InitPWM();
    InitADC1();

    //Inisialition des moteur a 0 % de le vitesse maximal.

    PWMSetSpeedConsigne(0, MOTOR_GAUCHE);
    PWMSetSpeedConsigne(0, MOTOR_DROIT);

    //Inisialisation de la range de led 1 a 0.
    LED_BLANCHE_1 = 0;
    LED_BLEUE_1 = 0;
    LED_ORANGE_1 = 0;
    LED_ROUGE_1 = 0;
    LED_VERTE_1 = 0;
    //Inisialisation de la r&nge de led 2 a 1.
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
            
            LED_BLANCHE_1 = robotState.distanceTelemetreExGauche <= 10;
            LED_BLEUE_1 = robotState.distanceTelemetreGauche <= 10;
            LED_ORANGE_1 = robotState.distanceTelemetreCentre <= 30;
            LED_ROUGE_1 = robotState.distanceTelemetreDroit <= 10;
            LED_VERTE_1 = robotState.distanceTelemetreExDroit <= 10;

            if (MODE == 1) { // Mode de fonctionnement numero 1 
                SetNextRobotStateInAutomaticMode();
                OperatingSystemLoop();
            }else if(MODE == 2){ // Mode de fonctionement numero 2
                VectorielMobilityLowObstacle();
                OperatingMotor();
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
            
        case STATE_AVANCE:
            PWMSetSpeedConsigne(25, MOTOR_DROIT);
            PWMSetSpeedConsigne(25, MOTOR_GAUCHE);
            stateRobot = STATE_AVANCE_EN_COURS;
            break;
        case STATE_AVANCE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;
        case STATE_TOURNE_GAUCHE:
            PWMSetSpeedConsigne(20, MOTOR_DROIT);
            PWMSetSpeedConsigne(0, MOTOR_GAUCHE);
            stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
            break;
        case STATE_TOURNE_GAUCHE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;
        case STATE_TOURNE_DROITE:
            PWMSetSpeedConsigne(0, MOTOR_DROIT);
            PWMSetSpeedConsigne(20, MOTOR_GAUCHE);
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
    //Dtermination de la position des obstacles en fonction des t�l�mtres
    if (robotState.distanceTelemetreExDroit <= 20 &&
            robotState.distanceTelemetreExDroit > 10 &&
            robotState.distanceTelemetreDroit > 20 &&
            robotState.distanceTelemetreCentre > 30 &&
            robotState.distanceTelemetreGauche > 20 &&
            robotState.distanceTelemetreExGauche > 20) //Obstacle � droite
        positionObstacle = OBSTACLE_A_DROITE;

    else if (robotState.distanceTelemetreExDroit > 20 &&
            robotState.distanceTelemetreDroit > 20 &&
            robotState.distanceTelemetreCentre > 30 &&
            robotState.distanceTelemetreGauche > 20 &&
            robotState.distanceTelemetreExGauche <= 20 &&
            robotState.distanceTelemetreExGauche > 10) //Obstacle � gauche
        positionObstacle = OBSTACLE_A_GAUCHE;

    else if (robotState.distanceTelemetreExDroit > 20 &&
            robotState.distanceTelemetreDroit > 20 &&
            robotState.distanceTelemetreCentre <= 30 &&
            robotState.distanceTelemetreGauche <= 20 &&
            robotState.distanceTelemetreGauche > 10 &&
            robotState.distanceTelemetreExGauche <= 20 &&
            robotState.distanceTelemetreExGauche > 10) //Obstacle en face et gauche
        positionObstacle = OBSTACLE_EN_FACE_GAUCHE;

    else if (robotState.distanceTelemetreExDroit <= 20 &&
            robotState.distanceTelemetreExDroit > 10 &&
            robotState.distanceTelemetreDroit <= 20 &&
            robotState.distanceTelemetreDroit > 10 &&
            robotState.distanceTelemetreCentre <= 30 &&
            robotState.distanceTelemetreGauche > 20 &&
            robotState.distanceTelemetreExGauche > 20) //Obstacle en face et gauche
        positionObstacle = OBSTACLE_EN_FACE_DROITE;

    else if (robotState.distanceTelemetreCentre <= 30) //Obstacle en face
        positionObstacle = OBSTACLE_EN_FACE;

    else if (robotState.distanceTelemetreExDroit > 10 &&
            robotState.distanceTelemetreDroit > 10 &&
            robotState.distanceTelemetreCentre > 30 &&
            robotState.distanceTelemetreGauche > 10 &&
            robotState.distanceTelemetreExGauche > 10) //pas d'obstacle
        positionObstacle = PAS_D_OBSTACLE;
    
    
    else if (robotState.distanceTelemetreExDroit <= 13 )
        positionObstacle = OBSTACLE_A_ExDROITE_PROCHE;
    
    else if (robotState.distanceTelemetreDroit <= 13 )
        positionObstacle = OBSTACLE_A_DROITE_PROCHE;
    
    else if (robotState.distanceTelemetreGauche <= 13 )
        positionObstacle = OBSTACLE_A_GAUCHE_PROCHE;
    
    else if (robotState.distanceTelemetreExGauche <= 13 )
        positionObstacle = OBSTACLE_A_ExGAUCHE_PROCHE;
    
    //�Dtermination de l�tat �venir du robot
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
        nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;

    else if (positionObstacle == OBSTACLE_A_ExDROITE_PROCHE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
    
    else if (positionObstacle == OBSTACLE_A_DROITE_PROCHE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
    
    else if (positionObstacle == OBSTACLE_A_GAUCHE_PROCHE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
    
    else if (positionObstacle == OBSTACLE_A_ExGAUCHE_PROCHE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;

    //Si l?on n?est pas dans la transition de l�?tape en cours
    if (nextStateRobot != stateRobot - 1)
        stateRobot = nextStateRobot;
}

#define NB_SENSOR 5

float *sensorAdresse[NB_SENSOR] = {&robotState.distanceTelemetreExGauche, &robotState.distanceTelemetreGauche, 
    &robotState.distanceTelemetreCentre, &robotState.distanceTelemetreDroit, &robotState.distanceTelemetreExDroit};

void VectorielMobilityLowObstacle(void){
    //Les 5 capteur infrarouge du robot sont espacer d'un angle de 30° chacun, 
    //on utilise le capteur centralle comme référence (angle : 0°), les capteur de gauche
    //on donc un angle de -30° et -60° et ce de droit on un angle +30° et +60°.
    const float CosRad[NB_SENSOR] = {0.500, 0.8660254038, 1.000, 0.8660254038, 0.500};
    const float SinRad[NB_SENSOR] = {-0.8660254038, -0.500, 0.000, 0.500, 0.8660254038};
 
    float weight_sensor[NB_SENSOR];

        for(uint8_t i = 0; i < NB_SENSOR; i++){
            weight_sensor[i] = 500 / *sensorAdresse[i];
        }

    float unitaryVector[NB_SENSOR][2];
    float Vx = 0;
    float Vy = 0;

    for(uint8_t x = 0; x < NB_SENSOR; x++){
        unitaryVector[x][0] = weight_sensor[x] * CosRad[x];
        Vx = Vx + unitaryVector[x][0];

        unitaryVector[x][1] = weight_sensor[x] * SinRad[x];
        Vy = Vy - unitaryVector[x][1];
    }

    robotState.vectorX = Vx;
    robotState.vectorY = Vy;
    
    robotState.targetAngle = atan2f(Vy, Vx)*(180/PI);
    //robotState.targetMagnitude = sqrtf((Vx*Vx) + (Vy*Vy));

}

void OperatingMotor(void){

    float compensationMG;
    float compensationMD;
    
    const float power = 15;
    
    if(robotState.vectorY > 0){
        compensationMD = 10;
        compensationMG = 0;
    }else if(robotState.vectorY < 0){
        compensationMD = 0;
        compensationMG = 10;       
    }
    
    PWMSetSpeedConsigne(power + compensationMD, MOTOR_DROIT);
    PWMSetSpeedConsigne(power + compensationMG, MOTOR_GAUCHE);
   
    /*
    if(robotState.targetAngle > 7){
        if(robotState.targetAngle > 7 && robotState.targetAngle <= 15){
            pwmPowerLeft = 15;
            pwmPowerRight = 0;
        }else if(robotState.targetAngle > 15){
            pwmPowerLeft = 0.4444 * robotState.targetAngle + 3.3333;
            pwmPowerRight = 0;
        }
        stateRobot = ROTATION_P;
    }else if(robotState.targetAngle < -7){
        if(robotState.targetAngle < -7 && robotState.targetAngle >= -15){
            pwmPowerLeft = 0;
            pwmPowerRight = 15;
        }else if(robotState.targetAngle < -15){
            pwmPowerLeft = 0;
            pwmPowerRight = -0.4444 * robotState.targetAngle + 3.3333;
        }
        stateRobot = ROTATION_P; 
    }else if(robotState.targetAngle >= -7 && robotState.targetAngle <= 7){
        stateRobot = STATE_AVANCE;
    }
    */
    /*
    switch(stateRobot){
        case STATE_ATTENTE:
            PWMSetSpeedConsigne(0, MOTOR_DROIT);
            PWMSetSpeedConsigne(0, MOTOR_GAUCHE);
        break;

        case STATE_AVANCE:
            PWMSetSpeedConsigne(25, MOTOR_DROIT);
            PWMSetSpeedConsigne(25, MOTOR_GAUCHE);
        break;

        case ROTATION_P: 
            PWMSetSpeedConsigne(pwmPowerRight, MOTOR_DROIT);
            PWMSetSpeedConsigne(pwmPowerLeft, MOTOR_GAUCHE);
        break;
    }
     */
}