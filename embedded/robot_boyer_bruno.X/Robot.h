#ifndef ROBOT_H
#define ROBOT_H

typedef struct robotStateBITS {

    struct {
        unsigned char taskEnCours;

        float vitesseGaucheConsigne;
        float vitesseGaucheCommandeCourante;
        float vitesseDroiteConsigne;
        float vitesseDroiteCommandeCourante;
        float distanceTelemetreExGauche;
        float distanceTelemetreGauche;
        float distanceTelemetreCentre;
        float distanceTelemetreDroit;
        float distanceTelemetreExDroit;
        float targetCorner;
        float targetMagnitude;
    };
} ROBOT_STATE_BITS;

extern volatile ROBOT_STATE_BITS robotState;
extern double talon;




void PWMUpdateSpeed(void);

#endif /* ROBOT_H */