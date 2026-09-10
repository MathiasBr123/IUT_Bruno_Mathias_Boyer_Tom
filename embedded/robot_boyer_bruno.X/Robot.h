#ifndef ROBOT_H
#define ROBOT_H

typedef struct robotStateBITS {

    struct {
        unsigned char taskEnCours;

        float vitesseGaucheConsigne;
        float vitesseGaucheCommandeCourante;
        float vitesseDroiteConsigne;
        float vitesseDroiteCommandeCourante;
    };
} ROBOT_STATE_BITS;

extern volatile ROBOT_STATE_BITS robotState;

const uint8_t acceleration = 5;

void PWMUpdateSpeed(void);

#endif /* ROBOT_H */