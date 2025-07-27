#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "main.h"

// Defines
#define SOFTWARE_PWM_PERIOD (5)   // Software PWM period in ticks - the less the faster is the PWM freq

// Enum representing current robot direction state
typedef enum {
    DIR_STOP,
    DIR_FORWARD,
    DIR_BACKWARD,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_ROTATE_LEFT,
    DIR_ROTATE_RIGHT,
    DIR_RIGHT_FORWARD,
    DIR_RIGHT_BACKWARD,
    DIR_LEFT_FORWARD,
    DIR_LEFT_BACKWARD
} Direction_t;

// External declarations of global variables defined in main.c
extern volatile uint8_t pwm_left;
extern volatile uint8_t pwm_right;
extern Direction_t current_direction;

// Function declarations
void SoftwarePWM_Loop(void);
void StopMotors(void);
void SafeDirectionChange(Direction_t new_dir);
void MoveForward(uint8_t speed_percent);
void MoveBackward(uint8_t speed_percent);
void MoveRightWheelForward(uint8_t speed_percent);
void MoveLeftWheelForward(uint8_t speed_percent);
void MoveRightWheelBackward(uint8_t speed_percent);
void MoveLeftWheelBackward(uint8_t speed_percent);

#endif // MOVEMENT_H