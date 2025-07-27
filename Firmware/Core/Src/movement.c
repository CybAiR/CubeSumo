#include "movement.h"
#include "main.h"

// Global variables - extern declarations
extern volatile uint8_t pwm_left;
extern volatile uint8_t pwm_right;
extern Direction_t current_direction;

// PWM counter incremented every 1ms from SysTick
static uint8_t pwm_counter = 0;

// Software based PWM controller - should be called every 1ms from SysTick_Handler
void SoftwarePWM_Loop(void)
{
    pwm_counter++;
    if (pwm_counter >= SOFTWARE_PWM_PERIOD)
    {
        pwm_counter = 0;
    }

    // Left motor PWM
    if (pwm_counter < pwm_left)
    {
        HAL_GPIO_WritePin(EN_LEFT_GPIO_Port, EN_LEFT_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(EN_LEFT_GPIO_Port, EN_LEFT_Pin, GPIO_PIN_RESET);
    }

    // Right motor PWM
    if (pwm_counter < pwm_right)
    {
        HAL_GPIO_WritePin(EN_RIGHT_GPIO_Port, EN_RIGHT_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(EN_RIGHT_GPIO_Port, EN_RIGHT_Pin, GPIO_PIN_RESET);
    }
}

// Stops both motors by disabling the enable pins
void StopMotors(void)
{
    HAL_GPIO_WritePin(EN_LEFT_GPIO_Port, EN_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_RIGHT_GPIO_Port, EN_RIGHT_Pin, GPIO_PIN_RESET);
    pwm_left = 0;
    pwm_right = 0;
    current_direction = DIR_STOP;
}

// Stops motors briefly if direction is changing to avoid shoot-through
void SafeDirectionChange(Direction_t new_dir)
{
    if (current_direction != DIR_STOP && current_direction != new_dir)
    {
        StopMotors();
        HAL_Delay(100); // Dead time to allow safe switching
    }
}

// Moves robot forward at specified speed (0–100%)
void MoveForward(uint8_t speed_percent)
{
    SafeDirectionChange(DIR_FORWARD);

    HAL_GPIO_WritePin(PHASE_LEFT_GPIO_Port, PHASE_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PHASE_RIGHT_GPIO_Port, PHASE_RIGHT_Pin, GPIO_PIN_RESET);

    uint8_t duty;

    if (speed_percent > 100)
    {
        duty = SOFTWARE_PWM_PERIOD;
    }
    else
    {
        duty = (speed_percent * SOFTWARE_PWM_PERIOD) / 100;
    }

    pwm_left = duty;
    pwm_right = duty;

    current_direction = DIR_FORWARD;
}

// Moves robot backward at specified speed (0–100%)
void MoveBackward(uint8_t speed_percent)
{
    SafeDirectionChange(DIR_BACKWARD);

    HAL_GPIO_WritePin(PHASE_LEFT_GPIO_Port, PHASE_LEFT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PHASE_RIGHT_GPIO_Port, PHASE_RIGHT_Pin, GPIO_PIN_SET);

    uint8_t duty;

    if (speed_percent > 100)
    {
        duty = SOFTWARE_PWM_PERIOD;
    }
    else
    {
        duty = (speed_percent * SOFTWARE_PWM_PERIOD) / 100;
    }

    pwm_left = duty;
    pwm_right = duty;

    current_direction = DIR_BACKWARD;
}

// Moves right wheel forward at specified speed (0–100%)
void MoveRightWheelForward(uint8_t speed_percent)
{
    SafeDirectionChange(DIR_RIGHT_FORWARD);

    HAL_GPIO_WritePin(PHASE_LEFT_GPIO_Port, PHASE_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PHASE_RIGHT_GPIO_Port, PHASE_RIGHT_Pin, GPIO_PIN_RESET);

    uint8_t duty;

    if (speed_percent > 100)
    {
        duty = SOFTWARE_PWM_PERIOD;
    }
    else
    {
        duty = (speed_percent * SOFTWARE_PWM_PERIOD) / 100;
    }

    pwm_left = duty;
    pwm_right = 0;

    current_direction = DIR_RIGHT_FORWARD;
}

// Moves left wheel forward at specified speed (0–100%)
void MoveLeftWheelForward(uint8_t speed_percent)
{
    SafeDirectionChange(DIR_LEFT_FORWARD);

    HAL_GPIO_WritePin(PHASE_LEFT_GPIO_Port, PHASE_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PHASE_RIGHT_GPIO_Port, PHASE_RIGHT_Pin, GPIO_PIN_RESET);

    uint8_t duty;

    if (speed_percent > 100)
    {
        duty = SOFTWARE_PWM_PERIOD;
    }
    else
    {
        duty = (speed_percent * SOFTWARE_PWM_PERIOD) / 100;
    }

    pwm_left = 0;
    pwm_right = duty;

    current_direction = DIR_LEFT_FORWARD;
}

// Moves right wheel backward at specified speed (0–100%)
void MoveRightWheelBackward(uint8_t speed_percent)
{
    SafeDirectionChange(DIR_RIGHT_BACKWARD);

    HAL_GPIO_WritePin(PHASE_LEFT_GPIO_Port, PHASE_LEFT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PHASE_RIGHT_GPIO_Port, PHASE_RIGHT_Pin, GPIO_PIN_RESET);

    uint8_t duty;

    if (speed_percent > 100)
    {
        duty = SOFTWARE_PWM_PERIOD;
    }
    else
    {
        duty = (speed_percent * SOFTWARE_PWM_PERIOD) / 100;
    }

    pwm_left = duty;
    pwm_right = 0;

    current_direction = DIR_RIGHT_BACKWARD;
}

//moves left wheel backward at specified speed (0–100%)
void MoveLeftWheelBackward(uint8_t speed_percent)
{
    SafeDirectionChange(DIR_LEFT_BACKWARD);

    HAL_GPIO_WritePin(PHASE_LEFT_GPIO_Port, PHASE_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PHASE_RIGHT_GPIO_Port, PHASE_RIGHT_Pin, GPIO_PIN_SET);

    uint8_t duty;

    if (speed_percent > 100)
    {
        duty = SOFTWARE_PWM_PERIOD;
    }
    else
    {
        duty = (speed_percent * SOFTWARE_PWM_PERIOD) / 100;
    }

    pwm_left = 0;
    pwm_right = duty;

    current_direction = DIR_LEFT_BACKWARD;
}

// Spins robot right so it rotates in place at specified speed (0–100%)
void RotateRight(uint8_t speed_percent)
{
    SafeDirectionChange(DIR_ROTATE_RIGHT);

    HAL_GPIO_WritePin(PHASE_LEFT_GPIO_Port, PHASE_LEFT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PHASE_RIGHT_GPIO_Port, PHASE_RIGHT_Pin, GPIO_PIN_RESET);

    uint8_t duty;

    if (speed_percent > 100)
    {
        duty = SOFTWARE_PWM_PERIOD;
    }
    else
    {
        duty = (speed_percent * SOFTWARE_PWM_PERIOD) / 100;
    }

    pwm_left = duty;
    pwm_right = duty;

    current_direction = DIR_ROTATE_RIGHT;
}

// Spins robot left so it rotates in place at specified speed (0–100%)
void RotateLeft(uint8_t speed_percent)
{
    SafeDirectionChange(DIR_ROTATE_LEFT);

    HAL_GPIO_WritePin(PHASE_LEFT_GPIO_Port, PHASE_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PHASE_RIGHT_GPIO_Port, PHASE_RIGHT_Pin, GPIO_PIN_SET);

    uint8_t duty;

    if (speed_percent > 100)
    {
        duty = SOFTWARE_PWM_PERIOD;
    }
    else
    {
        duty = (speed_percent * SOFTWARE_PWM_PERIOD) / 100;
    }

    pwm_left = duty;
    pwm_right = duty;

    current_direction = DIR_ROTATE_LEFT;
}