#ifndef _PID_H
#define _PID_H

#include <stdint.h>

typedef struct {
        float set_point; /**< @brief Target value */
        float kp;        /**< @brief Proportional gain */
        float ki;        /**< @brief Integral gain */
        float kd;        /**< @brief Derivative gain */

        float previous_0_err; /**< @brief Last error */
        float previous_1_err; /**< @brief Error from two steps ago */

        float output; /**< @brief PID output */
}PID_ControllerTypeDef;

void pid_controller_update(PID_ControllerTypeDef *self, float actual, float time_delta);

void pid_controller_init(PID_ControllerTypeDef *self, float kp, float ki, float kd);

#endif
