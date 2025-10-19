#ifndef _PID_H
#define _PID_H

#include <stdint.h>

typedef struct {
	float set_point; /**< @brief 目标值 */
	float kp;        /**< @brief 比例增益 */
	float ki;        /**< @brief 积分增益 */
	float kd;        /**< @brief 微分增益 */
	
	float previous_0_err; /**< @brief 上次误差 */
	float previous_1_err; /**< @brief 上上次误差 */
	
	float output; /**< @brief PID输出 */
}PID_ControllerTypeDef;

void pid_controller_update(PID_ControllerTypeDef *self, float actual, float time_delta);

void pid_controller_init(PID_ControllerTypeDef *self, float kp, float ki, float kd);

#endif
