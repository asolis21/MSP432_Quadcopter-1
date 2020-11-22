/*
 * PID.h
 *
 *  Created on: Oct 5, 2020
 *      Author: Alex
 */

#ifndef PID_H_
#define PID_H_


/*PID Variable Indexes*/
#define P   0
#define I   1
#define D   2

/*PID Coefficient Indexes */
#define KP   0
#define KI   1
#define KD   2

#define ALPHA   0.15//0.87

typedef struct
{
    float setPoint;
    float error;
    float prevError;
    float pid[3];  //P - I - D variables
    float coefficients[3];
    float max;
    float min;
    float integral_max;
    float integral_min;
    float throttle;
    float command;
    float prevCommand;
}PID_t;

void pid_init(PID_t *pid);
float pid_update(PID_t *pid, float angle, float dt);
void pid_setStartPoint(PID_t *pid, float set_angle);
void pid_set_coefficients(PID_t *pid, float *coefficients);
void pid_maxmin_command(PID_t *pid, float max, float min);
void pid_maxmin_integral(PID_t *pid, float max, float min);
void setThrottle(PID_t * pid, float throttle);
void setPointUpdate(PID_t *channel0, const uint32_t channel0Value[]);


#endif /* PID_H_ */
