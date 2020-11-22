#include "PID.h"
#include <math.h>
void pid_init(PID_t *pid)
{
    pid->error = 0.0;
    pid->command = 0.0;
    pid->prevCommand =0.0;
    pid->integral_max = 0.0;
    pid->setPoint = 0.0;// CHANGE BACK TO 0.0 AFTER TESTING
    pid->prevError = 0.0;
    pid->max = 2000;//2000
    pid->min = -2000;//-2000
    pid->integral_max = 600;
    pid->integral_min = -600;
    pid->coefficients[KP] = 30; //25*; Decrease from 40; when constant oscillations use Zeiger Nichols to find I and D
    pid->coefficients[KI] = 15; //0.1; TU = 1S, TI= TU/2= .5 KI=30*.5
    pid->coefficients[KD] = 20; //2.8;//30, 15, 15 works for single axis //2 Axis: 30,15,20

}

float pid_update(PID_t *pid, float actual_angle, float dt)
{
   /*Compute the error*/
   pid->error = pid->setPoint - actual_angle;

   //Calculate proportional term
   pid->pid[P] = pid->coefficients[KP]*pid->error;

   //Calculate integral term
   if(dt >1)
   {
       dt =0;
   }
   pid->pid[I] +=  (pid->coefficients[KI]) * (pid->error) * dt;

   if(pid->pid[I] > pid->integral_max)
   {
       pid->pid[I] = pid->integral_max;
   }
   if(pid->pid[I] < pid->integral_min)
   {
       pid->pid[I] = pid->integral_min;
   }

   //Calculate derivative term
   if(dt !=0.0)
     {
       pid->pid[D] = pid->coefficients[KD]*((pid->error - pid->prevError)/dt);
     }
   else
   {
       pid->pid[D] =0.0;
   }

   //Sum all the terms for the PID
   pid->command = pid->pid[P] + pid->pid[I] + pid->pid[D];

   //Constraint the PID value from min to max   |  min <= PID <= max
   if(pid->command < pid->min)
   {
       pid->command = pid->min;
   }

   if(pid->command > pid->max)
   {
       pid->command = pid->max;
   }

   pid->prevError = pid->error;

   return pid->command;
}

void setThrottle(PID_t * pid, float throttle)
{
    pid->throttle = throttle;
}

void pid_setStartPoint(PID_t *pid, float set_angle)
{
   pid->setPoint = set_angle;
}

void pid_set_coefficients(PID_t *pid, float *coefficients)
{
  pid->coefficients[KP] = coefficients[KP];
  pid->coefficients[KI] = coefficients[KP];
  pid->coefficients[KD] = coefficients[KD];
}

void pid_maxmin_command(PID_t *pid, float max, float min)
{
    pid->min = min;
    pid->max = max;
}

void pid_maxmin_integral(PID_t *pid, float max, float min)
{
    pid->integral_max = max;
    pid->integral_min = min;
}
