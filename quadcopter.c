#include "MPU6050.h"
#include "QMC5883.h"
#include "ESC.h"
#include "UARTDEBUG.h"
#include <math.h>
#include "PID.h"
#include "RC.h"

#include "EasyHal/time_dev.h"
#include "ti_drivers_config.h"

void *mainThread(void *arg0)
{
    time_dev_init();
    UARTDEBUG_init(9600);
    MPU6050_init(MPU6050_DEFAULT_ADDRESS);
    QMC5883_init();
    ESC_init();
    //ESC_calibrate();
    ESC_arm();
    PPM_init();


    int16_t raw_accel[3];
    int16_t raw_gyro[3];

    int32_t accel_offset[3] = {0, 0, 0};
    int32_t gyro_offset[3] = {0, 0, 0};
    int32_t mag_offset[3] = {0, 0, 0};

    //RC data
    uint32_t channels[8];

    //CALIBRATE ACCELEROMETER
    uint32_t j;
    for(j = 0; j < 200; j++)
    {
        MPU6050_raw_accelerometer(raw_accel);

        accel_offset[0] += raw_accel[0];
        accel_offset[1] += raw_accel[1];
        accel_offset[2] += raw_accel[2];

        usleep(4000);
    }

    //accel_offset[0] /= 200;
    //accel_offset[1] /= 200;
    //accel_offset[2] /= 200;

    //CALIBRATE GYROSCOPE
    uint32_t i;
    for(i = 0; i < 1000; i++)
    {
        MPU6050_raw_gyroscope(raw_gyro);

        gyro_offset[0] += raw_gyro[0];
        gyro_offset[1] += raw_gyro[1];
        gyro_offset[2] += raw_gyro[2];

        usleep(4000);
    }

    gyro_offset[0] /= 1000;
    gyro_offset[1] /= 1000;
    gyro_offset[2] /= 1000;


    float accel[3];
    float gyro[3];
    float mag[3];

    float a_pitch, a_roll;
    float g_pitch, g_roll, g_yaw;
    float m_pitch, m_roll, m_yaw;

    float Total_roll = 0.0;
    float Total_pitch = 0.0;
    float Total_yaw = 0.0;

    PID_t roll_pid;
    PID_t pitch_pid;
    PID_t yaw_pid;
    //PID_t channel_0;

    int32_t ROLL_PID;
    int32_t PITCH_PID;
    int32_t YAW_PID;

    uint32_t start;
    float dt = 0.0;

    pid_init(&roll_pid);
    pid_init(&pitch_pid);
    pid_init(&yaw_pid);
    //pid_init(&channel_0);


    ESC_speed(ESC0, 16000);
    ESC_speed(ESC1, 16000);
    ESC_speed(ESC2, 16000);
    ESC_speed(ESC3, 16000);

    while(1)
    {
        start = millis();

        PPM_channels(channels);
        //setPointUpdate(&channel_0, channels);

        /***************ACCELEROMETER_RAW***************/
        MPU6050_raw_accelerometer(raw_accel);

        //raw_accel[0] -= accel_offset[0];
        //raw_accel[1] -= accel_offset[1];
        //raw_accel[2] -= accel_offset[2];

        accel[0] = (float)raw_accel[0];
        accel[1] = (float)raw_accel[1];
        accel[2] = (float)raw_accel[2];

        /****************GYROSCOPE_RAW*****************/
        MPU6050_raw_gyroscope(raw_gyro);

        raw_gyro[0] -= gyro_offset[0];
        raw_gyro[1] -= gyro_offset[1];
        raw_gyro[2] -= gyro_offset[2];

        gyro[0] = ((float)raw_gyro[0]/131.0f)*dt;
        gyro[1] = ((float)raw_gyro[1]/131.0f)*dt;
        gyro[2] = ((float)raw_gyro[2]/131.0f)*dt;


        /****************MAGNETOMETER_RAW*****************/
        //QMC5883_raw_magnetometer(raw_mag);
        QMC5883_magnetometer(mag, mag_offset);

        /****************ACCELEROMETER_ANGLES*****************/
        a_roll = atan2f(accel[0], sqrt(accel[1]*accel[1] + accel[2]*accel[2])) * (180.0f/M_PI);
        a_pitch =  atan2f(accel[1], sqrt(accel[0]*accel[0] + accel[2]*accel[2])) * (180.0f/M_PI);

        /****************GYROSCOPE_ANGLES*****************/
        g_pitch += gyro[0];
        g_roll += gyro[1];
        g_yaw += gyro[2];

        /****************MAGNETOMETER_ANGLES*****************///m_pitch = mag[0], m_roll= mag[1], m_yaw= mag[2]
        m_pitch = -(mag[0])/57.2957;
        m_roll  = -(mag[1])/57.2957;

        float hx = mag[0]*cos(m_pitch) + mag[1]*sin(m_roll)*sin(m_pitch) - mag[2]*cos(m_roll)*sin(m_pitch);
        float hy = mag[1]*cos(m_roll) + mag[2]*sin(m_roll);

        float heading = atan2(hy, hx)*57.2957;

        if(hy < 0) heading = 180 + (180 + atan2(hy, hx)*57.2957);
        else       heading = atan2(hy, hx)*57.2957;

        m_yaw = heading;

        /********DATA FUSION**********/
        //Complementary Filter
        Total_roll  = 0.99*(g_roll) + 0.01*(a_roll);
        Total_pitch = 0.99*(g_pitch) + 0.01*(a_pitch);
        Total_yaw   = 0.98*(g_yaw) + 0.02*(g_yaw);// Switch Mag back after Testing

        //Update Counts (PID)
        ROLL_PID  = (int32_t)pid_update(&roll_pid, Total_roll, dt);
        PITCH_PID = (int32_t)pid_update(&pitch_pid, Total_pitch, dt);
        YAW_PID   = (int32_t)pid_update(&yaw_pid, Total_yaw, dt);

        /*Two Axis*/
        ESC_speed(ESC0, 16000 + PITCH_PID - ROLL_PID + YAW_PID); //Yaw: if we turn Drone Clockwise; motor
        ESC_speed(ESC1, 16000 - PITCH_PID - ROLL_PID - YAW_PID);
        ESC_speed(ESC2, 16000 + PITCH_PID + ROLL_PID + YAW_PID);
        ESC_speed(ESC3, 16000 - PITCH_PID + ROLL_PID - YAW_PID);

        //Kill switch
        if(channels[6] < 1500)
        {
        ESC_speed(ESC0, 0);
        ESC_speed(ESC1, 0);
        ESC_speed(ESC2, 0);
        ESC_speed(ESC3, 0);
        }

        UARTDEBUG_printf("Roll = %f   Pitch = %f   Yaw = %f\n", Total_roll, Total_pitch, Total_yaw);


        dt = (millis() - start)/1e3;

    }
}


