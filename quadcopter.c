#include "MPU6050.h"
#include "QMC5883.h"
#include "ESC.h"
#include "UARTDEBUG.h"
#include <math.h>
#include "PID.h"

#include "EasyHal/time_dev.h"

void *mainThread(void *arg0)
{
    time_dev_init();

    UARTDEBUG_init(9600);
    MPU6050_init(MPU6050_DEFAULT_ADDRESS);// Do we still have to write 0x00 to the power management
    QMC5883_init();
    ESC_init();

    ESC_arm();
    ESC_calibrate();

    int16_t raw_accel[3];
    int16_t raw_gyro[3];
    int16_t raw_mag[3];

    float accel[3];
    float gyro[3];
    float mag[3];

    float gyro_bias[3];

    float a_pitch, a_roll;
    float g_pitch, g_roll, g_yaw;
    float m_yaw;

    float Total_roll;
    float Total_pitch;
    float Total_yaw;

    //PID_t roll_pid;
    //PID_t pitch_pid;
    //PID_t yaw_pid;

    uint32_t start;
    float dt = 0.0;

    while(1)
    {
        start = millis();

        MPU6050_raw_accelerometer(raw_accel);
        MPU6050_ScaleRaw_accelerometer(accel, raw_accel);

        MPU6050_raw_gyroscope(raw_gyro);
        MPU6050_bias_gyroscope(gyro_bias, raw_gyro);
        MPU6050_ScaleRaw_gyroscope(gyro, raw_gyro, gyro_bias);

        QMC5883_raw_magnetometer(raw_mag);
        QMC5883_ScaleRaw_magnetometer(mag, raw_mag);

        //Compute Accelerometer Angles
        //a_roll = atan2f(accel[0], sqrt(accel[1]*accel[1] + accel[2]*accel[2])) * (180.0f/M_PI);//gymbolock
        //a_pitch =  atan2f(accel[1], sqrt(accel[0]*accel[0] + accel[2]*accel[2])) * (180.0f/M_PI);

        a_roll = atan2f(accel[0], accel[2]) * (180.0f/M_PI);//gymbolock
        a_pitch =  atan2f(accel[1], accel[2]) * (180.0f/M_PI);

        //Compute Gyro Angles
        g_roll = gyro[0]*dt;
        g_pitch = gyro[1]*dt;
        //g_yaw = gyro[2]*dt;

        //Compute Yaw Angles
        //m_yaw = atan2f(mag[1], mag[0]) * (180.0f/M_PI);
        //m_yaw = (m_yaw > 360.0f) ? (m_yaw - 360.0f) : m_yaw;
        //m_yaw = (m_yaw < 0) ? (m_yaw + 360.0f) : m_yaw;


        //Data Fusion
        //Complementary Filter
        //Total_roll  = 0.98*(a_roll + g_roll) + 0.02*(accel[0]);
        //Total_pitch = 0.98*(a_pitch + g_pitch) + 0.02*(accel[1]);
        //Total_yaw   = 0.98*(m_yaw + g_yaw) + 0.02*(mag[2]);

        Total_roll  = 0.99*(a_roll) + 0.01*(g_roll);
        Total_pitch = 0.99*(a_pitch) + 0.01*(g_pitch);
        //Total_yaw   = 0.98*(m_yaw + g_yaw) + 0.02*(mag[2]);


        //Update Counts (PID)
        //roll_pid  = (int32_t)pid_update(&roll_pid, Total_roll, dt);
        //pitch_pid = (int32_t)pid_update(&pitch_pid, Total_pitch, dt);
        //yaw_pid   = (int32_t)pid_update(&yaw_pid, Total_yaw, dt);


        //UARTDEBUG_printf("ax = %i, ay = %i, az = %i, ", raw_accel[0], raw_accel[1], raw_accel[2]);
        //UARTDEBUG_printf("gx = %i, gy = %f, gz = %i, ", raw_gyro[0], raw_gyro[1], raw_gyro[2]);
        //UARTDEBUG_printf("mx = %i, my = %i, mz = %i, dt = %f\r\n", raw_mag[0], raw_mag[1], raw_mag[2], dt);


        UARTDEBUG_printf("%f, %f, %f \n", Total_pitch , g_pitch, a_pitch);
        ///
        //
        //UARTDEBUG_printf("Total_p = %f, \n", Total_pitch);
        //UARTDEBUG_printf("Total_p = %f,\n ", Total_pitch);
        //UARTDEBUG_printf("m_t = %f,\n ", Total_yaw);






        dt = (millis() - start)/1e3;
    }
}


