#include "main.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

/* ============================================================
 * HANDLES
 * ============================================================ */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

I2C_HandleTypeDef hi2c2;


/* ============================================================
 * VESC COMMANDS
 * ============================================================ */

#define COMM_GET_VALUES        4
#define COMM_SET_RPM           8

#define VESC_TIMEOUT_MS        100


/* ============================================================
 * MPU6050
 * ============================================================ */

#define MPU6050_ADDR           (0x68 << 1)

#define MPU6050_SMPLRT_DIV     0x19
#define MPU6050_CONFIG         0x1A
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_CONFIG   0x1C
#define MPU6050_ACCEL_XOUT_H   0x3B
#define MPU6050_PWR_MGMT_1     0x6B
#define MPU6050_WHO_AM_I       0x75


/* ============================================================
 * COMPLEMENTARY FILTER
 * ============================================================ */

#define FILTER_GYRO_WEIGHT     0.98f
#define FILTER_ACCEL_WEIGHT    0.02f


/* ============================================================
 * BALANCE CONTROLLER
 *
 * REDUCED FOR INITIAL GROUND TEST
 * ============================================================ */

#define KP_RPM                 80.0f
#define KD_RPM                  8.0f


/* ============================================================
 * MOTOR LIMITS
 * ============================================================ */

#define MAX_ERPM              1500
#define MAX_ERPM_STEP           60
#define ERPM_FILTER_ALPHA      0.15f


/* ============================================================
 * START / ARM SETTINGS
 * ============================================================ */

#define ARM_TILT_ANGLE          0.7f

#define BALANCE_DEADBAND        0.3f

#define TILT_CUTOFF            10.0f


/* ============================================================
 * MOTOR DIRECTION
 *
 * If correction direction is wrong:
 * change 1.0f to -1.0f
 * ============================================================ */

#define MOTOR_SIGN              1.0f


/* ============================================================
 * CALIBRATION
 * ============================================================ */

#define GYRO_CAL_SAMPLES        500
#define PITCH_ZERO_SAMPLES      200


/* ============================================================
 * CONTROL LOOP
 * ============================================================ */

#define CONTROL_PERIOD_MS       10


/* ============================================================
 * STARTUP
 * ============================================================ */

#define STARTUP_ZERO_TIME       3000


/* ============================================================
 * VESC DATA
 * ============================================================ */

typedef struct
{
    int32_t motor_current_x100;
    int32_t input_current_x100;
    int32_t input_voltage_x10;
    int32_t rpm;
    int16_t duty_x1000;
    uint8_t fault;

} VESC_Data_t;


VESC_Data_t left_vesc;
VESC_Data_t right_vesc;


/* ============================================================
 * GLOBAL VARIABLES
 * ============================================================ */

static float gyro_y_offset = 0.0f;
static float pitch_zero = 0.0f;

static float filtered_pitch = 0.0f;
static float gyro_filtered = 0.0f;

static float commanded_erpm = 0.0f;
static float smoothed_erpm = 0.0f;


/*
 * 0 = motors locked
 * 1 = balance controller active
 */

static uint8_t balance_armed = 0;


/*
 * Prevents derivative kick during first
 * control cycle after arming.
 */

static uint8_t just_armed = 0;


/* ============================================================
 * FUNCTION PROTOTYPES
 * ============================================================ */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);

static void Debug_Print(const char *text);

static uint16_t VESC_CRC16(
        const uint8_t *data,
        uint16_t len);

static uint8_t VESC_Set_RPM(
        UART_HandleTypeDef *huart,
        int32_t erpm);

static uint8_t VESC_Get_Values(
        UART_HandleTypeDef *huart,
        VESC_Data_t *data);

static void Stop_Motors(void);

static uint8_t MPU6050_Write(
        uint8_t reg,
        uint8_t value);

static uint8_t MPU6050_Read(
        uint8_t reg,
        uint8_t *data,
        uint8_t len);

static uint8_t MPU6050_Init(void);

static uint8_t MPU6050_Read_Raw(
        int16_t *ax,
        int16_t *ay,
        int16_t *az,
        int16_t *gx,
        int16_t *gy,
        int16_t *gz);

static uint8_t Calibrate_Gyro(void);

static uint8_t Calibrate_Pitch_Zero(void);

static float Update_Complementary_Filter(
        float accel_pitch,
        float gyro_y_dps,
        float dt);

static float Calculate_PD_ERPM(
        float pitch,
        float gyro_dps);

static float Smooth_ERPM(
        float target);

static float Limit_ERPM_Rate(
        float target);


/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    float accel_pitch;
    float gyro_y_dps;

    float pitch_error;
    float pd_erpm;
    float target_erpm;

    float dt;

    uint32_t last_control_time;
    uint32_t telemetry_time;
    uint32_t startup_time;

    char msg[350];


    /* ========================================================
     * INITIALIZE HAL
     * ======================================================== */

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();

    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();

    MX_I2C2_Init();

    HAL_Delay(100);


    /* ========================================================
     * HEADER
     * ======================================================== */

    Debug_Print(
        "\r\n\r\n"
        "================================================\r\n"
        " STM32H7A3ZI-Q + MPU6050 + DUAL VESC\r\n"
        " SAFE LOW-SPEED BALANCE CONTROLLER\r\n"
        "================================================\r\n"
        "I2C2  : PB10 SCL / PB11 SDA\r\n"
        "USART1: LEFT VESC\r\n"
        "USART2: RIGHT VESC\r\n"
        "USART3: DEBUG\r\n"
        "BAUD  : 115200\r\n"
        "================================================\r\n"
    );


    /* ========================================================
     * INITIAL STATE
     * ======================================================== */

    balance_armed = 0;
    just_armed = 0;

    commanded_erpm = 0.0f;
    smoothed_erpm = 0.0f;

    filtered_pitch = 0.0f;
    gyro_filtered = 0.0f;


    /* ========================================================
     * FORCE MOTORS OFF
     * ======================================================== */

    Stop_Motors();

    Debug_Print(
        "MOTORS LOCKED OFF\r\n"
    );


    /* ========================================================
     * MPU6050 INITIALIZATION
     * ======================================================== */

    Debug_Print(
        "\r\n"
        "MPU6050 INITIALIZATION...\r\n"
    );

    if (!MPU6050_Init())
    {
        Debug_Print(
            "MPU6050 INIT FAILED\r\n"
        );

        Stop_Motors();

        while (1)
        {
            HAL_Delay(1000);
        }
    }

    Debug_Print(
        "MPU6050 INIT PASS\r\n"
    );


    /* ========================================================
     * GYRO CALIBRATION
     * ======================================================== */

    Stop_Motors();

    Debug_Print(
        "\r\n"
        "KEEP FRAME COMPLETELY STILL\r\n"
        "GYRO CALIBRATION...\r\n"
    );

    HAL_Delay(1000);

    if (!Calibrate_Gyro())
    {
        Debug_Print(
            "GYRO CALIBRATION FAILED\r\n"
        );

        Stop_Motors();

        while (1)
        {
            HAL_Delay(1000);
        }
    }

    Debug_Print(
        "GYRO CALIBRATION PASS\r\n"
    );


    /* ========================================================
     * PITCH ZERO
     * ======================================================== */

    Stop_Motors();

    Debug_Print(
        "\r\n"
        "KEEP FRAME UPRIGHT\r\n"
        "PITCH ZERO CALIBRATION...\r\n"
    );

    HAL_Delay(1000);

    if (!Calibrate_Pitch_Zero())
    {
        Debug_Print(
            "PITCH ZERO CALIBRATION FAILED\r\n"
        );

        Stop_Motors();

        while (1)
        {
            HAL_Delay(1000);
        }
    }

    Debug_Print(
        "PITCH ZERO CALIBRATION PASS\r\n"
    );


    /* ========================================================
     * RESET FILTERS
     * ======================================================== */

    filtered_pitch = pitch_zero;
    gyro_filtered = 0.0f;

    commanded_erpm = 0.0f;
    smoothed_erpm = 0.0f;

    balance_armed = 0;
    just_armed = 0;


    /* ========================================================
     * ZERO STARTUP PERIOD
     * ======================================================== */

    Debug_Print(
        "\r\n"
        "================================================\r\n"
        " CALIBRATION COMPLETE\r\n"
        " MOTORS STILL LOCKED OFF\r\n"
        "================================================\r\n"
    );

    Debug_Print(
        "KEEP FRAME STILL...\r\n"
    );

    startup_time = HAL_GetTick();

    while ((HAL_GetTick() - startup_time)
           < STARTUP_ZERO_TIME)
    {
        VESC_Set_RPM(
            &huart1,
            0
        );

        VESC_Set_RPM(
            &huart2,
            0
        );

        HAL_Delay(50);
    }


    /* ========================================================
     * READY
     * ======================================================== */

    Stop_Motors();

    balance_armed = 0;
    just_armed = 0;

    commanded_erpm = 0.0f;
    smoothed_erpm = 0.0f;

    Debug_Print(
        "\r\n"
        "================================================\r\n"
        " SYSTEM READY\r\n"
        " MOTORS OFF\r\n"
        "================================================\r\n"
    );

    Debug_Print(
        "Tilt > 1.5 deg to ARM\r\n"
    );

    Debug_Print(
        "Motor command starts from ZERO\r\n"
    );

    Debug_Print(
        "MAX ERPM = 1500\r\n"
    );

    Debug_Print(
        "TILT > 20 deg = SAFETY STOP\r\n"
    );


    last_control_time = HAL_GetTick();
    telemetry_time = HAL_GetTick();


    /* ========================================================
     * MAIN CONTROL LOOP
     * ======================================================== */

    while (1)
    {
        uint32_t now;

        now = HAL_GetTick();


        /* ====================================================
         * 10 ms LOOP
         * ==================================================== */

        if ((now - last_control_time)
            < CONTROL_PERIOD_MS)
        {
            continue;
        }


        dt =
            (float)(now - last_control_time)
            / 1000.0f;

        last_control_time = now;


        if (dt <= 0.0f || dt > 0.05f)
        {
            dt = 0.01f;
        }


        /* ====================================================
         * READ MPU
         * ==================================================== */

        if (!MPU6050_Read_Raw(
                &ax,
                &ay,
                &az,
                &gx,
                &gy,
                &gz))
        {
            Debug_Print(
                "\r\n"
                "MPU READ ERROR\r\n"
            );

            balance_armed = 0;
            just_armed = 0;

            commanded_erpm = 0.0f;
            smoothed_erpm = 0.0f;

            Stop_Motors();

            HAL_Delay(100);

            continue;
        }


        /* ====================================================
         * GYRO
         * ==================================================== */

        gyro_y_dps =
            ((float)gy / 131.0f)
            -
            gyro_y_offset;


        /* ====================================================
         * GYRO LOW-PASS FILTER
         * ==================================================== */

        gyro_filtered =
            gyro_filtered
            +
            0.20f *
            (gyro_y_dps - gyro_filtered);


        /* ====================================================
         * ACCELEROMETER PITCH
         * ==================================================== */

        accel_pitch =
            atan2f(
                (float)ax,
                sqrtf(
                    ((float)ay * (float)ay)
                    +
                    ((float)az * (float)az)
                )
            )
            *
            57.2957795f;


        /* ====================================================
         * COMPLEMENTARY FILTER
         * ==================================================== */

        filtered_pitch =
            Update_Complementary_Filter(
                accel_pitch,
                gyro_filtered,
                dt
            );


        /* ====================================================
         * PITCH ERROR
         * ==================================================== */

        pitch_error =
            filtered_pitch
            -
            pitch_zero;


        /* ====================================================
         * HARD TILT SAFETY
         * ==================================================== */

        if (fabsf(pitch_error)
            >= TILT_CUTOFF)
        {
            balance_armed = 0;
            just_armed = 0;

            commanded_erpm = 0.0f;
            smoothed_erpm = 0.0f;

            Stop_Motors();

            Debug_Print(
                "\r\n"
                "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n"
                " TILT CUTOFF\r\n"
                " MOTORS STOPPED\r\n"
                "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n"
            );

            HAL_Delay(100);

            continue;
        }


        /* ====================================================
         * ARMING
         * ==================================================== */

        if (!balance_armed)
        {
            commanded_erpm = 0.0f;
            smoothed_erpm = 0.0f;

            Stop_Motors();


            /*
             * Intentional tilt detected
             */

            if (fabsf(pitch_error)
                >= ARM_TILT_ANGLE)
            {
                balance_armed = 1;
                just_armed = 1;

                commanded_erpm = 0.0f;
                smoothed_erpm = 0.0f;

                Debug_Print(
                    "\r\n"
                    ">>> BALANCE CONTROLLER ARMED <<<\r\n"
                );

                Debug_Print(
                    ">>> MOTOR COMMAND = 0 ERPM <<<\r\n"
                );
            }


            /*
             * Do not calculate PD in same
             * cycle as arming.
             */

            if (just_armed)
            {
                continue;
            }

            continue;
        }


        /* ====================================================
         * FIRST CYCLE AFTER ARM
         * ==================================================== */

        if (just_armed)
        {
            /*
             * Clear derivative history.
             */

            gyro_filtered = 0.0f;

            smoothed_erpm = 0.0f;
            commanded_erpm = 0.0f;

            just_armed = 0;


            VESC_Set_RPM(
                &huart1,
                0
            );

            VESC_Set_RPM(
                &huart2,
                0
            );

            continue;
        }


        /* ====================================================
         * PD CONTROLLER
         * ==================================================== */

        pd_erpm =
            Calculate_PD_ERPM(
                pitch_error,
                gyro_filtered
            );


        /* ====================================================
         * LIMIT
         * ==================================================== */

        if (pd_erpm > MAX_ERPM)
        {
            pd_erpm = MAX_ERPM;
        }

        if (pd_erpm < -MAX_ERPM)
        {
            pd_erpm = -MAX_ERPM;
        }


        /* ====================================================
         * SMOOTHING
         * ==================================================== */

        target_erpm =
            Smooth_ERPM(
                pd_erpm
            );


        /* ====================================================
         * SLEW RATE LIMIT
         * ==================================================== */

        commanded_erpm =
            Limit_ERPM_Rate(
                target_erpm
            );


        /* ====================================================
         * LEFT VESC
         * ==================================================== */

        if (!VESC_Set_RPM(
                &huart1,
                (int32_t)commanded_erpm))
        {
            balance_armed = 0;

            commanded_erpm = 0.0f;
            smoothed_erpm = 0.0f;

            Stop_Motors();

            continue;
        }


        /* ====================================================
         * RIGHT VESC
         * ==================================================== */

        if (!VESC_Set_RPM(
                &huart2,
                (int32_t)commanded_erpm))
        {
            balance_armed = 0;

            commanded_erpm = 0.0f;
            smoothed_erpm = 0.0f;

            Stop_Motors();

            continue;
        }


        /* ====================================================
         * TELEMETRY
         * ==================================================== */

        if ((now - telemetry_time) >= 100)
        {
            int32_t pitch_x100;
            int32_t gyro_x100;
            int32_t pd_x10;

            telemetry_time = now;


            VESC_Get_Values(
                &huart1,
                &left_vesc
            );

            VESC_Get_Values(
                &huart2,
                &right_vesc
            );


            /*
             * Convert floats to integers.
             */

            pitch_x100 =
                (int32_t)(pitch_error * 100.0f);

            gyro_x100 =
                (int32_t)(gyro_filtered * 100.0f);

            pd_x10 =
                (int32_t)(pd_erpm * 10.0f);


            snprintf(
                msg,
                sizeof(msg),

                "STATE=%s | "
                "PITCH=%ld.%02ld | "
                "GYRO=%ld.%02ld | "
                "PD=%ld.%01ld | "
                "CMD=%ld | "
                "L=%ld | "
                "R=%ld | "
                "F=%u/%u\r\n",

                balance_armed ?
                "ARMED" :
                "OFF",

                /* PITCH */

                (long)(pitch_x100 / 100),

                (long)labs(
                    pitch_x100 % 100
                ),


                /* GYRO */

                (long)(gyro_x100 / 100),

                (long)labs(
                    gyro_x100 % 100
                ),


                /* PD */

                (long)(pd_x10 / 10),

                (long)labs(
                    pd_x10 % 10
                ),


                /* COMMAND */

                (long)commanded_erpm,


                /* LEFT */

                (long)left_vesc.rpm,


                /* RIGHT */

                (long)right_vesc.rpm,


                /* FAULTS */

                left_vesc.fault,
                right_vesc.fault
            );


            Debug_Print(msg);
        }
    }
}


/* ============================================================
 * DEBUG PRINT
 * ============================================================ */

static void Debug_Print(
        const char *text)
{
    HAL_UART_Transmit(
        &huart3,
        (uint8_t *)text,
        strlen(text),
        1000
    );
}


/* ============================================================
 * VESC CRC16
 * ============================================================ */

static uint16_t VESC_CRC16(
        const uint8_t *data,
        uint16_t len)
{
    uint16_t crc = 0;

    for (uint16_t i = 0;
         i < len;
         i++)
    {
        crc ^=
            ((uint16_t)data[i]) << 8;

        for (uint8_t j = 0;
             j < 8;
             j++)
        {
            if (crc & 0x8000)
            {
                crc =
                    (crc << 1)
                    ^
                    0x1021;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}


/* ============================================================
 * VESC SET RPM
 * ============================================================ */

static uint8_t VESC_Set_RPM(
        UART_HandleTypeDef *huart,
        int32_t erpm)
{
    uint8_t tx[10];
    uint16_t crc;

    tx[0] = 0x02;
    tx[1] = 0x05;
    tx[2] = COMM_SET_RPM;

    tx[3] =
        (uint8_t)((erpm >> 24) & 0xFF);

    tx[4] =
        (uint8_t)((erpm >> 16) & 0xFF);

    tx[5] =
        (uint8_t)((erpm >> 8) & 0xFF);

    tx[6] =
        (uint8_t)(erpm & 0xFF);


    crc =
        VESC_CRC16(
            &tx[2],
            5
        );


    tx[7] =
        (uint8_t)(crc >> 8);

    tx[8] =
        (uint8_t)(crc & 0xFF);

    tx[9] = 0x03;


    if (HAL_UART_Transmit(
            huart,
            tx,
            10,
            VESC_TIMEOUT_MS)
        != HAL_OK)
    {
        return 0;
    }

    return 1;
}


/* ============================================================
 * STOP MOTORS
 * ============================================================ */

static void Stop_Motors(void)
{
    VESC_Set_RPM(
        &huart1,
        0
    );

    VESC_Set_RPM(
        &huart2,
        0
    );
}


/* ============================================================
 * VESC GET VALUES
 * ============================================================ */

static uint8_t VESC_Get_Values(
        UART_HandleTypeDef *huart,
        VESC_Data_t *data)
{
    uint8_t tx[6];

    uint16_t crc;

    uint8_t first;
    uint8_t length;

    uint8_t packet[100];

    uint16_t received_crc;
    uint16_t calculated_crc;


    tx[0] = 0x02;
    tx[1] = 0x01;
    tx[2] = COMM_GET_VALUES;


    crc =
        VESC_CRC16(
            &tx[2],
            1
        );


    tx[3] =
        (uint8_t)(crc >> 8);

    tx[4] =
        (uint8_t)(crc & 0xFF);

    tx[5] = 0x03;


    __HAL_UART_CLEAR_OREFLAG(
        huart
    );


    if (HAL_UART_Transmit(
            huart,
            tx,
            6,
            VESC_TIMEOUT_MS)
        != HAL_OK)
    {
        return 0;
    }


    if (HAL_UART_Receive(
            huart,
            &first,
            1,
            VESC_TIMEOUT_MS)
        != HAL_OK)
    {
        return 0;
    }


    if (first != 0x02)
    {
        return 0;
    }


    if (HAL_UART_Receive(
            huart,
            &length,
            1,
            VESC_TIMEOUT_MS)
        != HAL_OK)
    {
        return 0;
    }


    if (length > 90)
    {
        return 0;
    }


    if (HAL_UART_Receive(
            huart,
            packet,
            length + 3,
            VESC_TIMEOUT_MS)
        != HAL_OK)
    {
        return 0;
    }


    if (packet[length + 2] != 0x03)
    {
        return 0;
    }


    if (packet[0] != COMM_GET_VALUES)
    {
        return 0;
    }


    calculated_crc =
        VESC_CRC16(
            packet,
            length
        );


    received_crc =
        ((uint16_t)packet[length] << 8)
        |
        packet[length + 1];


    if (calculated_crc != received_crc)
    {
        return 0;
    }


    if (length < 54)
    {
        return 0;
    }


    data->motor_current_x100 =
        ((int32_t)packet[5] << 24)
        |
        ((int32_t)packet[6] << 16)
        |
        ((int32_t)packet[7] << 8)
        |
        packet[8];


    data->input_current_x100 =
        ((int32_t)packet[9] << 24)
        |
        ((int32_t)packet[10] << 16)
        |
        ((int32_t)packet[11] << 8)
        |
        packet[12];


    data->duty_x1000 =
        (int16_t)(
            ((uint16_t)packet[21] << 8)
            |
            packet[22]
        );


    data->rpm =
        ((int32_t)packet[23] << 24)
        |
        ((int32_t)packet[24] << 16)
        |
        ((int32_t)packet[25] << 8)
        |
        packet[26];


    data->input_voltage_x10 =
        (int16_t)(
            ((uint16_t)packet[27] << 8)
            |
            packet[28]
        );


    data->fault =
        packet[53];


    return 1;
}


/* ============================================================
 * MPU WRITE
 * ============================================================ */

static uint8_t MPU6050_Write(
        uint8_t reg,
        uint8_t value)
{
    uint8_t data[2];

    data[0] = reg;
    data[1] = value;


    return
        HAL_I2C_Master_Transmit(
            &hi2c2,
            MPU6050_ADDR,
            data,
            2,
            100
        )
        ==
        HAL_OK;
}


/* ============================================================
 * MPU READ
 * ============================================================ */

static uint8_t MPU6050_Read(
        uint8_t reg,
        uint8_t *data,
        uint8_t len)
{
    if (HAL_I2C_Master_Transmit(
            &hi2c2,
            MPU6050_ADDR,
            &reg,
            1,
            100)
        != HAL_OK)
    {
        return 0;
    }


    if (HAL_I2C_Master_Receive(
            &hi2c2,
            MPU6050_ADDR,
            data,
            len,
            100)
        != HAL_OK)
    {
        return 0;
    }


    return 1;
}


/* ============================================================
 * MPU INIT
 * ============================================================ */

static uint8_t MPU6050_Init(void)
{
    uint8_t who_am_i;


    if (!MPU6050_Read(
            MPU6050_WHO_AM_I,
            &who_am_i,
            1))
    {
        return 0;
    }


    if (who_am_i != 0x68)
    {
        return 0;
    }


    /* Wake */

    if (!MPU6050_Write(
            MPU6050_PWR_MGMT_1,
            0x00))
    {
        return 0;
    }

    HAL_Delay(100);


    /* Sample rate */

    if (!MPU6050_Write(
            MPU6050_SMPLRT_DIV,
            7))
    {
        return 0;
    }


    /* Digital low-pass filter */

    if (!MPU6050_Write(
            MPU6050_CONFIG,
            3))
    {
        return 0;
    }


    /* Gyro ±250 dps */

    if (!MPU6050_Write(
            MPU6050_GYRO_CONFIG,
            0x00))
    {
        return 0;
    }


    /* Accelerometer ±2g */

    if (!MPU6050_Write(
            MPU6050_ACCEL_CONFIG,
            0x00))
    {
        return 0;
    }


    return 1;
}


/* ============================================================
 * MPU RAW DATA
 * ============================================================ */

static uint8_t MPU6050_Read_Raw(
        int16_t *ax,
        int16_t *ay,
        int16_t *az,
        int16_t *gx,
        int16_t *gy,
        int16_t *gz)
{
    uint8_t data[14];


    if (!MPU6050_Read(
            MPU6050_ACCEL_XOUT_H,
            data,
            14))
    {
        return 0;
    }


    *ax =
        (int16_t)(
            ((uint16_t)data[0] << 8)
            |
            data[1]
        );


    *ay =
        (int16_t)(
            ((uint16_t)data[2] << 8)
            |
            data[3]
        );


    *az =
        (int16_t)(
            ((uint16_t)data[4] << 8)
            |
            data[5]
        );


    *gx =
        (int16_t)(
            ((uint16_t)data[8] << 8)
            |
            data[9]
        );


    *gy =
        (int16_t)(
            ((uint16_t)data[10] << 8)
            |
            data[11]
        );


    *gz =
        (int16_t)(
            ((uint16_t)data[12] << 8)
            |
            data[13]
        );


    return 1;
}


/* ============================================================
 * GYRO CALIBRATION
 * ============================================================ */

static uint8_t Calibrate_Gyro(void)
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    int64_t sum_gy = 0;


    for (int i = 0;
         i < GYRO_CAL_SAMPLES;
         i++)
    {
        if (!MPU6050_Read_Raw(
                &ax,
                &ay,
                &az,
                &gx,
                &gy,
                &gz))
        {
            return 0;
        }


        sum_gy += gy;

        HAL_Delay(2);
    }


    gyro_y_offset =
        ((float)sum_gy /
         (float)GYRO_CAL_SAMPLES)
        /
        131.0f;


    return 1;
}


/* ============================================================
 * PITCH ZERO CALIBRATION
 * ============================================================ */

static uint8_t Calibrate_Pitch_Zero(void)
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    float sum = 0.0f;


    for (int i = 0;
         i < PITCH_ZERO_SAMPLES;
         i++)
    {
        float accel_pitch;


        if (!MPU6050_Read_Raw(
                &ax,
                &ay,
                &az,
                &gx,
                &gy,
                &gz))
        {
            return 0;
        }


        accel_pitch =
            atan2f(
                (float)ax,
                sqrtf(
                    ((float)ay * (float)ay)
                    +
                    ((float)az * (float)az)
                )
            )
            *
            57.2957795f;


        sum += accel_pitch;

        HAL_Delay(5);
    }


    pitch_zero =
        sum /
        (float)PITCH_ZERO_SAMPLES;


    return 1;
}


/* ============================================================
 * COMPLEMENTARY FILTER
 * ============================================================ */

static float Update_Complementary_Filter(
        float accel_pitch,
        float gyro_y_dps,
        float dt)
{
    float gyro_angle;


    gyro_angle =
        filtered_pitch
        +
        gyro_y_dps * dt;


    filtered_pitch =
        FILTER_GYRO_WEIGHT *
        gyro_angle
        +
        FILTER_ACCEL_WEIGHT *
        accel_pitch;


    return filtered_pitch;
}


/* ============================================================
 * PD CONTROLLER
 * ============================================================ */

static float Calculate_PD_ERPM(
        float pitch,
        float gyro_dps)
{
    float effective_pitch;
    float erpm;


    /*
     * Remove the intentional arming zone.
     *
     * Example:
     *
     * pitch = 1.5 deg
     * effective = 0
     *
     * pitch = 2.0 deg
     * effective = 0.5 deg
     *
     * This prevents the arm action itself from
     * producing a large proportional command.
     */

    if (fabsf(pitch) <= ARM_TILT_ANGLE)
    {
        effective_pitch = 0.0f;
    }
    else
    {
        if (pitch > 0.0f)
        {
            effective_pitch =
                pitch - ARM_TILT_ANGLE;
        }
        else
        {
            effective_pitch =
                pitch + ARM_TILT_ANGLE;
        }
    }


    /*
     * Balance deadband
     */

    if (fabsf(effective_pitch)
        < BALANCE_DEADBAND)
    {
        effective_pitch = 0.0f;
    }


    /*
     * PD controller
     */

    erpm =
        (KP_RPM * effective_pitch)
        +
        (KD_RPM * gyro_dps);


    erpm *= MOTOR_SIGN;


    /*
     * Limit
     */

    if (erpm > MAX_ERPM)
    {
        erpm = MAX_ERPM;
    }


    if (erpm < -MAX_ERPM)
    {
        erpm = -MAX_ERPM;
    }


    return erpm;
}


/* ============================================================
 * ERPM SMOOTHING
 * ============================================================ */

static float Smooth_ERPM(
        float target)
{
    smoothed_erpm =
        smoothed_erpm
        +
        ERPM_FILTER_ALPHA *
        (target - smoothed_erpm);


    return smoothed_erpm;
}


/* ============================================================
 * ERPM SLEW LIMIT
 * ============================================================ */

static float Limit_ERPM_Rate(
        float target)
{
    float difference;


    difference =
        target - commanded_erpm;


    if (difference > MAX_ERPM_STEP)
    {
        difference =
            MAX_ERPM_STEP;
    }


    if (difference < -MAX_ERPM_STEP)
    {
        difference =
            -MAX_ERPM_STEP;
    }


    return commanded_erpm + difference;
}


/* ============================================================
 * SYSTEM CLOCK
 * ============================================================ */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    HAL_PWREx_ConfigSupply(
        PWR_DIRECT_SMPS_SUPPLY
    );


    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );


    while (!__HAL_PWR_GET_FLAG(
                PWR_FLAG_VOSRDY))
    {
    }


    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_NONE;


    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct)
        != HAL_OK)
    {
        Error_Handler();
    }


    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2 |
        RCC_CLOCKTYPE_D3PCLK1 |
        RCC_CLOCKTYPE_D1PCLK1;


    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_HSI;

    RCC_ClkInitStruct.SYSCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_HCLK_DIV1;

    RCC_ClkInitStruct.APB3CLKDivider =
        RCC_APB3_DIV2;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_APB1_DIV2;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_APB2_DIV2;

    RCC_ClkInitStruct.APB4CLKDivider =
        RCC_APB4_DIV2;


    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_2)
        != HAL_OK)
    {
        Error_Handler();
    }
}


/* ============================================================
 * USART1 - LEFT VESC
 * ============================================================ */

static void MX_USART1_UART_Init(void)
{
    huart1.Instance =
        USART1;

    huart1.Init.BaudRate =
        115200;

    huart1.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart1.Init.StopBits =
        UART_STOPBITS_1;

    huart1.Init.Parity =
        UART_PARITY_NONE;

    huart1.Init.Mode =
        UART_MODE_TX_RX;

    huart1.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart1.Init.OverSampling =
        UART_OVERSAMPLING_16;

    huart1.Init.OneBitSampling =
        UART_ONE_BIT_SAMPLE_DISABLE;

    huart1.Init.ClockPrescaler =
        UART_PRESCALER_DIV1;

    huart1.AdvancedInit.AdvFeatureInit =
        UART_ADVFEATURE_NO_INIT;


    if (HAL_UART_Init(&huart1)
        != HAL_OK)
    {
        Error_Handler();
    }
}


/* ============================================================
 * USART2 - RIGHT VESC
 * ============================================================ */

static void MX_USART2_UART_Init(void)
{
    huart2.Instance =
        USART2;

    huart2.Init.BaudRate =
        115200;

    huart2.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart2.Init.StopBits =
        UART_STOPBITS_1;

    huart2.Init.Parity =
        UART_PARITY_NONE;

    huart2.Init.Mode =
        UART_MODE_TX_RX;

    huart2.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart2.Init.OverSampling =
        UART_OVERSAMPLING_16;

    huart2.Init.OneBitSampling =
        UART_ONE_BIT_SAMPLE_DISABLE;

    huart2.Init.ClockPrescaler =
        UART_PRESCALER_DIV1;

    huart2.AdvancedInit.AdvFeatureInit =
        UART_ADVFEATURE_NO_INIT;


    if (HAL_UART_Init(&huart2)
        != HAL_OK)
    {
        Error_Handler();
    }
}


/* ============================================================
 * USART3 - DEBUG
 * ============================================================ */

static void MX_USART3_UART_Init(void)
{
    huart3.Instance =
        USART3;

    huart3.Init.BaudRate =
        115200;

    huart3.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart3.Init.StopBits =
        UART_STOPBITS_1;

    huart3.Init.Parity =
        UART_PARITY_NONE;

    huart3.Init.Mode =
        UART_MODE_TX_RX;

    huart3.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart3.Init.OverSampling =
        UART_OVERSAMPLING_16;

    huart3.Init.OneBitSampling =
        UART_ONE_BIT_SAMPLE_DISABLE;

    huart3.Init.ClockPrescaler =
        UART_PRESCALER_DIV1;

    huart3.AdvancedInit.AdvFeatureInit =
        UART_ADVFEATURE_NO_INIT;


    if (HAL_UART_Init(&huart3)
        != HAL_OK)
    {
        Error_Handler();
    }
}


/* ============================================================
 * I2C2
 *
 * PB10 = SCL
 * PB11 = SDA
 * ============================================================ */

static void MX_I2C2_Init(void)
{
    hi2c2.Instance =
        I2C2;

    hi2c2.Init.Timing =
        0x10707DBC;

    hi2c2.Init.OwnAddress1 =
        0;

    hi2c2.Init.AddressingMode =
        I2C_ADDRESSINGMODE_7BIT;

    hi2c2.Init.DualAddressMode =
        I2C_DUALADDRESS_DISABLE;

    hi2c2.Init.OwnAddress2 =
        0;

    hi2c2.Init.OwnAddress2Masks =
        I2C_OA2_NOMASK;

    hi2c2.Init.GeneralCallMode =
        I2C_GENERALCALL_DISABLE;

    hi2c2.Init.NoStretchMode =
        I2C_NOSTRETCH_DISABLE;


    if (HAL_I2C_Init(&hi2c2)
        != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_I2CEx_ConfigAnalogFilter(
            &hi2c2,
            I2C_ANALOGFILTER_ENABLE)
        != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_I2CEx_ConfigDigitalFilter(
            &hi2c2,
            0)
        != HAL_OK)
    {
        Error_Handler();
    }
}


/* ============================================================
 * GPIO
 * ============================================================ */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();


    /* --------------------------------------------------------
     * USART1
     *
     * PA9  = TX
     * PA10 = RX
     * -------------------------------------------------------- */

    GPIO_InitStruct.Pin =
        GPIO_PIN_9 |
        GPIO_PIN_10;

    GPIO_InitStruct.Mode =
        GPIO_MODE_AF_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_VERY_HIGH;

    GPIO_InitStruct.Alternate =
        GPIO_AF7_USART1;


    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );


    /* --------------------------------------------------------
     * USART2
     *
     * PA2 = TX
     * PA3 = RX
     * -------------------------------------------------------- */

    GPIO_InitStruct.Pin =
        GPIO_PIN_2 |
        GPIO_PIN_3;

    GPIO_InitStruct.Alternate =
        GPIO_AF7_USART2;


    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );


    /* --------------------------------------------------------
     * USART3
     *
     * PD8 = TX
     * PD9 = RX
     * -------------------------------------------------------- */

    GPIO_InitStruct.Pin =
        GPIO_PIN_8 |
        GPIO_PIN_9;

    GPIO_InitStruct.Alternate =
        GPIO_AF7_USART3;


    HAL_GPIO_Init(
        GPIOD,
        &GPIO_InitStruct
    );


    /* --------------------------------------------------------
     * I2C2
     *
     * PB10 = SCL
     * PB11 = SDA
     * -------------------------------------------------------- */

    GPIO_InitStruct.Pin =
        GPIO_PIN_10 |
        GPIO_PIN_11;

    GPIO_InitStruct.Mode =
        GPIO_MODE_AF_OD;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_VERY_HIGH;

    GPIO_InitStruct.Alternate =
        GPIO_AF4_I2C2;


    HAL_GPIO_Init(
        GPIOB,
        &GPIO_InitStruct
    );
}


/* ============================================================
 * ERROR HANDLER
 * ============================================================ */

void Error_Handler(void)
{
    Stop_Motors();

    __disable_irq();

    while (1)
    {
    }
}


#ifdef USE_FULL_ASSERT

void assert_failed(
        uint8_t *file,
        uint32_t line)
{
    (void)file;
    (void)line;
}

#endif
