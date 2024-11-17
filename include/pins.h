#pragma once

#pragma region Motors

    // ALL MOTORS ARE ON THE MCP23017
    #define MOTOR_0A    0
    #define MOTOR_0B    1

    #define MOTOR_1A    2
    #define MOTOR_1B    3

    #define MOTOR_2A    4
    #define MOTOR_2B    5

    #define MOTOR_3A    6
    #define MOTOR_3B    7

#pragma endregion


#pragma region Encoders
    
        #define ENCODER_0A  14
        #define ENCODER_0B  27
    
        #define ENCODER_1A  25
        #define ENCODER_1B  26
    
        #define ENCODER_2A  32
        #define ENCODER_2B  33
    
        #define ENCODER_3A  34
        #define ENCODER_3B  35

#pragma endregion


#pragma region HX711

    #define HX711_0SCK  23
    #define HX711_0DT   16

    #define HX711_1SCK  23
    #define HX711_1DT   17

    #define HX711_2SCK  23
    #define HX711_2DT   18

    #define HX711_3SCK  23
    #define HX711_3DT   19

#pragma endregion


#pragma region IMU
    
        #define IMU_SDA 21
        #define IMU_SCL 22

#pragma endregion


#pragma region LordForgiveMeForWhatImAboutToDo

constexpr struct
{
    int pinA;
    int pinB;
} MOTOR_PINS[] = {
    {0, 1},
    {2, 3},
    {4, 5},
    {6, 7}
};

constexpr struct
{
    int pinTrig;
    int pinDir;
} ENCODER_PINS[] = {
    {14, 27},
    {25, 26},
    {32, 33},
    {34, 35}
};

constexpr struct
{
    int pinSCK;
    int pinDT;
} HX711_PINS[] = {
    {23, 16},
    {23, 17},
    {23, 18},
    {23, 19}
};

constexpr struct 
{
    int sda;
    int scl;
} IMU_PINS = {
    21, 22
};

#pragma endregion

