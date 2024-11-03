#include <Arduino.h>

// #include "HX711.h"
// #include "DFRobot_BMI160.h"

#include "pins.h"

static volatile int32_t encoder1Pos = 0;
static volatile int32_t encoder2Pos = 0;

// static HX711 hx711_1;
// static HX711 hx711_2;

// static DFRobot_BMI160 bmi160;
// constexpr uint8_t BMI160_ADDR = 0x68;

void IRAM_ATTR encoder1_isr()
{
    if (digitalRead(ENCODER_1B))
    {
        encoder1Pos++;
    }
    else
    {
        encoder1Pos--;
    }
}

void IRAM_ATTR encoder2_isr()
{
    if (digitalRead(ENCODER_2B))
    {
        encoder2Pos++;
    }
    else
    {
        encoder2Pos--;
    }
}

void setup()
{
    // set up serial
    Serial.begin(115200);
    delay(500);

    // set up motor/encoder pins
    pinMode(MOTOR_1A, OUTPUT);
    pinMode(MOTOR_1B, OUTPUT);
    pinMode(ENCODER_1A, INPUT);
    pinMode(ENCODER_1B, INPUT);

    pinMode(MOTOR_2A, OUTPUT);
    pinMode(MOTOR_2B, OUTPUT);
    pinMode(ENCODER_2A, INPUT);
    pinMode(ENCODER_2B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_1A), encoder1_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_2A), encoder2_isr, RISING);

    // set up HX711
    // hx711_1.begin(HX711_1DT, HX711_1SCK);
    // hx711_2.begin(HX711_2DT, HX711_2SCK);

    // set up IMU
    // if (bmi160.I2cInit(BMI160_ADDR) == BMI160_OK)
    // {
    //     Serial.println("BMI160 found");
    // }
    // else
    // {
    //     Serial.println("BMI160 not found");
    //     while(1); // hang program
    // }

    Serial.println("Setup complete");
}

void motorEncoderDemo()
{
    Serial.println("Running motor/encoder demo");
    Serial.println("Encoder 1 before: " + String(encoder1Pos));
    Serial.println("Encoder 2 before: " + String(encoder2Pos));

    // turn motor 1 forward
    digitalWrite(MOTOR_1A, LOW);
    digitalWrite(MOTOR_1B, HIGH);
    delay(1000);

    // turn motor 1 backward
    // digitalWrite(MOTOR_1A, HIGH);
    // digitalWrite(MOTOR_1B, LOW);
    // delay(500);

    // stop motor 1
    digitalWrite(MOTOR_1A, LOW);
    digitalWrite(MOTOR_1B, LOW);

    // turn motor 2 forward
    digitalWrite(MOTOR_2A, LOW);
    digitalWrite(MOTOR_2B, HIGH);
    delay(1000);

    // turn motor 2 backward
    // digitalWrite(MOTOR_2A, HIGH);
    // digitalWrite(MOTOR_2B, LOW);
    // delay(500);

    // stop motor 2
    digitalWrite(MOTOR_2A, LOW);
    digitalWrite(MOTOR_2B, LOW);

    Serial.println("Encoder 1 after: " + String(encoder1Pos));
    Serial.println("Encoder 2 after: " + String(encoder2Pos));
}

// void hx711Demo()
// {
//     constexpr uint32_t DEMO_TIME = 5000;
//     uint32_t timeStart = millis();

//     Serial.println("Running HX711 demo");
//     while (millis() - timeStart < DEMO_TIME)
//     {
//         if (hx711_1.is_ready())
//         {
//             Serial.println("Reading HX711 1: " + String(hx711_1.read()));
//         }
//         if (hx711_2.is_ready())
//         {
//             Serial.println("Reading HX711 2: " + String(hx711_2.read()));
//         }
//         delay(500);
//     }
// }

// void bmi160Demo()
// {
//     constexpr uint32_t DEMO_TIME = 5000;
//     uint32_t timeStart = millis();

//     Serial.println("Running BMI160 demo");
//     while (millis() - timeStart < DEMO_TIME)
//     {
//         int16_t accelGyro[6];

//         int ret = bmi160.getAccelGyroData(accelGyro);

//         if (ret != 0)
//         {
//             Serial.println("Error getting BMI160 data");
//             return;
//         }

//         Serial.println("Accel X: " + String(accelGyro[0]));
//         Serial.println("Accel Y: " + String(accelGyro[1]));
//         Serial.println("Accel Z: " + String(accelGyro[2]));
//         Serial.println("Gyro X: " + String(accelGyro[3]));
//         Serial.println("Gyro Y: " + String(accelGyro[4]));
//         Serial.println("Gyro Z: " + String(accelGyro[5]));
//     }
// }

void loop()
{
    Serial.println("Running demo");
    motorEncoderDemo();
    // hx711Demo();
    // bmi160Demo();

    delay(1000);
}