#include <Arduino.h>
#include "pins.h"

#include "Motor.h"
#include "HX711.h"
// #include "DFRobot_BMI160.h"

Motor motor1(MOTOR_1A, MOTOR_1B, ENCODER_1A, ENCODER_1B);
Motor motor2(MOTOR_2A, MOTOR_2B, ENCODER_2A, ENCODER_2B);

static HX711 hx711_1;
static HX711 hx711_2;

// static DFRobot_BMI160 bmi160;
// constexpr uint8_t BMI160_ADDR = 0x68;


void setup()
{
    // set up serial
    Serial.begin(115200);
    delay(500);

    motor1.setPulsesPerMillimeter(695);
    motor2.setPulsesPerMillimeter(695);
    motor1.setPositionTolerance(0.05);
    motor2.setPositionTolerance(0.05);
    attachInterrupt(digitalPinToInterrupt(ENCODER_1A), []{ motor1.isr(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_2A), []{ motor2.isr(); }, RISING);


    // set up HX711
    hx711_1.begin(HX711_1DT, HX711_1SCK);
    hx711_2.begin(HX711_2DT, HX711_2SCK);

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
    
    motor1.move(2);
    motor2.move(-3);
    while (!motor1.isInRange() || !motor2.isInRange())
    {
        motor1.update();
        motor2.update();
    }

    Serial.println("Motors in range");
    delay(1000);

}

void hx711Demo()
{
    if (hx711_1.is_ready() && hx711_2.is_ready())
    {
        long reading1 = hx711_1.read();
        long reading2 = hx711_2.read();
        Serial.println("HX711 1: " + String(reading1) + "\tHX711 2: " + String(reading2));
    }
    delay(100);
}

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
    // motorEncoderDemo();
    hx711Demo();
    // bmi160Demo();

    delay(2000);
}