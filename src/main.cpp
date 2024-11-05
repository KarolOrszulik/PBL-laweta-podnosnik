#include <Arduino.h>
#include "pins.h"
#include "constants.h"

#include "Motor.h"
#include "HX711.h"
#include "DFRobot_BMI160.h"

Motor motor1(MOTOR_1A, MOTOR_1B, ENCODER_1A, ENCODER_1B);
Motor motor2(MOTOR_2A, MOTOR_2B, ENCODER_2A, ENCODER_2B);

static HX711 hx711_1;
static HX711 hx711_2;

static DFRobot_BMI160 bmi160;
constexpr uint8_t BMI160_ADDR = 0x69;


void setup()
{
    // set up serial
    Serial.begin(115200);
    delay(1000);

    motor1.setPulsesPerMillimeter(MOTOR_CPR * THREAD_PITCH);
    motor2.setPulsesPerMillimeter(MOTOR_CPR * THREAD_PITCH);
    motor1.setPositionTolerance(0);
    motor2.setPositionTolerance(0);
    attachInterrupt(digitalPinToInterrupt(ENCODER_1A), []{ motor1.isr(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_2A), []{ motor2.isr(); }, RISING);


    // set up HX711
    hx711_1.begin(HX711_1DT, HX711_1SCK);
    hx711_2.begin(HX711_2DT, HX711_2SCK);

    // set up IMU
    if (bmi160.I2cInit(BMI160_ADDR) == BMI160_OK)
    {
        Serial.println("BMI160 found");
    }
    else
    {
        Serial.println("BMI160 not found");
        while(1); // hang program
    }

    Serial.println("Setup complete");
}

void motorEncoderDemo()
{
    Serial.println("Running motor/encoder demo");
    
    motor1.move(2);
    motor2.move(3);
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

void bmi160Demo()
{

    int16_t accelGyro[6];

    int ret = bmi160.getAccelGyroData(accelGyro);

    if (ret != 0)
    {
        Serial.println("Error getting BMI160 data");
        return;
    }

    Serial.println("Accel: " + String(accelGyro[0]) + " " + String(accelGyro[1]) + " " + String(accelGyro[2])
               + "\tGyro " + String(accelGyro[3]) + " " + String(accelGyro[4]) + " " + String(accelGyro[5]));

    delay(100);
}

void calculateMotorTransmission()
{
    constexpr int TACHOMETER_PIN = 4;

    pinMode(TACHOMETER_PIN, INPUT);

    bool lastState = digitalRead(TACHOMETER_PIN);
    int tachPulses = 0;

    digitalWrite(MOTOR_1A, HIGH);
    digitalWrite(MOTOR_1B, LOW);

    while (true)
    {
        // increment tachPulses on rising edge
        const bool state = digitalRead(TACHOMETER_PIN);
        if (state && !lastState)
        {
            tachPulses++;

            const float ratio = motor1.getPulses() / (float)tachPulses;
            Serial.print("Pulses: " + String(motor1.getPulses()));
            Serial.print("\tTach pulses: " + String(tachPulses));
            Serial.println("\tRatio: " + String(ratio));

            delay(400);
        }
        lastState = state;
    }
}

void loop()
{
    // calculateMotorTransmission();

    static enum
    {
        NONE,
        MOTOR_ENCODER,
        MOTOR_ENCODER_2,
        ENCODER,
        HX711,
        BMI160,

        UP,
        DOWN
    } state = NONE;


    if (Serial.available())
    {
        char c = Serial.read();
        switch (c)
        {
            case 'm':
                state = MOTOR_ENCODER;
                break;
            case 'M':
                state = MOTOR_ENCODER_2;
                break;
            case 'h':
                state = HX711;
                break;
            case 'b':
                state = BMI160;
                break;
            case 'e':
                state = ENCODER;
                break;
            case 'u':
                state = UP;
                break;
            case 'd':
                state = DOWN;
                break;
            default:
                state = NONE;
                break;
        }
    }

    switch (state)
    {
        case MOTOR_ENCODER:
            motorEncoderDemo();
            break;
        case MOTOR_ENCODER_2:
            Serial.println("Pulses before: " + String(motor2.getPulses()));
            motor2.move(1);
            while (!motor2.isInRange())
            {
                motor2.update();
            }
            delay(100);
            {
                int32_t target = motor2.getTargetPulses();
                int32_t pulses = motor2.getPulses();
                Serial.println("Target: " + String(target));
                Serial.println("Pulses after: " + String(pulses));
                Serial.println("Error: " + String(pulses - target));
            }
            delay(1000);
            break;

        case HX711:
            hx711Demo();
            break;
        case BMI160:
            bmi160Demo();
            break;
        case ENCODER:
            Serial.println("Motor 1: " + String(motor1.getPulses()));
            break;
        case UP:
            Serial.println("Moving up");
            motor1.move(1);
            while(!motor1.isInRange())
            {
                motor1.update();
            }
            state = NONE;
            break;
        case DOWN:
            Serial.println("Moving down");
            motor1.move(-1);
            while(!motor1.isInRange())
            {
                motor1.update();
            }
            state = NONE;
            break;
        default:
            break;
    }
}