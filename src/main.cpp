#include <Arduino.h>
#include "pins.h"
#include "constants.h"
#include "HX711_calibration_values.h"

#include "DFRobot_BMI160.h"
#include "Adafruit_MCP23X17.h"

#include "Motor/L9110S_Motor.h"
#include "Encoder/Encoder.h"
#include "Pin/GPIO_Pin.h"
#include "Pin/MCP23017_Pin.h"
#include "Scale/HX711_Scale.h"
#include "Controller/SimpleController.h"
#include "Servo/Servo.h"


static Adafruit_MCP23X17 mcp;
static DFRobot_BMI160 bmi160;

static IScale* scales[4] = {};
static IMotor* motors[4] = {};
static IEncoder* encoders[4] = {};

static IServo* servo = nullptr;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    mcp.begin_I2C(0x20) ? Serial.println("MCP23017 found") : Serial.println("MCP23017 not found");
    bmi160.I2cInit(0x69) == BMI160_OK ? Serial.println("BMI160 found") : Serial.println("BMI160 not found");

    motors[0] = new L9110S_Motor(new MCP23017_Pin(&mcp, MOTOR_0A), new MCP23017_Pin(&mcp, MOTOR_0B));
    motors[1] = new L9110S_Motor(new MCP23017_Pin(&mcp, MOTOR_1A), new MCP23017_Pin(&mcp, MOTOR_1B));
    motors[2] = new L9110S_Motor(new MCP23017_Pin(&mcp, MOTOR_2A), new MCP23017_Pin(&mcp, MOTOR_2B));
    motors[3] = new L9110S_Motor(new MCP23017_Pin(&mcp, MOTOR_3A), new MCP23017_Pin(&mcp, MOTOR_3B));

    // for (auto motor : motors)
    //     motor->stop();

    encoders[0] = new Encoder(new GPIO_Pin(ENCODER_0A), new GPIO_Pin(ENCODER_0B));
    encoders[1] = new Encoder(new GPIO_Pin(ENCODER_1A), new GPIO_Pin(ENCODER_1B));
    encoders[2] = new Encoder(new GPIO_Pin(ENCODER_2A), new GPIO_Pin(ENCODER_2B));
    encoders[3] = new Encoder(new GPIO_Pin(ENCODER_3A), new GPIO_Pin(ENCODER_3B));

    attachInterrupt(digitalPinToInterrupt(encoders[0]->getTriggerPin()), [] { encoders[0]->isr(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(encoders[1]->getTriggerPin()), [] { encoders[1]->isr(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(encoders[2]->getTriggerPin()), [] { encoders[2]->isr(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(encoders[3]->getTriggerPin()), [] { encoders[3]->isr(); }, RISING);


    scales[0] = new HX711_Scale(new GPIO_Pin(HX711_0DT), new GPIO_Pin(HX711_0SCK));
    scales[1] = new HX711_Scale(new GPIO_Pin(HX711_1DT), new GPIO_Pin(HX711_1SCK));
    scales[2] = new HX711_Scale(new GPIO_Pin(HX711_2DT), new GPIO_Pin(HX711_2SCK));
    scales[3] = new HX711_Scale(new GPIO_Pin(HX711_3DT), new GPIO_Pin(HX711_3SCK));

    scales[0]->setScale(HX0_SCALE);
    scales[1]->setScale(HX1_SCALE);
    scales[2]->setScale(HX2_SCALE);
    scales[3]->setScale(HX3_SCALE);

    Serial.print("Taring scales");
    for (auto scale : scales)
    {
        Serial.print(".");
        scale->tare(2);
    }
    Serial.println();


    servo = new Servo(motors[0], encoders[3], new SimpleController(0.1f));
    servo->setPulsesPerMM(MOTOR_CPR / THREAD_PITCH);
    servo->setPositionLimits(0.f, 50.f);
    servo->enableUpdates();

    xTaskCreatePinnedToCore(
        [] (void*) {
            while (true)
            {
                servo->update();
            }
        },
        "Servo updates",
        10000,
        NULL,
        1,
        NULL,
        1
    );

    Serial.println("Setup complete");
}

void demoAllMotors()
{
    for (auto motor : motors)
    {
        motor->moveForward();
        delay(500);
        motor->stop();
        delay(1000);

        motor->moveBackward();
        delay(500);
        motor->stop();
        delay(1000);
    }
}

void demoAllEncoders()
{
    for (auto encoder : encoders)
    {
        Serial.print(encoder->getPulses());
        Serial.print("\t");
    }
    Serial.println();
    delay(100);
}

void demoAllScales()
{
    for (auto scale : scales)
    {
        Serial.print(scale->getWeight());
        Serial.print("\t");
    }
    Serial.println();
    delay(100);
}

void demoBMI160()
{
    int16_t accelGyro[6];
    bmi160.getAccelGyroData(accelGyro);
    Serial.println("Accel: " + String(accelGyro[0]) + "\t" + String(accelGyro[1]) + "\t" + String(accelGyro[2])
                + "\t\tGyro " + String(accelGyro[3]) + "\t" + String(accelGyro[4]) + "\t" + String(accelGyro[5]));
    delay(100);
}

void demoServo()
{
    static float distance = 1.f;

    servo->moveTargetPosition(distance);

    constexpr float POSITION_TOLERANCE = 0.1;
    while (abs(servo->getPositionError()) > POSITION_TOLERANCE); // wait

    distance *= -1.f;
}

void demoHome()
{
    servo->home(true);

    servo->setTargetPosition(1.f);
}

void loop()
{
    static enum : char
    {
        NONE = 'n',
        MOTORS = 'm',
        ENCODERS = 'e',
        SCALES = 's',
        BMI160 = 'b',
        SERVO = 'v',
        HOME = 'h',
        UP = 'u',
        DOWN = 'd'
    } state;

    if (Serial.available())
    {
        try
        {
            state = (decltype(state))Serial.read();
        } catch (...)
        {
            state = NONE;
        }
    }

    switch (state)
    {
        case NONE:
            break;
        case MOTORS:
            demoAllMotors();
            break;
        case ENCODERS:
            demoAllEncoders();
            break;
        case SCALES:
            demoAllScales();
            break;
        case BMI160:
            demoBMI160();
            break;
        case SERVO:
            demoServo();
            state = NONE;
            break;
        case HOME:
            demoHome();
            state = NONE;
            break;
        case UP:
            servo->moveTargetPosition(1.f);
            state = NONE;
            break;
        case DOWN:
            servo->moveTargetPosition(-1.f);
            state = NONE;
            break;
    }
}