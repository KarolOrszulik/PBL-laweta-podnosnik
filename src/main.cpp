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
#include "StopSignal/StopSignal.h"

#define NUM_AXIS 4

static Adafruit_MCP23X17 mcp;
static DFRobot_BMI160 bmi160;

static IScale* scales[NUM_AXIS] = {};
static IMotor* motors[NUM_AXIS] = {};
static IEncoder* encoders[NUM_AXIS] = {};
static IServo* servos[NUM_AXIS] = {};

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Wire.begin(21, 22);
    Wire1.begin(12, 13);

    bmi160.softReset() == BMI160_OK ? Serial.println("BMI160 reset success") : Serial.println("BMI160 reset failed");
    bmi160.I2cInit(0x69) == BMI160_OK ? Serial.println("BMI160 init success") : Serial.println("BMI160 init failed");
    mcp.begin_I2C(0x20, &Wire1) ? Serial.println("MCP23017 found") : Serial.println("MCP23017 not found");

    for (int i = 0; i < NUM_AXIS; i++)
    {
        motors[i] = new L9110S_Motor(new MCP23017_Pin(&mcp, MOTOR_PINS[i].pinA), new MCP23017_Pin(&mcp, MOTOR_PINS[i].pinB));
        encoders[i] = new Encoder(new GPIO_Pin(ENCODER_PINS[i].pinTrig), new GPIO_Pin(ENCODER_PINS[i].pinDir));

        scales[i] = new HX711_Scale(new GPIO_Pin(HX711_PINS[i].pinDT), new GPIO_Pin(HX711_PINS[i].pinSCK));
        scales[i]->tare(2);
        scales[i]->setScale(HX711_CALIBRATION_VALUES[i].scale);

        servos[i] = new Servo(motors[i], encoders[i], new SimpleController(0.1f));
        servos[i]->setPulsesPerMM(MOTOR_CPR / THREAD_PITCH);
        servos[i]->setPositionLimits(0.f, 50.f);
        servos[i]->enableUpdates();
    }

    attachInterrupt(digitalPinToInterrupt(encoders[0]->getTriggerPin()), []
                    { encoders[0]->isr(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(encoders[1]->getTriggerPin()), []
                    { encoders[1]->isr(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(encoders[2]->getTriggerPin()), []
                    { encoders[2]->isr(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(encoders[3]->getTriggerPin()), []
                    { encoders[3]->isr(); }, RISING);

    xTaskCreatePinnedToCore(
        [] (void*)
        {
            while (true)
            {
                for (auto servo : servos)
                    servo->update();
                delay(10);
            }
        },
        "Servo updates",
        10000,
        NULL,
        1,
        NULL,
        1);


    StopSignal::instance()->clearStop();

    Serial.println("Setup complete");
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
    int16_t gyroAccel[6];
    const int result = bmi160.getAccelGyroData(gyroAccel);
    if (result == 0)
    {
        Serial.println(
              "Gyro [rad/s]: "
            + String(gyroAccel[0] * 3.14159f / 180.f) + "\t"
            + String(gyroAccel[1] * 3.14159f / 180.f) + "\t"
            + String(gyroAccel[2] * 3.14159f / 180.f) + "\t\t"
            + "Accel [g]: "
            + String(gyroAccel[3] / 16384.f) + "\t"
            + String(gyroAccel[4] / 16384.f) + "\t"
            + String(gyroAccel[5] / 16384.f));
    }
    else
    {
        Serial.println("BMI160 data read failed");
    }
    delay(100);
}

void demoServo()
{
    static float distance = 1.f;

    servos[0]->moveTargetPosition(distance);

    constexpr float POSITION_TOLERANCE = 0.1;
    while (abs(servos[0]->getPositionError()) > POSITION_TOLERANCE)
        ; // wait

    distance *= -1.f;
}

void demoHome()
{
    servos[0]->home(true);
    servos[0]->setTargetPosition(1.f);
}

void loop()
{
    static enum : char {
        NONE = 'n',
        ENCODERS = 'e',
        SCALES = 's',
        BMI160 = 'b',
        SERVO = 'v',
        HOME = 'h',
        UP = 'u',
        DOWN = 'd',
        STOP = 'S',
    } state;

    if (Serial.available())
    {
        try
        {
            state = (decltype(state))Serial.read();
        }
        catch (...)
        {
            state = NONE;
        }
    }

    switch (state)
    {
    case NONE:
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
        servos[0]->moveTargetPosition(1.f);
        state = NONE;
        break;
    case DOWN:
        servos[0]->moveTargetPosition(-1.f);
        state = NONE;
        break;
    case STOP:
        StopSignal::instance()->stop();
        delay(1000);
        StopSignal::instance()->clearStop();
        state = NONE;
        break;
    }
}