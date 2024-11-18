#include <Arduino.h>
#include "pins.h"
#include "constants.h"
#include "HX711_calibration_values.h"
#include "secret_wifi.h"

#include <DFRobot_BMI160.h>
#include <Adafruit_MCP23X17.h>

#include <WiFi.h>
#include <PubSubClient.h>

#include "Motor/L9110S_Motor.h"
#include "Encoder/Encoder.h"
#include "Pin/GPIO_Pin.h"
#include "Pin/MCP23017_Pin.h"
#include "Scale/HX711_Scale.h"
#include "Controller/SimpleController.h"
#include "Servo/Servo.h"
#include "StopSignal/StopSignal.h"

#include <unordered_map>

#define NUM_AXIS 4

static Adafruit_MCP23X17 mcp;
static DFRobot_BMI160 bmi160;

static IScale* scales[NUM_AXIS] = {};
static IMotor* motors[NUM_AXIS] = {};
static IEncoder* encoders[NUM_AXIS] = {};
static IServo* servos[NUM_AXIS] = {};

#define MQTT_SERVER "192.168.100.6" // thinkpad

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

enum class State {
        NONE,
        ENCODERS,
        SCALES,
        BMI160,
        SERVO,
        HOME,
        UP,
        DOWN,
        STOP,
};
State state = State::NONE;

static std::unordered_map<char, State> charToState = {
    {'e', State::ENCODERS},
    {'w', State::SCALES},
    {'b', State::BMI160},
    {'s', State::SERVO},
    {'h', State::HOME},
    {'u', State::UP},
    {'d', State::DOWN},
    {'t', State::STOP},  
};


void setupWiFi()
{
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi " + String(WIFI_SSID));
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println("WiFi connected with local IP " + WiFi.localIP().toString());
}

void ensureMQTTConnection()
{
    while (!mqttClient.connected())
    {
        Serial.print("Attempting MQTT connection...");
        if (mqttClient.connect("ESP32-Podnosnik"))
        {
            Serial.println("connected");
            mqttClient.subscribe("podnosnik/command");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void setup()
{
    // set up serial
    Serial.begin(115200);
    delay(1000);

    // set up WiFi
    setupWiFi();

    // set up MQTT client
    mqttClient.setServer(MQTT_SERVER, 1883);
    mqttClient.setCallback([] (char* topic, byte* payload, unsigned int length)  {  
        Serial.println("MQTT message [" + String(topic) + "] " + String(payload, length));
        try
        {
            state = charToState[payload[0]];
        }
        catch(const std::exception& e)
        {
            state = State::NONE;
        }
    });

    // set up I2C
    Wire.begin(21, 22);
    Wire1.begin(12, 13);

    // set up I2C devices - BMI160 and MCP23017
    bmi160.softReset() == BMI160_OK ? Serial.println("BMI160 reset success") : Serial.println("BMI160 reset failed");
    bmi160.I2cInit(0x69) == BMI160_OK ? Serial.println("BMI160 init success") : Serial.println("BMI160 init failed");
    mcp.begin_I2C(0x20, &Wire1) ? Serial.println("MCP23017 found") : Serial.println("MCP23017 not found");

    // set up motors, encoders, scales and servos
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

    // set up interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(encoders[0]->getTriggerPin()), []
                    { encoders[0]->isr(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(encoders[1]->getTriggerPin()), []
                    { encoders[1]->isr(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(encoders[2]->getTriggerPin()), []
                    { encoders[2]->isr(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(encoders[3]->getTriggerPin()), []
                    { encoders[3]->isr(); }, RISING);

    // set up task to update servos
    xTaskCreatePinnedToCore(
        [] (void*)
        {
            while (true)
            {
                for (auto servo : servos)
                    servo->update();
                vTaskDelay(20 / portTICK_PERIOD_MS);
            }
        },
        "Servo updates", 10000, NULL, 1, NULL, 1);
    
    // set up task to report weights on scales
    xTaskCreatePinnedToCore(
        [] (void*)
        {
            while (true)
            {
                String str = "Weight on scales [g]: ";
                for (auto scale : scales)
                    str += String(scale->getWeight()) + " ";
                
                mqttClient.publish("podnosnik/waga", str.c_str());

                vTaskDelay(2000 / portTICK_PERIOD_MS);
            }
        },
        "Weight report MQTT", 10000, NULL, 1, NULL, 1);

    // clear stop signal
    StopSignal::instance()->clearStop();

    // print setup complete
    Serial.println("Setup complete");
}


#pragma region Demos

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
    Serial.print("Weights on scales [g]: ");
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
    
    state = State::NONE;
}

void demoHome()
{
    servos[0]->home(true);
    servos[0]->setTargetPosition(1.f);
    state = State::NONE;
}

void demoUp()
{
    servos[0]->moveTargetPosition(1.f);
    state = State::NONE;

}

void demoDown()
{
    servos[0]->moveTargetPosition(-1.f);
    state = State::NONE;
}

void demoStop()
{
    StopSignal::instance()->stop();
    delay(1000);
    StopSignal::instance()->clearStop();
    state = State::NONE;
}

#pragma endregion


static std::unordered_map<State, std::function<void()>> stateToFunction = {
    {State::ENCODERS, demoAllEncoders},
    {State::SCALES, demoAllScales},
    {State::BMI160, demoBMI160},
    {State::SERVO, demoServo},
    {State::HOME, demoHome},
    {State::UP, demoUp},
    {State::DOWN, demoDown},
    {State::STOP, demoStop},
};

void loop()
{
    ensureMQTTConnection();
    mqttClient.loop();

    if (Serial.available())
    {
        try
        {
            state = charToState[Serial.read()];
        }
        catch(const std::exception& e)
        {
            state = State::NONE;
        }
    }

    if (state != State::NONE)
        stateToFunction[state]();
}