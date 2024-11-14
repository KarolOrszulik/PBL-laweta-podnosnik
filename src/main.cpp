// #include <Arduino.h>

// #include <WiFi.h>
// #include <PubSubClient.h>

// #include "pins.h"
// #include "constants.h"

// #include "Servo.h"
// #include "HX711.h"
// #include "Scale.h"
// #include "DFRobot_BMI160.h"

// static Servo motor1(MOTOR_1A, MOTOR_1B, ENCODER_1A, ENCODER_1B);
// static Servo motor2(MOTOR_2A, MOTOR_2B, ENCODER_2A, ENCODER_2B);

// static HX711 hx711_1;
// static HX711 hx711_2;

// static DFRobot_BMI160 bmi160;
// constexpr uint8_t BMI160_ADDR = 0x69;

// const char* const WIFI_SSID = "pbl804";
// const char* const WIFI_PASSWD = "pbl123456789";

// const char* const MQTT_SERVER = "192.168.50.210";

// static WiFiClient wifiClient;
// static PubSubClient mqttClient(wifiClient);

// void setupWiFi()
// {
//     WiFi.begin(WIFI_SSID, WIFI_PASSWD);
//     while (WiFi.status() != WL_CONNECTED)
//     {
//         delay(500);
//         Serial.println("Connecting to WiFi...");
//     }
//     Serial.println("Connected to WiFi with local IP: " + WiFi.localIP().toString());

//     mqttClient.setServer(MQTT_SERVER, 1883);
//     mqttClient.setCallback([](char* topic, uint8_t* payload, unsigned int length) {
//         Serial.print("Message arrived [");
//         Serial.print(topic);
//         Serial.print("] ");
//         for (int i = 0; i < length; i++)
//         {
//             Serial.print((char)payload[i]);
//         }
//         Serial.println();
//     });
// }

// void reconnectMQTT()
// {
//     while (!mqttClient.connected())
//     {
//         Serial.print("Attempting MQTT connection...");
//         if (mqttClient.connect("ESP32-Podnosnik"))
//         {
//             Serial.println("connected");
//             mqttClient.subscribe("podnosnik/test");
//         }
//         else
//         {
//             Serial.print("failed, rc=");
//             Serial.print(mqttClient.state());
//             Serial.println(" try again in 3 seconds");
//             delay(3000);
//         }
//     }
// }

// void setup()
// {
//     // set up serial
//     Serial.begin(115200);
//     delay(1000);

//     setupWiFi();

//     motor1.setPulsesPerMillimeter(MOTOR_CPR / THREAD_PITCH);
//     motor2.setPulsesPerMillimeter(MOTOR_CPR / THREAD_PITCH);
//     motor1.setPositionTolerance(0);
//     motor2.setPositionTolerance(0);
//     attachInterrupt(digitalPinToInterrupt(ENCODER_1A), []{ motor1.isr(); }, RISING);
//     attachInterrupt(digitalPinToInterrupt(ENCODER_2A), []{ motor2.isr(); }, RISING);

//     xTaskCreatePinnedToCore(
//         [] (void*) {
//             hw_timer_t* timer = timerBegin(0, 80, true);
//             timerAttachInterrupt(timer, [] { 
//                 motor1.update();
//                 motor2.update();
//             }, true);
//             timerAlarmWrite(timer, 10000, true);
//             timerAlarmEnable(timer);
//             while(true) { delay(1000); }
//         },
//         "Servo updates",    // task name
//         10000,              // stack size
//         NULL,               // task argument
//         1,                  // task priority
//         NULL,               // task handle
//         1                   // core to run task on
//     );


//     // set up HX711
//     hx711_1.begin(HX711_1DT, HX711_1SCK);
//     hx711_2.begin(HX711_2DT, HX711_2SCK);

//     // set up IMU
//     if (bmi160.I2cInit(BMI160_ADDR) == BMI160_OK)
//     {
//         Serial.println("BMI160 found");
//     }
//     else
//     {
//         Serial.println("BMI160 not found");
//         // while(1); // hang program
//     }

//     Serial.println("Setup complete");
// }

// void motorEncoderDemo()
// {
//     Serial.println("Running motor/encoder demo");
    
//     motor1.changeTargetPosition(2);
//     motor2.changeTargetPosition(3);
//     while (!motor1.isInRange() || !motor2.isInRange())
//     {
//         motor1.update();
//         motor2.update();
//     }

//     Serial.println("Motors in range");
//     delay(1000);
// }

// void hx711Demo()
// {
//     if (hx711_1.is_ready() && hx711_2.is_ready())
//     {
//         long reading1 = hx711_1.read();
//         long reading2 = hx711_2.read();
//         Serial.println("HX711 1: " + String(reading1) + "\tHX711 2: " + String(reading2));
//     }
//     delay(100);
// }

// void bmi160Demo()
// {

//     int16_t accelGyro[6];

//     int ret = bmi160.getAccelGyroData(accelGyro);

//     if (ret != 0)
//     {
//         Serial.println("Error getting BMI160 data");
//         return;
//     }

//     Serial.println("Accel: " + String(accelGyro[0]) + " " + String(accelGyro[1]) + " " + String(accelGyro[2])
//                + "\tGyro " + String(accelGyro[3]) + " " + String(accelGyro[4]) + " " + String(accelGyro[5]));

//     delay(100);
// }

// void calculateMotorTransmission()
// {
//     constexpr int TACHOMETER_PIN = 4;

//     pinMode(TACHOMETER_PIN, INPUT);

//     bool lastState = digitalRead(TACHOMETER_PIN);
//     int tachPulses = 0;

//     digitalWrite(MOTOR_1A, HIGH);
//     digitalWrite(MOTOR_1B, LOW);

//     while (true)
//     {
//         // increment tachPulses on rising edge
//         const bool state = digitalRead(TACHOMETER_PIN);
//         if (state && !lastState)
//         {
//             tachPulses++;

//             const float ratio = motor1.getPulses() / (float)tachPulses;
//             Serial.print("Pulses: " + String(motor1.getPulses()));
//             Serial.print("\tTach pulses: " + String(tachPulses));
//             Serial.println("\tRatio: " + String(ratio));

//             delay(400);
//         }
//         lastState = state;
//     }
// }

// void scaleDemo()
// {
//     Scale scale(HX711_2DT, HX711_2SCK);

//     scale.calibrate(54.0f);

//     while(true)
//     {
//         Serial.println("Weight: " + String(scale.getWeight()));
//         delay(250);
//     }

// }

// void hxDataGathering()
// {
//     while (true)
//     {
//         Serial.println(hx711_2.read_average(20));

//         while (!Serial.available())
//             ;
//         while (Serial.available())
//             Serial.read();
//     }
// }

// void loop()
// {
//     if (!mqttClient.connected())
//         reconnectMQTT();
    
//     mqttClient.loop();

//     static enum : char
//     {
//         NONE = 'n',

//         MOTOR_ENCODER = 'm',
//         MOTOR_ENCODER_2 = 'M',
//         CALULATE_TRANSMISSION = 'c',
//         HOME_MOTOR = ',',
//         SEQUENCE = '.',

//         ENCODER = 'e',

//         HX711 = 'h',
//         HX711_DATA_GATHERING = 'H',
//         SCALE = 's',

//         BMI160 = 'b',

//         UP = 'u',
//         DOWN = 'd'
//     } state = NONE;


//     if (Serial.available())
//     {
//         char c = Serial.read();
//         try
//         {
//             state = static_cast<decltype(state)>(c);
//         } catch (...)
//         {
//             state = NONE;
//         }
//     }

//     switch (state)
//     {
//         case NONE:
//             break;
    
//         case MOTOR_ENCODER:
//             motorEncoderDemo();
//             break;

//         case MOTOR_ENCODER_2:
//             Serial.println("Pulses before: " + String(motor2.getPulses()));
//             motor2.changeTargetPosition(1);
//             while (!motor2.isInRange())
//                 motor2.update();
//             delay(100);
//             {
//                 int32_t target = motor2.getTargetPulses();
//                 int32_t pulses = motor2.getPulses();
//                 Serial.println("Target: " + String(target));
//                 Serial.println("Pulses after: " + String(pulses));
//                 Serial.println("Error: " + String(pulses - target));
//             }
//             delay(1000);
//             break;

//         case HX711:
//             hx711Demo();
//             break;

//         case BMI160:
//             bmi160Demo();
//             break;

//         case ENCODER:
//             Serial.println("Encoder 1: " + String(motor1.getPulses()));
//             break;

//         case UP:
//             Serial.print("Moving up...");
//             motor1.changeTargetPosition(1.f);
//             while(!motor1.isInRange())
//             {
//                 //motor1.update();
//                 Serial.print(".");
//                 delay(250);
//             }
//             state = NONE;
//             Serial.println("Done.");
//             break;

//         case DOWN:
//             Serial.print("Moving down...");
//             motor1.changeTargetPosition(-1.f);
//             while(!motor1.isInRange())
//             {
//                 //motor1.update();
//                 Serial.print(".");
//                 delay(250);
//             }
//             state = NONE;
//             Serial.println("Done.");
//             break;

//         case SCALE:
//             scaleDemo();
//             break;

//         case HX711_DATA_GATHERING:
//             hxDataGathering();
//             break;

//         case CALULATE_TRANSMISSION:
//             calculateMotorTransmission();
//             break;
        
//         case HOME_MOTOR:
//             motor1.home();
//             state = NONE;
//             break;
        
//         case SEQUENCE:
//             motor1.home();

//             Serial.println("Servo 1 position: " + String(motor1.getPosition()) + "mm");
//             Serial.println("Setting motor 1 target position to 10mm");
//             motor1.setTargetPosition(10);

//             while (!motor1.isInRange())
//                 motor1.update();

//             delay(1000);

//             Serial.println("Moving motor 1 by 10mm");
//             motor1.changeTargetPosition(10);

//             while (!motor1.isInRange())
//                 motor1.update();
            
//             Serial.println("Servo 1 position: " + String(motor1.getPosition()) + "mm");

//             state = NONE;
//     }
// }

#include <Arduino.h>
#include "pins.h"

#include "DFRobot_BMI160.h"

#include "Adafruit_MCP23X17.h"

#include "Motor/L9110S_Motor.h"
#include "Encoder/Encoder.h"
#include "Pin/GPIO_Pin.h"
#include "Pin/MCP23017_Pin.h"
#include "Scale/HX711_Scale.h"

Adafruit_MCP23X17 mcp;

static IMotor* motor1 = new L9110S_Motor(new MCP23017_Pin(&mcp, 0), new MCP23017_Pin(&mcp, 1));
static IEncoder* encoder1 = new Encoder(new GPIO_Pin(ENCODER_1A), new GPIO_Pin(ENCODER_1B));
static IScale* scale1 = new HX711_Scale(new GPIO_Pin(HX711_1DT), new GPIO_Pin(HX711_1SCK));

void setup()
{
    Serial.begin(115200);
    delay(1000);

    mcp.begin_I2C(0x20);
    attachInterrupt(digitalPinToInterrupt(encoder1->getTriggerPin()), [] { encoder1->isr(); }, RISING);

    xTaskCreatePinnedToCore(
        [](void*) {
            while (true)
            {
                Serial.print("Encoder: ");
                Serial.println(encoder1->getPulses());
                delay(250);
            }
        },
        "Encoder report",
        10000,
        NULL,
        1,
        NULL,
        1
    );
}

void loop()
{
    motor1->moveForward();
    delay(1000);
    motor1->stop();
    delay(1000);

    motor1->moveBackward();
    delay(1000);
    motor1->stop();
    delay(1000);
}