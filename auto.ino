// this is for the microcontroller

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <SabertoothSimplified.h>
#include <Servo.h>
#include <std_msgs/Int64.h>
#include <array>
#include <string>
#include <sstream>
#include <vector>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <sbus.h>
#include "DHT.h"
#include <Adafruit_NeoPixel.h>

// Define the number of LEDs and the data pin
#define NUM_LEDS 60
#define DATA_PIN 13
#define DHTPIN 23     // Define the data pin
#define DHTTYPE DHT21 // DHT 21 (AM2301)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

DHT dht(DHTPIN, DHTTYPE);

#define ST_ADDRESS 128 // Sabertooth address (default: 128)
#define MOTOR1 1       // Sabertooth motor 1 //right
#define MOTOR2 2       // Sabertooth motor 2 //left

Servo boxservo1;
Servo boxservo2;
Servo pan;
Servo tilt;
int panpos1 = 45;
int tiltpos1 = 75;
int panpos = 0;
int tiltpos = 0;

SabertoothSimplified ST(Serial5);
SabertoothSimplified ST_ARM(Serial4);

bfs::SbusRx sbus_rx(&Serial2); /* SBUS object, writing SBUS */
bfs::SbusData data;
bool failsafe, failsafe_bck;

// Constants
const int STEPS_PER_REV = 1600; // Number of steps per revolution of the stepper motor
const float GEAR_RATIO = 50.0;  // Gear ratio of the stepper motor's gear box

// Function to calculate the wheel angle from the encoder count
float calculateWheelAngle(long count)
{
    float steps = static_cast<float>(count) / GEAR_RATIO; // Adjust the count with the gear ratio
    float angle = (steps / STEPS_PER_REV) * 360.0;        // Calculate the wheel angle in degrees
    return angle;
}

void fillStrip(uint32_t color);
void showRed();

void showGreen();

void showBlue();
void showYellow();

void showCyan();

void showMagenta();
void showWhite();

void turnOff();

MultiStepper stepperss;
MultiStepper steppers;

long thrs_f = -10000;
long p = 0;
long thrs_b = 10000;
long thrs_f_m2 = -43000;
long thrs_b_m2 = 43000;
// int interrupt[] = {0,0,0,0,3,0,2,0};

const int max_speed = 100;
int speed = 50;
int gear = 50;
int step = 1600;

// arm
// base 10 11
// left right 6 7 8 9
// 4 5 gripper
//
long base_angle = 0;
long base_step;
long thrs_base_f = 100000;
long thrs_base_b = -100000;
int n = 0;

long base_ratio = 1000;
int z = 0;

long base;
long b = 0;
long l = 0;
long q = 0;
long x = 0;
long m = 0;
long y = 0;

int chVal[30] = {};

AccelStepper stepperB(1, 34, 33); // pul // dir //base  chVal[9]   chVal[1 and 2] sabertooth
AccelStepper stepperL(1, 36, 35); // science L
AccelStepper stepperR(1, 38, 37); // science R // uporer ta

AccelStepper stepperG(1, 40, 39); // gripper// gripper
// arm

long positionss[] = {0, 0, 0, 0};

long a = 0;

const int tt = 50;

long positions[2];

const int t = 100;
int limit_pin = 0;

ros::NodeHandle nh;
unsigned long lastJoyStickCtrlTime = 0;
int mode = 1;
bool r_mode = false;
bool wifi = false;
// Callback function to handle incoming joystick messages
void box1(int panval)
{

    if (panval == 1000)
    {

        boxservo1.write(0);
    }

    else if (panval == 2000)
    {

        boxservo1.write(90);
    }
}

void box2(int tiltval)
{

    if (tiltval == 2000)
    {
        tiltpos++;

        boxservo2.write(90);
    }

    else if (tiltval == 1000)
    {

        boxservo2.write(0);
    }
}
void panServo(int panval)
{

    if (panval == 1000)
    {
        panpos1 -= 3;
        panpos1 = max(0, panpos1);
        pan.write(panpos1);
    }

    else if (panval == 2000)
    {
        panpos1 += 3;
        panpos1 = min(90, panpos1);

        pan.write(panpos1);
    }
}

void panServoRc(int panval)
{

    if (panval <= 1450)
    {
        panpos1 -= 1;
        panpos1 = max(0, panpos1);
        pan.write(panpos1);
    }

    else if (panval >= 1550)
    {
        panpos1 += 1;
        panpos1 = min(90, panpos1);

        pan.write(panpos1);
    }
}

void tiltServo(int tiltval)
{

    if (tiltval == 2000)
    {
        tiltpos1 += 3;
        tiltpos1 = min(180, tiltpos1);

        tilt.write(tiltpos1);
    }

    else if (tiltval == 1000)
    {
        tiltpos1 -= 3;
        tiltpos1 = max(0, tiltpos1);

        tilt.write(tiltpos1);
    }
}

void tiltServoRc(int tiltval)
{
    if (tiltval > 1520)
    {
        tiltpos1 += 1;
        tiltpos1 = min(180, tiltpos1);

        tilt.write(tiltpos1);
    }

    else if (tiltval < 1480)
    {
        tiltpos1 -= 1;
        tiltpos1 = max(0, tiltpos1);

        tilt.write(tiltpos1);
    }
}

void dropCallback(const std_msgs::String &msg)
{
    box1(2000);
    box2(2000);
}
void wifiCallback(const std_msgs::String &msg)
{
    if (msg.data[0] == 'c')
    {
        wifi = true;
    }
    else
    {
        wifi = false;
    }
}

const int relayPin = 6;

void relayCallback(const std_msgs::String &msg)
{
    if (msg.data[0] == 'o')
    {
        digitalWrite(relayPin, HIGH);
    }
    else
    {
        digitalWrite(relayPin, LOW);
    }
}

// Function to fill the strip with a given color

void colorCallback(const std_msgs::String &msg)
{
    if (msg.data[0] == 'w')
    {
        showWhite();
    }
    else if (msg.data[0] == 'b')
    {
        turnOff();
    }
    else if (msg.data[0] == 'r')
    {
        showRed();
    }
    else if (msg.data[0] == 'g')
    {
        showGreen();
    }
    else if (msg.data[0] == 'y')
    {
        showYellow();
    }
}
bool red_once_onJoystick = false;
bool greenOnce_joystick = false;
void joystickCallback(const std_msgs::String &msg)
{
    std::vector<int> arr(20, 1500);
    nh.spinOnce();
    if (msg.data[0] != '[')
    {
    }
    else
    {

        lastJoyStickCtrlTime = millis();

        std::string input = msg.data;
        std::istringstream iss(input.substr(1, input.length() - 2)); // Remove the enclosing brackets

        std::string token;
        while (std::getline(iss, token, ','))
        {
            char index_char = token[0];
            int index;
            if (isdigit(index_char))
            {
                index = index_char - '0';
            }
            else
            {
                index = 10 + (index_char - 'A'); // Convert letter to number for indices A-E
            }

            int value = std::stoi(token.substr(1)); // Convert the rest of the string to an int
            if (index >= 0 && index < arr.size())
            {
                arr[index] = value;
            }
        }
    }

    // Now arr contains the parsed values.
    int SPEED = 70;
    if (arr[8] == 2000)
        SPEED = 70;

    ST.motor(MOTOR2, constrain(map(arr[0], 1000, 2000, -SPEED, SPEED), -SPEED, SPEED));
    ST.motor(MOTOR1, constrain(map(arr[1], 1000, 2000, -SPEED, SPEED), -SPEED, SPEED));

    if (arr.size() >= 4)
    {
        limit_pin = digitalRead(14);

        if (limit_pin == 0)
        {
            ST_ARM.motor(1, constrain(map(arr[2], 1000, 2000, 100, -100), -100, 100));
            ST_ARM.motor(2, constrain(map(arr[3], 1000, 2000, 100, -100), -100, 100));
        }
        else
        {
            if (arr[2] > 1500)
            {
                ST_ARM.motor(1, 0);
            }
            else
            {
                ST_ARM.motor(1, constrain(map(0, 1000, 2000, 100, -100), -100, 100));
            }
            if (arr[3] > 1500)
            {
                ST_ARM.motor(2, 0);
            }
            else
            {
                ST_ARM.motor(2, constrain(map(0, 1000, 2000, 100, -100), -100, 100));
            }
        }
    }

    if (arr.size() >= 5)
    {
        chVal[9] = arr[4];
    }
    if (arr.size() >= 6)
    {
        chVal[10] = arr[5];
    }
    if (arr.size() >= 7)
    {
        chVal[5] = arr[6];
    }

    if (arr.size() >= 8)
    {
        if (arr[7] == 1500)
        {
            mode = 1;
        }
        else if (arr[7] == 1000)
        {
            mode = 2;
        }
        else if (arr[7] == 2000)
        {
            mode = 3;
        }
    }
    if (arr.size() >= 14)
    {
        box1(arr[13]);
    }
    if (arr.size() >= 15)
    {
        box2(arr[14]);
    }
    if (arr.size() >= 12)
    {
        panServo(arr[11]);
    }
    if (arr.size() >= 13)
    {
        tiltServo(arr[12]);
    }

    if (msg.data[0] != '[')
    {
        if (!red_once_onJoystick)
        {

            showRed();
            red_once_onJoystick = true;
            greenOnce_joystick = false;
        }
    }
    else
    {

        if (!greenOnce_joystick)
        {
            greenOnce_joystick = true;
            showGreen();
            red_once_onJoystick = false;
        }
    }

    nh.spinOnce();
    delay(0.1);
}

ros::Subscriber<std_msgs::String> joystickSub("joystick", &joystickCallback);
ros::Subscriber<std_msgs::String> wifiSub("wifi", &wifiCallback);
ros::Subscriber<std_msgs::String> dropSub("drop", &dropCallback);
ros::Subscriber<std_msgs::String> colorSub("color", &colorCallback);
std_msgs::String str_msg;
ros::Publisher pub1("dht", &str_msg);

ros::Subscriber<std_msgs::String> relaySub("relay", &relayCallback);
void bangladeshFlag()
{
    // GRG
    for (int i = 0; i < NUM_LEDS; i++)
    {
        if (i < 20)
        {
            strip.setPixelColor(i, strip.Color(0, 0, 255));
        }
        else if (i < 40)
        {
            strip.setPixelColor(i, strip.Color(255, 0, 0));
        }
        else
        {
            strip.setPixelColor(i, strip.Color(0, 0, 255));
        }
    }
    strip.show();
}
void setup()
{
    // FastLED.addLeds<NEOPIXEL, 13>(leds, NUM_LEDS);
    // FastLED.setBrightness(50);
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    strip.setBrightness(50);

    dht.begin();
    bangladeshFlag();
    pinMode(relayPin, OUTPUT);
    digitalWrite(relayPin, LOW);
    nh.getHardware()->setBaud(115200);
    nh.advertise(pub1);
    pinMode(14, INPUT_PULLUP);
    // for (int i = 0; i < 100; i++)
    // {
    //     digitalWrite(relayPin, HIGH);
    //     delay(2);
    //     digitalWrite(relayPin, LOW);
    //     delay(2);
    // }
    //  int timeout_ms = 100;

    // Serial.setTimeout(timeout_ms);

    Serial5.begin(9600);
    Serial4.begin(9600);
    Serial.begin(9600);
    // Serial.begin(115200);
    sbus_rx.Begin();

    boxservo1.attach(3);
    boxservo2.attach(2);
    boxservo1.write(panpos);
    boxservo2.write(tiltpos);
    // 11
    pan.attach(4);
    tilt.attach(5);
    pan.write(panpos1);
    tilt.write(tiltpos1);
    stepper_setup();
    positions[0] = 0;
    positions[1] = 0;

    nh.initNode();
    nh.subscribe(joystickSub);
    nh.subscribe(wifiSub);
    nh.subscribe(dropSub);
    nh.subscribe(colorSub);
    nh.subscribe(relaySub);
    // nh.subscribe(pxstateSub);
    // nh.subscribe(vstateSub);
    // nh.advertise(vals);
    // nh.advertise(pub);

    nh.spinOnce();
    delay(0.5);
}

void fillStrip(uint32_t color)
{
    for (int i = 0; i < NUM_LEDS; i++)
    {
        strip.setPixelColor(i, color);
    }
    strip.show();
}

void showRed()
{
    fillStrip(strip.Color(255, 0, 0));
}

void showGreen()
{
    fillStrip(strip.Color(0, 0, 255));
}

void showBlue()
{
    fillStrip(strip.Color(0, 255, 0));
}

void showYellow()
{
    fillStrip(strip.Color(255, 0, 255));
}

void showCyan()
{
    fillStrip(strip.Color(0, 255, 255));
}

void showMagenta()
{
    fillStrip(strip.Color(255, 0, 255));
}

void showWhite()
{
    fillStrip(strip.Color(255, 255, 255));
}

void turnOff()
{
    fillStrip(strip.Color(0, 0, 0));
}

void readAndPrintDHTValues()
{
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature))
    {
        str_msg.data = "0,0";
        if (nh.connected())
            pub1.publish(&str_msg);
        return;
    }
    str_msg.data = (String(temperature) + "," + String(humidity)).c_str();
    if (nh.connected())
        pub1.publish(&str_msg);
    delay(0.02);
}

void handleUtils()
{
    HANDLE_LIFTER();

    base_arm(); // ch 4
    gripper();  // ch

    nh.spinOnce();
    delay(0.01);
}

int DEADZONE = 100; // Define deadzone as per requirement
bool disarm = false;
void controlMotor(int x, int y)
{

    // Normalize the joystick values
    float x_norm = (x - 1500) / 500.0;
    float y_norm = (y - 1500) / 500.0;

    // Calculate motor speeds
    float left_speed_norm = y_norm + x_norm;
    float right_speed_norm = y_norm - x_norm;

    // Rescale to motor speed range
    float left_motor = 1500 + 500 * left_speed_norm;
    float right_motor = 1500 + 500 * right_speed_norm;

    // Ensure the values are within the valid range
    left_motor = constrain(left_motor, 1000, 2000);
    right_motor = constrain(right_motor, 1000, 2000);

    ST.motor(MOTOR2, constrain(map(left_motor, 1000, 2000, -70, 70), -70, 70));
    ST.motor(MOTOR1, constrain(map(right_motor, 1000, 2000, -70, 70), -70, 70));
}

void safety()
{

    // handleUtils();
    // failsafe_bck = tmp
}

void loop()
{
    // // readAndPrintDHTValues();
    // limit_pin = digitalRead(14);
    // // Serial.println(read_pin);
    // if (limit_pin == 1)
    // {
    //     safety();
    // }

    // if (!nh.connected() || !wifi)
    // {
    //     get_sbus();
    // }

    // box1(2000);
    // box2(2000);
    // delay(2000);
    // box1(1000);
    // box2(1000);
    // delay(2000);

    handleUtils();
    delay(0.01);
    // if (!nh.connected() && failsafe)  // serial off and failsafe on
    // {
    //   ST.motor(MOTOR1, 0);
    //   ST.motor(MOTOR2, 0);
    //   ST_ARM.motor(1, 0);
    //   ST_ARM.motor(2, 0);
    //   get_sbus();
    // } else if (!nh.connected() && !failsafe)  // serial off and failsafe off
    // {
    //   Serial.println("rc on");
    //   get_sbus();
    // } else if (nh.connected() && r_mode) {
    //   get_sbus();
    // } else if (nh.connected() && !wifi) {
    //   get_sbus();
    // }

    // Add additional code here if needed
}

void stepper_setup()
{
    steppers.addStepper(stepperL);
    steppers.addStepper(stepperR);
    stepperR.setMaxSpeed(1000);
    stepperR.setAcceleration(1000);
    stepperL.setMaxSpeed(1000);
    stepperL.setAcceleration(1000);
    stepperR.setSpeed(500);
    stepperL.setSpeed(500);

    stepperB.setMaxSpeed(900000);
    stepperB.setAcceleration(1000);
    stepperB.setSpeed(900000);

    stepperB.setMinPulseWidth(t);
    stepperL.setMinPulseWidth(t);
    stepperR.setMinPulseWidth(t);

    stepperG.setMaxSpeed(900000);
    stepperG.setAcceleration(1000);
    stepperG.setSpeed(900000);
    stepperG.setMinPulseWidth(t);
    nh.spinOnce();
    delay(0.01);
}

void print_stepper_pos()
{

    nh.spinOnce();
    delay(0.01);
}

void base_arm()
{

    if ((chVal[9] < 1400 || chVal[9] > 1600) && stepperB.currentPosition() <= thrs_base_f && stepperB.currentPosition() >= thrs_base_b)
    {
        int diff = abs(chVal[9] - 1500);
        int mp = map(diff, 0, 500, 50, 500);

        if (chVal[9] > 1600 && stepperB.currentPosition() <= thrs_base_f && chVal[9] < 2100)
        {
            // map chVal[9] from 0-500 to 50-500
            b += mp;
        }
        else if (chVal[9] < 1400 && stepperB.currentPosition() >= thrs_base_b && chVal[9] > 900)
            b -= mp;

        stepperB.moveTo(b);
        stepperB.setSpeed(5000);

        stepperB.runSpeedToPosition();
        // Serial.println(base_step);
    }

    else
    {
        b = 0;
        // positions[0]=0;
        // positions[1]=0;
    }
    if (stepperB.currentPosition() >= thrs_base_f)
        positions[0] = thrs_base_f;
    if (stepperB.currentPosition() <= thrs_base_b)
        positions[0] = thrs_base_b;
    nh.spinOnce();
    delay(0.01);
}

void fix()
{
    // chVal[5]=0
    positions[0] = 2400;
    positions[1] = 2400;

    steppers.moveTo(positions);

    steppers.runSpeedToPosition();

    stepperL.setCurrentPosition(0);
    stepperR.setCurrentPosition(0);
    nh.spinOnce();
    delay(0.01);
}

void HANDLE_LIFTER()
{

    if (mode == 1)
    {
        stepperR.setCurrentPosition(0);
        stepperL.setCurrentPosition(0);
        z = 0;
        if (chVal[10] > 1600 && chVal[10] < 2100)
        {
            z += 50;
        }
        else if (chVal[10] < 1400 && chVal[10] > 900)
        {
            {
                z -= 50;
            }
        }
        // else z =0;
        positions[0] = z;
        positions[1] = z;

        steppers.moveTo(positions);

        steppers.runSpeedToPosition();
        // stepperR.setCurrentPosition(0);
        //  stepperL.setCurrentPosition(0);
    }
    if (mode == 2)
    {

        stepperR.setCurrentPosition(0);
        stepperL.setCurrentPosition(0);
        x = 0;
        if (chVal[10] < 1450 && x > -12000)
            x -= 50;
        if (chVal[10] > 1550 && x < 12000)
            x += 50;

        positions[0] = x;
        positions[1] = -x;

        steppers.moveTo(positions);
        steppers.runSpeedToPosition();
        //  stepperR.setCurrentPosition(0);
        //  stepperL.setCurrentPosition(0);
    }
    nh.spinOnce();
    delay(0.01);
}
void gripper()
{
    stepperG.setCurrentPosition(0);
    m = 0;
    if (chVal[5] > 1600 && chVal[5] < 2100)
        m -= 500;
    else if (chVal[5] < 1400 && chVal[5] > 900)
        m += 500;
    // //else z =0;

    stepperG.moveTo(m);

    stepperG.moveTo(m);
    stepperG.setSpeed(14400);
    stepperG.runSpeedToPosition();
    nh.spinOnce();
    delay(0.01);
}

void checkFailSafe()
{
    if (sbus_rx.Read())
    {
        failsafe = data.failsafe;
    }
}
int b1 = 0;
int b2 = 0;
bool flag4_1 = false;
bool flag4_2 = false;
int last_success = 0;
int last_fail = 0;
bool failed = false;
bool success = false;