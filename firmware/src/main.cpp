#include <Arduino.h>
#include <Ping.h>
#include <Encoder.h>
#include "NewPing.h"
#include "platform.h"
#include "variables.h"
#include "functions.hpp"

const int maxDistance = 50;

NewPing sonarMiddle(ultrasound_trigger_pin[(int)UltraSoundSensor::Front],
                    ultrasound_echo_pin[(int)UltraSoundSensor::Front],
                    maxDistance);

unsigned int pingDelay = 100; // How frequently send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;      // Holds the next ping time.

// ##########
Encoder rightSide(ENCODER_REAR_RIGHT_1, ENCODER_REAR_RIGHT_2);
Encoder leftSide(ENCODER_REAR_LEFT_1, ENCODER_REAR_LEFT_2);

float YawCalibrationCenter = 90.0f;
float PitchCalibrationCenter = 58.0f;

const byte numChars = 64;
char receivedChars[numChars];
char tempChars[numChars]; // temporary array for use when parsing

dataPacket packet;

boolean newData = false;

float yawRequested = 0;
float pitchRequested = 0;

float yawErrorAccumulated = 0;
float pitchErrorAccumulated = 0;

float motor_yawRequested = 0;

boolean openRoad = true;
//============

long positionLeft = 0;
long positionRight = 0;

bool isTurning = false;
float optimalSpeed = 0;
float maxSpeed = 150.0;

//============

void echoCheck()
{ // Timer2 interrupt calls this function every 24uS where you can check the ping status.
    // Don't do anything here!
    if (sonarMiddle.check_timer())
    { // This is how you check to see if the ping was received.
        //Serial.print("Ping: ");
        //Serial.print(sonarMiddle.ping_result / US_ROUNDTRIP_CM); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
        //Serial.println("cm");
        if (sonarMiddle.ping_result / US_ROUNDTRIP_CM < 20) //||
        //(sonarLeft.ping_result / US_ROUNDTRIP_CM<15) ||
        //(sonarRight.ping_result / US_ROUNDTRIP_CM<15))
        {
            openRoad = false;
            Serial.print("Ping: ");
            Serial.print(sonarMiddle.ping_result / US_ROUNDTRIP_CM);
            Serial.println("cm");
        }
        else
        {
            openRoad = true;
        }
    }
    // Don't do anything here!
}

void recvWithStartEndMarkers()
{
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false)
    {
        rc = Serial.read();

        if (recvInProgress == true)
        {
            if (rc != endMarker)
            {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars)
                {
                    ndx = numChars - 1;
                }
            }
            else
            {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker)
        {
            recvInProgress = true;
        }
    }
}

//============

dataPacket parseData()
{ // split the data into its parts

    dataPacket tmpPacket;

    char *strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars, ",");   // get the first part - the string
    strcpy(tmpPacket.message, strtokIndx); // copy it to messageFromPC

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    tmpPacket.first = atof(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    tmpPacket.second = atof(strtokIndx);

    return tmpPacket;
}

void showParsedData(dataPacket packet)
{
    Serial.print("Message ");
    Serial.println(packet.message);
    Serial.print("Yaw ");
    Serial.println(packet.first);
    Serial.print("Pitch ");
    Serial.println(packet.second);
}

void setup()
{
    //Serial.begin(115200); // Starting Serial Terminal
    pingTimer = millis(); // Start now.
    initSerial(115200);
    Serial.println("This demo expects 3 pieces of data - text, a float and a floating point value");
    Serial.println("Enter data in this style <HelloWorld, 12.7, 24.7>  ");
    Serial.println();

    initMotors();

    calibrateServo(ServoSelector::Yaw, (int)YawCalibrationCenter);
    calibrateServo(ServoSelector::Pitch, (int)PitchCalibrationCenter);

    initServos();
    centerServos();

    initESP826();
    // initLED();
    Brake();
    delay(500);

    // Serial.println("Initalization ended");
}

//============

void loop()
{
    if (millis() >= pingTimer)
    {                                      // pingSpeed milliseconds since last ping, do another ping.
        pingTimer += pingDelay;            // Set the next ping time.
        sonarMiddle.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
        positionLeft = leftSide.read();
        positionRight = rightSide.read();
    }
    // parse input data
    recvWithStartEndMarkers();

    if (newData == true)
    {
        strcpy(tempChars, receivedChars);
        // this temporary copy is necessary to protect the original data
        //   because strtok() used in parseData() replaces the commas with \0
        packet = parseData();
        if (strcmp(packet.message, "servo") == 0)
        {
            if (isStopped == false)
            {

                yawRequested = packet.first;
                pitchRequested = packet.second;

                {
                    float yawError = -yawRequested;
                    float Kp = 25.0f;
                    float Ki = 4.0f;

                    float output = Kp * yawError + Ki * yawErrorAccumulated;
                    float tmp = YawCalibrationCenter + output;
                    if ((tmp > yawMin) && (tmp < yawMax))
                    {
                        yawErrorAccumulated += yawError;
                    }
                    moveServo(ServoSelector::Yaw, (int)(tmp));
                }
                {
                    float pitchError = -pitchRequested;
                    float Kp = 15.0f;
                    float Ki = 3.0f;

                    float output = Kp * pitchError + Ki * pitchErrorAccumulated;
                    pitchErrorAccumulated += pitchError;

                    // move servo
                    moveServo(ServoSelector::Pitch, (int)(PitchCalibrationCenter + output));
                }
            }
        }

        if (strcmp(packet.message, "follow") == 0)
        {
            if (openRoad)
            {
                motor_yawRequested = packet.first;

                float yawError = -motor_yawRequested;
                float Kp = 25.0f;
                float Ki = 4.0f;

                float output = Kp * yawError + Ki * yawErrorAccumulated;

                if (output > 10) //turn left
                {
                    SetPowerLevel(EngineSelector::Right, 150);
                    SetPowerLevel(EngineSelector::Left, -150);
                    isTurning = true;
                }
                else if (output < -10)
                {
                    SetPowerLevel(EngineSelector::Right, -150);
                    SetPowerLevel(EngineSelector::Left, 150);
                    isTurning = true;
                }
                else
                {
                    isTurning = false;
                }
            }
        }
        if (strcmp(packet.message, "stop") == 0)
        {
            isStopped = true;
        }

        if (strcmp(packet.message, "start") == 0)
        {
            isStopped = false;
        }

        if (strcmp(packet.message, "StartMotor") == 0)
        {
            if (openRoad && !isTurning)
            {
                int levelRequest = packet.first;
                SetPowerLevel(EngineSelector::Right, levelRequest);
                SetPowerLevel(EngineSelector::Left, levelRequest);
            }
        }

        if (strcmp(packet.message, "StopMotor") == 0)
        {
            SetPowerLevel(EngineSelector::Right, 0);
            SetPowerLevel(EngineSelector::Left, 0);
        }

        newData = false;
    }
}
