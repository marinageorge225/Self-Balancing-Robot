#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "SimpleKalmanFilter.h"

MPU6050 mpu;

const int enA = 9;
const int in1 = 13;
const int in2 = 12;
const int enB = 6;
const int in3 = 11;
const int in4 = 10;
const int motorSpeed = 150;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

double setpoint = 200;
double Kp = 21;
double Kd = 0.8;
double Ki = 140;

double input, output;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double processNoise = 0.8;      // Adjust based on system behavior
double measurementNoise = 1.2;  // Adjust based on sensor characteristics
double initialEstimate = 0.02;  // Adjust based on expected initial deviation

SimpleKalmanFilter kalmanFilter(processNoise, measurementNoise, initialEstimate);  // Adjust these parameters based on your system

volatile bool mpuInterrupt = false;

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Serial.begin(115200);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(-3629);
    mpu.setYGyroOffset(-9851);
    mpu.setZGyroOffset(14321);
    mpu.setZAccelOffset(32767);

    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for the first interrupt..."));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();

        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);

    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in1, OUTPUT);

    analogWrite(in4, LOW);
    analogWrite(in3, LOW);
    analogWrite(in2, LOW);
    analogWrite(in1, LOW);
    analogWrite(enA, motorSpeed);
    analogWrite(enB, motorSpeed);
}

void loop() {
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {
        pid.Compute();

        Serial.print(input);
        Serial.print(" =>");
        Serial.println(output);

        if (input > 176 && input < 225) {
            if (output > 0)
                Forward();
            else if (output < 0)
                Reverse();
        } else
            Stop();
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);

        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Apply Kalman filter to sensor readings
        input = kalmanFilter.updateEstimate(ypr[1] * 180 / M_PI + 180);
    }
}

void Forward() {
    analogWrite(in4, 150);
    analogWrite(in3, 0);
    analogWrite(in2, 150);
    analogWrite(in1, 0);
    Serial.print("F");
}

void Reverse() {
    analogWrite(in4, 0);
    analogWrite(in3, 80 + output);
    analogWrite(in2, 0);
    analogWrite(in1, 100 + output);
    Serial.print("R");
}

void Stop() {
    analogWrite(in4, 0);
    analogWrite(in3, 0);
    analogWrite(in2, 0);
    analogWrite(in1, 0);
    Serial.print("S");
}