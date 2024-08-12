#include "Motor.h"

Motor::Motor(int motorPin1, int motorPin2, 
        int pwmPin, int encoderPinA, int encoderPinB) 
{
    _motorPin1 = motorPin1;
    _motorPin2 = motorPin2;
    _pwmPin = pwmPin;
    _encoderPinA = encoderPinA;
    _encoderPinB = encoderPinB;
    setupPins();
}

void Motor::setupPins() {
    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
    pinMode(_pwmPin, OUTPUT);
    pinMode(_encoderPinA, INPUT);
    pinMode(_encoderPinB, INPUT);

    Encoder encoder(_encoderPinA, _encoderPinB);
}

int Motor::getEncoderAValue() {
    return digitalRead(_encoderPinA);
}

int Motor::getEncoderBValue() {
    int value = digitalRead(_encoderPinB);
}

void Motor::setMotor(int direction, int pwmValue) {
    // constrain pwm value between 0 and 255
    if (pwmValue > 255) {
        pwmValue = 255;
    } else if (pwmValue < 0) {
        pwmValue = 0;
    }
    
    analogWrite(_pwmPin, pwmValue);
    if (direction == 1) {
        digitalWrite(_motorPin1, HIGH);
        digitalWrite(_motorPin2, LOW);
    } else if (direction == -1) {
        digitalWrite(_motorPin1, LOW);
        digitalWrite(_motorPin2, HIGH);
    } else {
        digitalWrite(_motorPin1, LOW);
        digitalWrite(_motorPin2, LOW);
    }
}

void Motor::stop() {
    setMotor(0, 0);
}
