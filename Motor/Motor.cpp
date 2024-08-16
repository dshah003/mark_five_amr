#include "Motor.h"

Motor::Motor(int motorPin1, int motorPin2, 
        int pwmPin, int encoderPinA, int encoderPinB) : 
        _encoder(encoderPinA, encoderPinB),
        _motorPin1(motorPin1),
        _motorPin2(motorPin2),
        _pwmPin(pwmPin),
        _encoderPinA(encoderPinA),
        _encoderPinB(encoderPinB)
{
    setupPins();
}

void Motor::setupPins() {
    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
    pinMode(_pwmPin, OUTPUT);
    pinMode(_encoderPinA, INPUT);
    pinMode(_encoderPinB, INPUT);
}

int Motor::getEncoderAValue() {
    return digitalRead(_encoderPinA);
}

int Motor::getEncoderBValue() {
    int value = digitalRead(_encoderPinB);
}

void Motor::setMotorSpeed(int direction, int pwmValue) {
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
    setMotorSpeed(0, 0);
}

int Motor::getEncoderCount() {
    return _encoder.read();
}

void Motor::setPIDParams(float kp, float ki, float kd) {
    _pid.setParams(kp, ki, kd, _MAX_PWM);
    return;
}

void Motor::setMotorSpeedWithPID(int motorSpeed) {
    float current_velocity;
    int position;
    int pwm_signal;
    int direction;

    long currentTime = micros();
    
    float deltaTime = ((float) (currentTime - _prevTime)) / ( 1.0e6 );
    _prevTime = currentTime;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        position = _encoder.read();
    }

    current_velocity = ((position - _prevPosition) / deltaTime);
    _prevPosition = position;

    _pid.evaluate(position, motorSpeed, deltaTime, pwm_signal, direction);
    setMotorSpeed(direction, pwm_signal);
    return;    
}
