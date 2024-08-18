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
    analogWrite(_pwmPin, pwmValue);
}

void Motor::setMotorSpeed(int pwmValue) {
    int direction = 0;
    // constrain pwm value between 0 and 255
    if (pwmValue > 255) {
        pwmValue = 255;
    } else if (pwmValue < -255) {
        pwmValue = -255;
    }

    if(pwmValue > 0) {
        direction = 1;
    } else if(pwmValue < 0) {
        direction = -1;
        pwmValue = -pwmValue;
    }
    
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
    analogWrite(_pwmPin, pwmValue);
}

void Motor::stop() {
    setMotorSpeed(0, 0);
}

int Motor::getEncoderCount() {
    return _encoder.read();
}

float Motor::getCurrentVelocity() {
    return _current_velocity;
}

void Motor::setPIDParams(float kp, float ki, float kd) {
    _pid.setParams(kp, ki, kd, _MAX_PWM);
    return;
}

void Motor::setMotorSpeedWithPID(int motorSpeed) {
    int position;
    int pwm_signal;
    int direction;

    // Convert to wheel speeds (ticks per second)


    long currentTime = micros();
    
    float deltaTime = ((float) (currentTime - _prevTime)) / ( 1.0e6 );
    _prevTime = currentTime;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        position = _encoder.read();
    }

    _current_velocity = ((position - _prevPosition) / deltaTime);
    _prevPosition = position;

    _pid.evaluate(_current_velocity, motorSpeed, deltaTime, pwm_signal, direction);
    setMotorSpeed(direction, pwm_signal);
    return;    
}

// void Motor::setMotorSpeedWithPID(int motorSpeed) {
//     int pos
//     ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
//         position = _encoder.read();
//     }