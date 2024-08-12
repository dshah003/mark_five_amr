#ifndef Motor_h
#define Motor_h

#include "Arduino.h"
#include "Encoder.h"

class Motor {
    public:
        /**
         * Constructs a Motor object with the given pin configurations.
         *
         * @param motorPin1 pin number for the first motor input pin
         * @param motorPin2 pin number for the second motor input pin
         * @param pwmPin pin number for the enable pin
         * @param encoderPinA pin number for Encoder A pin
         * @param encoderPinB pin number for Encoder B pin
         */
        Motor(int motorPin1, int motorPin2, 
            int pwmPin, int encoderPinA, int encoderPinB);
        
        /**
         * @brief Actuates the Motor to set the speed and direction.
         * 
         * @param direction 
         * @param pwm 
         */
        void setMotor(int direction, int pwm);
        
        /**
         * @brief Stop the motors.
         * 
         */
        void stop();

        /**
         * @brief Get the Encoder's Pin A Value
         * 
         * @return int 
         */
        int getEncoderAValue();

        /**
         * @brief Get the Encoder's Pin B Value
         * 
         * @return int 
         */
        int getEncoderBValue();

        void setMotorSpeed(int speed);

    private:
        int _motorPin1;
        int _motorPin2;
        int _pwmPin;
        int _encoderPinA;
        int _encoderPinB;

        /**
         * @brief Sets up the pins for the motor appropriately.
         */
        void setupPins();
 
};

#endif