#include <AccelStepper.h>
#include <Wire.h>

// ================== STEPPER ==================
AccelStepper stepA(AccelStepper::DRIVER, PA0, PA1);
AccelStepper stepB(AccelStepper::DRIVER, PA2, PA3);
AccelStepper stepC(AccelStepper::DRIVER, PA4, PA5);

// ================== LIMIT SWITCH (NC) ==================
const uint8_t LIMIT_A = PA6;
const uint8_t LIMIT_B = PB0;
const uint8_t LIMIT_C = PA7;

bool homeDone = false;

// ================== HOMING ==================
void doHoming()
{
    const float HOMING_SPEED = -2000.0f;   // CHẬM – CHẮC
    const long HOMING_BACKOFF = 100;

    while ( digitalRead(LIMIT_A) == HIGH ||
            digitalRead(LIMIT_B) == HIGH ||
            digitalRead(LIMIT_C) == HIGH )
    {
        bool moveA = digitalRead(LIMIT_A) == HIGH;
        bool moveB = digitalRead(LIMIT_B) == HIGH;
        bool moveC = digitalRead(LIMIT_C) == HIGH;

        stepA.setSpeed(moveA ? HOMING_SPEED : 0);
        stepB.setSpeed(moveB ? HOMING_SPEED : 0);
        stepC.setSpeed(moveC ? HOMING_SPEED : 0);

        stepA.runSpeed();
        stepB.runSpeed();
        stepC.runSpeed();
    }

    stepA.setCurrentPosition(0);
    stepB.setCurrentPosition(0);
    stepC.setCurrentPosition(0);

    stepA.moveTo(HOMING_BACKOFF);
    stepB.moveTo(HOMING_BACKOFF);
    stepC.moveTo(HOMING_BACKOFF);

    while (stepA.distanceToGo() || stepB.distanceToGo() || stepC.distanceToGo())
    {
        stepA.run();
        stepB.run();
        stepC.run();
    }

    stepA.setCurrentPosition(0);
    stepB.setCurrentPosition(0);
    stepC.setCurrentPosition(0);

    homeDone = true;
}

// ================== SETUP ==================
void setup()
{
    Serial1.begin(115200);

    pinMode(LIMIT_A, INPUT_PULLUP);
    pinMode(LIMIT_B, INPUT_PULLUP);
    pinMode(LIMIT_C, INPUT_PULLUP);

    stepA.setMaxSpeed(10000);
    stepA.setAcceleration(8000); 

    stepB.setMaxSpeed(10000);
    stepB.setAcceleration(8000);

    stepC.setMaxSpeed(10000);
    stepC.setAcceleration(8000);

    doHoming();
}

// ================== LOOP ==================
void loop()
{

}