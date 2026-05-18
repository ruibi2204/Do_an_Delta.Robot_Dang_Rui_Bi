#include <AccelStepper.h>
#include <Wire.h>

// ================== STEPPER ==================
AccelStepper stepA(AccelStepper::DRIVER, PA0, PA1);
AccelStepper stepB(AccelStepper::DRIVER, PA2, PA3);
AccelStepper stepC(AccelStepper::DRIVER, PA4, PA5);

// ================== LIMIT SWITCH (NC) ==================
#define LIMIT_A PA6
#define LIMIT_B PB0
#define LIMIT_C PA7

bool homeDone = false;

// ================== HOMING ==================
void doHoming()
{
    const float HOMING_SPEED = -2000;   // CHẬM – CHẮC

    while ( digitalRead(LIMIT_A) == HIGH ||
            digitalRead(LIMIT_B) == HIGH ||
            digitalRead(LIMIT_C) == HIGH )
    {
        if (digitalRead(LIMIT_A) == HIGH) stepA.setSpeed(HOMING_SPEED);
        else stepA.setSpeed(0);

        if (digitalRead(LIMIT_B) == HIGH) stepB.setSpeed(HOMING_SPEED);
        else stepB.setSpeed(0);

        if (digitalRead(LIMIT_C) == HIGH) stepC.setSpeed(HOMING_SPEED);
        else stepC.setSpeed(0);

        stepA.runSpeed();
        stepB.runSpeed();
        stepC.runSpeed();
    }

    stepA.setCurrentPosition(0);
    stepB.setCurrentPosition(0);
    stepC.setCurrentPosition(0);

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