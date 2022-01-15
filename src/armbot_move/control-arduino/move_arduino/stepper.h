#pragma once

class Stepper {
    public:
        Stepper(int _stepPin, int _dirPin, int _enablePin) {
        stepPin = _stepPin;
        dirPin = _dirPin;
        enablePin = _enablePin;
    }

    AccelStepper getStepper() {
        AccelStepper stepper(1, stepPin, dirPin);
        stepper.setEnablePin(enablePin);
        stepper.setPinsInverted(false, false, true);
        stepper.enableOutputs();
        stepper.setMaxSpeed(500);

        return stepper;
    }

    AccelStepper setSpeedStepper(AccelStepper _stepper, int _speed) {
       _stepper.setMaxSpeed(_speed);

       return _stepper;
    }

    private:
        int stepPin;
        int dirPin;
        int enablePin;
};
