// Zack Bright        - zbright  _ mit _ edu,    Sept 2015
// Daniel J. Gonzalez - dgonz    _ mit _ edu,    Sept 2015
// Fangzhou Xia       - xiafz    _ mit _ edu,    Sept 2015
// Peter KT Yu        - peterkty _ mit _ edu,    Sept 2016
// Ryan Fish          - fishr    _ mit _ edu,    Sept 2016

#include "helper.h"

//Encoder Measurement Class function implementation
EncoderMeasurement::EncoderMeasurement(): 
    encoder1CountPrev(0),
    encoder2CountPrev(0),
    v_R(0), v_L(0)
{
        
    rot_per_tick = 1.0 / tick_per_rot;
    rad_per_tick = 2 * PI * rot_per_tick;
    met_per_tick = wheelRadius * rad_per_tick; 
    
    maxMV = voltage * motor_const / gearing * wheelRadius;  // max wheel speed (m/2)
}

void EncoderMeasurement::update() {
    float encoder1Count = readEncoder(1);
    float encoder2Count = -1 * readEncoder(2);

    float dEncoder1 = (encoder1Count - encoder1CountPrev);
    float dEncoder2 = (encoder2Count - encoder2CountPrev);
    
    //update the angle increment in radians
    float dphi1 = (dEncoder1 * rad_per_tick);
    float dphi2 = (dEncoder2 * rad_per_tick);

    //for encoder index and motor position switching (Right is 1, Left is 2)
    dThetaR = dphi1;
    dThetaL = dphi2;
    
    //linear distance swept out by wheel
    float dWheel1 = (dEncoder1 * met_per_tick);
    float dWheel2 = (dEncoder2 * met_per_tick);
    
    //wheel velocity (Right is 1, Left is 2)
    float mV1 = dWheel1 / PERIOD;
    float mV2 = dWheel2 / PERIOD;
    v_R = mV1;
    v_L = mV2;
    encoder1CountPrev = encoder1Count;
    encoder2CountPrev = encoder2Count;
}

//RobotPose Class function implementation
void RobotPose::reset() {
    X = 0;
    Y = 0;
    Th = 0;
}

void RobotPose::update(float dThetaL, float dThetaR) {
    Th += (r / (2.0 * b)) * (dThetaR - dThetaL);
    X += (r / 2.0) * cos(Th) * (dThetaR + dThetaL);
    Y += (r / 2.0) * sin(Th) * (dThetaR + dThetaL);
}

// PIController Class function implementation (not the focus of this lab)
void PIController::doPIControl(String side, float desV, float currV) {
    float error = desV - currV;

    if (side == "Right") {     // Motor 1 
        // P
        float PCommand = Kpv1 * error;

        // I
        mIntegratedVError1 = constrain(mIntegratedVError1 + error * PERIOD, -0.02, 0.02);
        float ICommand = Kiv1 * mIntegratedVError1;
        
        // Sum
        float command =  PCommand + ICommand;

        int sign = 1;
        md.setM1Speed(constrain(sign * command, -400, 400));   // use sign to make sure positive commands move robot forward
    }
    else if (side == "Left") {  // Motor 2
        // P
        float PCommand = Kpv2 * error;

        // I
        mIntegratedVError2 = constrain(mIntegratedVError2 + error * PERIOD, -0.02, 0.02);
        float ICommand = Kiv2 * mIntegratedVError2;

        // Sum
        float command =  PCommand + ICommand;

        int sign = -1;
        md.setM2Speed(constrain(sign * command, -400, 400));   // use sign to make sure positive commands move robot forward
    }
    else {
        md.setM1Speed(0);
        md.setM2Speed(0);
        Serial.println("ERROR: INVALID MOTOR CHOICE GIVEN TO PI CONTROLLER");
    }
}


