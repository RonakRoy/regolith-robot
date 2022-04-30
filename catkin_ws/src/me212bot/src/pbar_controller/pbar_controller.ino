#include "DualTB9051FTGMotorShield.h"

DualTB9051FTGMotorShield md;

// 600 pulses per rotation
int encoder_count = 0;
int current_count = 0;

int desired_count = 0;
int error = 0;
long current_time, last_time;
int last_error = 0;
int error_sum = 0;

float kP = 7.5;
float kI = 0.001;
float kD = 0.5;

int motor_speed = 0;

void setup() {
    Serial.begin(115200);
    md.init();

    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2), int2, RISING);
    attachInterrupt(digitalPinToInterrupt(3), int3, RISING);

    md.enableDrivers();
    md.setM2Speed(0);

    last_time = millis();
}

void loop() {
    receiveSerialData();

    current_time = millis();
    current_count = encoder_count;

    error = desired_count - current_count;
    error_sum = error * (current_time-last_time);
    error_sum = constrain(error_sum, -10000, 10000);

    motor_speed = kP * error + kI * error_sum + kD * (error - last_error)/(current_time-last_time);
    motor_speed = constrain(motor_speed, -400, 400);

    md.setM2Speed(50+motor_speed);

    Serial.print("PBAR,");
    Serial.print(current_count);
    Serial.println("");

}

void receiveSerialData() {
    if (Serial.available() > 0) {
        String commandString = Serial.readStringUntil('\n');
        float command[1];
        for (int i = 0, indexPointer = 0; indexPointer != -1 ; i++) {
            indexPointer = commandString.indexOf(',');
            String tempString = commandString.substring(0, indexPointer);
            command[i] = tempString.toFloat();
            commandString = commandString.substring(indexPointer+1);
        }
        desired_count = command[0];
    }
}

void int2() {
    if (digitalRead(3)==LOW) encoder_count++;
    else encoder_count--;
}

void int3() {
    if (digitalRead(2)==LOW) encoder_count--;
    else encoder_count++;
}
