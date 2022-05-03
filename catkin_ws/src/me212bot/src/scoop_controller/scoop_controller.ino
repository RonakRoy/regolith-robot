#define WRIST_STEP 3
#define WRIST_DIR 6
#define WRIST_PULSE_TIME 20000

#define JAW_STEP 2
#define JAW_DIR 5
#define JAW_PULSE_TIME 2000

int desired_jaw_pos;
int desired_wrist_pos;

int jaw_pos = 0;
int wrist_pos = 0;

void setup() {
    Serial.begin(115200);

    pinMode(WRIST_STEP, OUTPUT);
    pinMode(WRIST_DIR, OUTPUT);

    pinMode(JAW_STEP, OUTPUT);
    pinMode(JAW_DIR, OUTPUT);
}

void loop() {
    receiveSerialData();

    if (wrist_pos != desired_wrist_pos) {
        digitalWrite(WRIST_DIR, (wrist_pos > desired_wrist_pos) ? LOW : HIGH);
        digitalWrite(WRIST_STEP,HIGH);
        delayMicroseconds(WRIST_PULSE_TIME);
        digitalWrite(WRIST_STEP,LOW);
        delayMicroseconds(WRIST_PULSE_TIME);

        wrist_pos += (wrist_pos > desired_wrist_pos) ? -1 : 1;
    }

    if (jaw_pos != desired_jaw_pos) {
        digitalWrite(JAW_DIR, (jaw_pos > desired_jaw_pos) ? HIGH : LOW);
        digitalWrite(JAW_STEP,HIGH);
        delayMicroseconds(JAW_PULSE_TIME);
        digitalWrite(JAW_STEP,LOW);
        delayMicroseconds(JAW_PULSE_TIME);

        jaw_pos += (jaw_pos > desired_jaw_pos) ? -1 : 1;
    }

    Serial.print("SCOOP,");
    Serial.print(wrist_pos);
    Serial.print(",");
    Serial.print(jaw_pos);
    Serial.println("");
}

void receiveSerialData() {
    if (Serial.available() > 0) {
        String commandString = Serial.readStringUntil('\n');
        float command[2];
        for (int i = 0, indexPointer = 0; indexPointer != -1 ; i++) {
            indexPointer = commandString.indexOf(',');
            String tempString = commandString.substring(0, indexPointer);
            command[i] = tempString.toFloat();
            commandString = commandString.substring(indexPointer+1);
        }
        desired_wrist_pos = command[0];
        desired_jaw_pos = command[1];
    }
}
