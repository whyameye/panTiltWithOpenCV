#include <Arduino.h>
#include <Servo.h>

#define PIR_PIN 8

Servo pan;
Servo tilt;

void setup() {
    // initialize digital pin LED_BUILTIN as an output.
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    tilt.attach(10); //  min 40 max 115 (higher)
    pan.attach(9); // min 0 max 180 straight ahead 90
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    pinMode(PIR_PIN, INPUT);
    digitalWrite(PIR_PIN, HIGH);
}

// the loop function runs over and over again forever
void loop() {
    static int byteNum = 0;
    static int humanWasInSpace = false;
    int humanInSpace = digitalRead(8);

    if (humanInSpace != humanWasInSpace) {
        Serial.println(humanInSpace);
        humanWasInSpace = humanInSpace;
    }

    if (Serial.available() > 0) {
        int incomingByte = Serial.read();
        if (incomingByte == 255) {
            byteNum = 1;
            digitalWrite(LED_BUILTIN, LOW);
        } else if ((byteNum == 1) && (incomingByte < 181)) {
            pan.write(incomingByte);
            byteNum++;
        } else if ((byteNum == 2) && (incomingByte >= 40) && (incomingByte <= 115)){
            tilt.write(incomingByte);
            byteNum = 0;
        }
    }
}
