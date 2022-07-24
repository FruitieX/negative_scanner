#include <AccelStepper.h>

const int STEP_PIN = 4;
const int DIR_PIN = 5;
const int FOCUS_PIN = 9;
const int SHUTTER_PIN = 10;

String serial_buffer = "";

AccelStepper stepper(1, STEP_PIN, DIR_PIN);

void setup() {
  pinMode(SHUTTER_PIN, OUTPUT);
  digitalWrite(SHUTTER_PIN, LOW);
  
  pinMode(FOCUS_PIN, OUTPUT);
  digitalWrite(FOCUS_PIN, LOW);
  
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  
  Serial.begin(115200);

  stepper.setMaxSpeed(4000);
  stepper.setAcceleration(25000);
  stepper.setPinsInverted(true);
}

void loop() {
  if (Serial.available() > 0) {
    char incoming_byte = Serial.read();
    
    if (incoming_byte == 'M') {
      int steps = serial_buffer.toInt();
      serial_buffer = "";

      Serial.print("Moving");
      Serial.println(steps);

      if (steps == 0) {
        stepper.stop();
      } else {
        stepper.move(steps);
      }
    }
    else if (incoming_byte == 's') {
      int speed = serial_buffer.toInt();
      serial_buffer = "";
      stepper.setMaxSpeed(speed);
    }
    else if (incoming_byte == 'S') {
      // Shutter
      digitalWrite(SHUTTER_PIN, HIGH);
      delay(50);
      digitalWrite(SHUTTER_PIN, LOW);
    }
    else if (incoming_byte == 'F') {
      digitalWrite(FOCUS_PIN, HIGH);
    } else if (incoming_byte == 'f') {
      digitalWrite(FOCUS_PIN, LOW);
    }
    else if (incoming_byte == '\n') {
      // Ignore newline
    }
    else {
      serial_buffer += incoming_byte;
    }
  }
  
  stepper.run();
}
