#include <AccelStepper.h>

const int RST_PIN = 3;
const int SLP_PIN = 4;
const int STEP_PIN = 5;
const int DIR_PIN = 6;
const int FOCUS_PIN = 9;
const int SHUTTER_PIN = 10;
const int MICROSTEP_PIN = 11;

const int STEP_PULSE = 600;

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
  
  // (don't) enable quarter microstepping
  digitalWrite(MICROSTEP_PIN, LOW);
  pinMode(MICROSTEP_PIN, OUTPUT);
  
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);
  
  pinMode(SLP_PIN, OUTPUT);
  digitalWrite(SLP_PIN, HIGH);
  
  Serial.begin(115200);

  stepper.setMaxSpeed(1500); // 1600 ~= max before motor starts missing steps
  stepper.setAcceleration(10000);
}

void loop() {
  if (Serial.available() > 0) {
    char incoming_byte = Serial.read();
    
    if (incoming_byte == 'M') {
      int steps = serial_buffer.toInt();
      serial_buffer = "";

      if (steps == 0) {
        stepper.stop();
      } else {
        stepper.move(steps);
      }
    }
    else if (incoming_byte == 'S') {
      // Force motor to stop to avoid vibrations
      //digitalWrite(SLP_PIN, LOW);
      
      // Shutter
      //digitalWrite(FOCUS_PIN, HIGH);
      //delay(50);
      digitalWrite(SHUTTER_PIN, HIGH);
      delay(50);
      digitalWrite(SHUTTER_PIN, LOW);
      //digitalWrite(FOCUS_PIN, LOW);

      //digitalWrite(FOCUS_PIN, HIGH);
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
