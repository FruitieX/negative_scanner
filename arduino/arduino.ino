
const int RST_PIN = 3;
const int SLP_PIN = 4;
const int STEP_PIN = 5;
const int DIR_PIN = 6;
const int FOCUS_PIN = 9;
const int SHUTTER_PIN = 10;
const int MICROSTEP_PIN = 11;

const int STEP_PULSE = 800;

void setup() {
  pinMode(SHUTTER_PIN, OUTPUT);
  digitalWrite(SHUTTER_PIN, LOW);
  
  pinMode(FOCUS_PIN, OUTPUT);
  digitalWrite(FOCUS_PIN, LOW);
  
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  
  // Enable quarter microstepping
  digitalWrite(MICROSTEP_PIN, HIGH);
  pinMode(MICROSTEP_PIN, OUTPUT);
  
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);
  
  pinMode(SLP_PIN, OUTPUT);
  digitalWrite(SLP_PIN, HIGH);
  
//  Serial.begin(115200);
  Serial.begin(19200);
//  Serial.begin(38400);
}

void loop() {
  if (Serial.available() > 0) {
    int incoming_byte = Serial.read();
    
    if (incoming_byte == 'L') {
      digitalWrite(SLP_PIN, HIGH);
      
      // Step to the left
      digitalWrite(DIR_PIN, LOW);
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(STEP_PULSE);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(STEP_PULSE);
    }
    else if (incoming_byte == 'R') {
      digitalWrite(SLP_PIN, HIGH);
      
      // Step to the right
      digitalWrite(DIR_PIN, HIGH);
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(STEP_PULSE);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(STEP_PULSE);
    }
    if (incoming_byte == 'S') {
      // Force motor to stop to avoid vibrations
      digitalWrite(SLP_PIN, LOW);
      
      // Shutter
      digitalWrite(FOCUS_PIN, HIGH);
      delay(50);
      digitalWrite(SHUTTER_PIN, HIGH);
      delay(50);
      digitalWrite(SHUTTER_PIN, LOW);
      digitalWrite(FOCUS_PIN, LOW);

      digitalWrite(FOCUS_PIN, HIGH);
    }
  }
}
