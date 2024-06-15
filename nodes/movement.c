int leftStepPin = 3;
int leftDirPin = 5;
int leftDir = LOW;

int rightStepPin = 6;
int rightDirPin = 4;
int rightDir = LOW;

int stepDelay = 3500;

void setup() {
  pinMode(leftStepPin, OUTPUT);
  pinMode(leftDirPin, OUTPUT);
  pinMode(rightStepPin, OUTPUT);
  pinMode(rightDirPin, OUTPUT);
  digitalWrite(leftDirPin, leftDir);
  digitalWrite(rightDirPin, rightDir);
}

void leftStep() {
  digitalWrite(leftStepPin, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(leftStepPin,LOW);
  delayMicroseconds(stepDelay);
}

void rightStep() {
  digitalWrite(rightStepPin, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(rightStepPin,LOW);
  delayMicroseconds(stepDelay);
}

void leftStepForwards() {
  if(leftDir != HIGH) {
    digitalWrite(leftDirPin, HIGH);
    leftDir = HIGH;
  }
  leftStep();
}

void rightStepForwards() {
  if(rightDir != HIGH) {
    digitalWrite(rightDirPin, HIGH);
    rightDir = HIGH;
  }
  rightStep();
}

void leftStepBackwards() {
  if(leftDir != LOW) {
    digitalWrite(leftDirPin, LOW);
    leftDir = LOW;
  }
  leftStep();
}

void rightStepBackwards() {
  if(rightDir != LOW) {
    digitalWrite(rightDirPin, LOW);
    rightDir = LOW;
  }
  rightStep();
}

void stepRight() {
  leftStepForwards();
  rightStepForwards();
}

void stepLeft() {
  leftStepBackwards();
  rightStepBackwards();
}

void stepUp() {
  leftStepForwards();
  rightStepBackwards();
}

void stepDown() {
  leftStepBackwards();
  rightStepForwards();
}

void loop() {
  
  for(int x = 0; x < 300; x++) {
    stepLeft();
  }
  delay(1000);

  for(int x = 0; x < 300; x++) {
    stepUp();
  }
  delay(1000);

  for(int x = 0; x < 300; x++) {
    stepRight();
  }
  delay(1000);

  for(int x = 0; x < 300; x++) {
    stepDown();
  }
  delay(1000);
  
}