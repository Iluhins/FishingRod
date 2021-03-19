#include <Servo.h>
#define calibration 2000
#define outputA 3
#define outputB 4
#define switchButton 2/*
#define lowerRight 7
#define lowerLeft 9
#define upperLeft 6
#define upperRight 10
#define upper 5
#define lower 8*/
  #define lowerRight 10
  #define lowerLeft 9
  #define upperLeft 6
  #define upperRight 5
  #define upper 7
  #define lower 8
#define upCoefficient 200
#define downCoefficient 1000
#define maxAngle 90
#define triggerAngle 90
#define LPF_K 100
#define servoSpeed 50
int basicSensitivity  = 0;// 370
int counter = 0;
int aState;
int bState;
int mode = 0;
int lastPush = 0;
Servo servo;
float fVal = 0;
int sensitivity = 2;
int intensivity = 2;
int amplitude = 2;
int angle = 10;
bool reset = false;
int val = 0;
long work = 0;
long check = 0;
bool hookon = true;
bool up = false;
bool modnum = false;
long displayMode = 0;
volatile byte state = LOW;
int interval = 0;
int look = 0;
bool stopCheck = false;
int maxFval = 0;
int rotationSpeedx3[5] = {0, 0, 0, 0, 0};
int basicSensitivityx3[5] = {0, 0, 0, 0, 0};
void clear() {
  digitalWrite(upperRight, HIGH);
  digitalWrite(lowerRight, HIGH);
  digitalWrite(upperLeft, HIGH);
  digitalWrite(lowerLeft, HIGH);
  digitalWrite(upper, HIGH);
  digitalWrite(lower, HIGH);
}
void setup() {
  Serial.begin(9600);
  servo.attach(1);// put your setup code here, to run once:
  pinMode (switchButton, INPUT_PULLUP);
  pinMode (outputA, INPUT_PULLUP);
  pinMode (outputB, INPUT_PULLUP);
  pinMode (A0, INPUT_PULLUP);
  interval = millis();
  while (look <= 10 * (calibration)) {
    val = analogRead(A0);
    if (interval < millis()) {
      interval = millis();
      look++;
      fVal += (val - fVal) / LPF_K;
      if (fVal > maxFval) {
        maxFval = fVal;
      }
      if (look % calibration == 0) {
        stopCheck = true;
        Serial.println(maxFval);
        basicSensitivity += maxFval;
        maxFval = 0;
        if (up) {
          setServo(0);
          up = false;
          Serial.println("up");
        } else {
          setServo(15 - (3 * (look / (2 * calibration))));
          up = true;
          Serial.println("down");
        }
      }
    }
  }
  basicSensitivity = basicSensitivity / 10 - 3;
  for (int c = 5; c <= 10; c++) {
    pinMode(c, OUTPUT);
  }
  attachInterrupt(digitalPinToInterrupt(outputA), trigger, CHANGE);
  attachInterrupt(digitalPinToInterrupt(switchButton), button, FALLING);
  delay(1000);
  Serial.print("Basic Sensitivity:");
  Serial.println(basicSensitivity);
  Serial.println("Rotation Speed:");
  for (int i = 0; i < 5; i++) {
    look = 0;
    if (rotationSpeedx3[i] == 0) {
      interval = millis();
      while (look <= 2 * (calibration)) {
        val = analogRead(A0);
        if (interval < millis()) {
          interval = millis();
          look++;
          fVal += (val - fVal) / LPF_K;
          if (look % calibration == 0) {
            stopCheck = true;
            if (up) {
              setServo(0);
              up = false;
              Serial.println("up");
            } else {
              setServo(15 - (3 * i));
              up = true;
              Serial.println("down");
            }
          }
        }
        if (look % calibration > basicSensitivity) {
          if ((stopCheck == true)  && (fVal < 50)) {
            if (rotationSpeedx3[i] < look % calibration) {
              rotationSpeedx3[i] = look % calibration;
              Serial.print("rotation time:");
              Serial.println(look % calibration);
            }
            stopCheck = false;
          }
          if (fVal > 50) {
            stopCheck = true;
          }
        }
      }
      i--;
    } else {
      Serial.println(rotationSpeedx3[i]);
    }
  }
}
void loop() {
  if (mode == 0) {
    printMode();
    Serial.println("Delay");
    Serial.println("setServo#1");
    setServo(0);
    delay (15000);
    if(mode == 0){
      mode = 1;
    }
    Serial.println("timer ended");
    interval = millis();
  }
  if ((mode == 1) && (hookon)) {
    if (millis() > interval) {
      interval = millis();
      printMode();
      aState = digitalRead(outputA);
      bState = digitalRead(outputB);

      if (millis() >= displayMode) {
        if (modnum) {
          displayMode = millis() + 1000;
          printMode();
        } else {
          displayMode = millis() + 1000;
          printNumber();
        }
        modnum = !modnum;
      }
      if (millis() >= work) {
        if (up) {
          work = millis() + downCoefficient * intensivity + rotationSpeedx3[amplitude - 1] + 20;
          up = false;
          Serial.print("setServo#6: ");
          Serial.println(amplitude * 3);
          setServo(amplitude * 3);
          Serial.println("Up");
        } else {
          up = true;
          work = millis() + upCoefficient * intensivity + rotationSpeedx3[amplitude - 1] + 20;
          Serial.print("setServo#7: ");
          Serial.println(0);
          setServo(0);
          Serial.println("Down");
        }
      }
      if ((!up) && (millis() >= work - downCoefficient * intensivity)) {
        val = analogRead(A0);
        fVal += (val - fVal) / LPF_K;
        if (fVal > (basicSensitivity + 10 * sensitivity)) {
          Serial.println("setServo#2");
          setServo(120);
          delay(200);
          Serial.println("setServo#3");
          setServo(80);
          Serial.print("Long triggered: ");
          Serial.println(fVal);
          hookon = false;
        }
      }
      if ((up) && (millis() >= work - upCoefficient * intensivity)) {
        val = analogRead(A0);
        fVal += (val - fVal) / LPF_K;
        if (fVal > (basicSensitivity + 10 * sensitivity)) {
          Serial.println("setServo#4");
          setServo(120);
          delay(200);
          Serial.println("setServo#5");
          setServo(80);
          Serial.print("Short triggered: ");
          Serial.println(fVal);
          hookon = false;
        }
      }
    }
  } else if (mode >= 2) {
    aState = digitalRead(outputA);
    bState = digitalRead(outputB);
  }
}
void button() {
  if (millis() > lastPush + 300) {
    Serial.println("Button Pressed");
    if (!hookon) {
      Serial.println(fVal);
      clear();
      mode = 0;
      hookon = true;
      fVal = 10;
    } else {
        mode++;
      if (mode > 4) {
        mode = 0;
      }
      printMode();
    }
  }
  lastPush = millis();
}
void trigger() {
  if (mode >= 2) {
    Serial.print("outputA: ");
    Serial.print(aState);
    Serial.print(" outputB: ");
    Serial.println(bState);
    if (aState) {
      if (bState != aState) {
        if (mode == 2) {
          if (amplitude < 3) {
            amplitude++;
          }
        } else if (mode == 3) {
          if (intensivity < 3) {
            intensivity++;
          }
        } else if (mode == 4) {
          if (sensitivity < 3) {
            sensitivity++;
          }
        }
      } else {
        if (mode == 2) {
          if (amplitude > 1) {
            amplitude--;
          }
        } else if (mode == 3) {
          if (intensivity > 1) {
            intensivity--;
          }
        } else if (mode == 4) {
          if (sensitivity > 1) {
            sensitivity--;
          }
        }
      }
    }
    Serial.println(amplitude);
    Serial.println(intensivity);
    Serial.println(sensitivity);
    printNumber();
  }
}
void printNumber() {
  int number;
  if (mode == 2) {
    number = amplitude;
  } else if (mode == 3) {
    number = intensivity;
  }
  else if (mode == 4) {
    number = sensitivity;
  }
  clear();
  if (number == 0) {
    digitalWrite(upperRight, LOW);
    digitalWrite(upper, LOW);
    digitalWrite(upperLeft, LOW);
  } else if (number == 1) {
    digitalWrite(upperRight, LOW);
    digitalWrite(lowerRight, LOW);
  } else if (number == 2) {
    digitalWrite(upperRight, LOW);
    digitalWrite(lowerLeft, LOW);
    digitalWrite(upper, LOW);
    digitalWrite(lower, LOW);
  } else if (number == 3) {
    digitalWrite(upperRight, LOW);
    digitalWrite(lowerRight, LOW);
    digitalWrite(upper, LOW);
    digitalWrite(lower, LOW);
  } else if (number == 4) {
    digitalWrite(upperRight, LOW);
    digitalWrite(upperLeft, LOW);
    digitalWrite(upper, LOW);
    digitalWrite(lowerRight, LOW);
  } else if (number == 5) {
    digitalWrite(lower, LOW);
    digitalWrite(upperLeft, LOW);
    digitalWrite(upper, LOW);
    digitalWrite(lowerRight, LOW);
  } else if (number == 6) {
    digitalWrite(upperLeft, LOW);
    digitalWrite(lowerLeft, LOW);
    digitalWrite(upper, LOW);
    digitalWrite(lowerRight, LOW);
    digitalWrite(lower, LOW);
  } else if (number == 7) {
    digitalWrite(upperRight, LOW);
    digitalWrite(lowerRight, LOW);
    digitalWrite(upper, LOW);
  } else if (number == 8) {
    digitalWrite(upperRight, LOW);
    digitalWrite(lowerRight, LOW);
    digitalWrite(upperLeft, LOW);
    digitalWrite(lowerLeft, LOW);
    digitalWrite(upper, LOW);
    digitalWrite(lower, LOW);
  } else if (number == 9) {
    digitalWrite(upperRight, LOW);
    digitalWrite(lowerRight, LOW);
    digitalWrite(upperLeft, LOW);
    digitalWrite(upper, LOW);
    digitalWrite(lower, LOW);
  }
}
void printMode() {
  clear();
  if (mode == 0) {
    digitalWrite(upperRight, LOW);
    digitalWrite(upper, LOW);
    digitalWrite(upperLeft, LOW);
  } else if (mode == 1) {
    int t = (millis() - work);
    if ((t % 1000) > -500) {
      digitalWrite(upper, LOW);
    } else {
      digitalWrite(upperLeft, LOW);
      digitalWrite(upperRight, LOW);
    }
  } else if (mode == 2) {
    Serial.println("Intensitivy");
    digitalWrite(upperLeft, LOW);
    digitalWrite(upper, LOW);
    digitalWrite(lowerLeft, LOW);
  } else if (mode == 3) {
    Serial.println("Amplitude");
    digitalWrite(upperRight, LOW);
    digitalWrite(lowerRight, LOW);
    digitalWrite(upperLeft, LOW);
    digitalWrite(upper, LOW);
    digitalWrite(lowerLeft, LOW);
  }
  else if (mode == 4) {
    Serial.println("Sensitivity");
    digitalWrite(lower, LOW);
    digitalWrite(upperLeft, LOW);
    digitalWrite(upper, LOW);
    digitalWrite(lowerRight, LOW);
  }
}
void setServo(int setValue) {
  servo.writeMicroseconds(1000 + setValue * 1000 / 90);
}
