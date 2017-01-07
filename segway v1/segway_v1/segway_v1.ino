#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Obliczone wartosci Pitch i Roll tylko z akcelerometru
float accPitch = 0;
float accRoll = 0;

const int
PWM_A   = 3, DIR_1_A   = 10, DIR_2_A   = 9, BRAKE_A = 9,
PWM_B   = 5, DIR_1_B   = 7, DIR_2_B   = 8, BRAKE_B = 8;

float DEBUG = 0 ;  // 0 false, 1 true

float angle = 0, lastAngle = 0, lastLastAngle = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Inicjalizacja MPU6050");
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Nie znaleziono MPU6050!");
    delay(500);
  }
  mpu.calibrateGyro();

  if (!DEBUG) delay(10000);

  pinMode(DIR_1_B, OUTPUT);    // Direction pin on channel B
  pinMode(DIR_2_B, OUTPUT);    // Direction pin on channel B
  pinMode(DIR_1_A, OUTPUT);    // Direction pin on channel A
  pinMode(DIR_2_A, OUTPUT);    // Direction pin on channel A
  pinMode(LED_BUILTIN, OUTPUT);

  setPwmFrequency(9, 1024);  
  setPwmFrequency(10, 1024);
  
}

double roundValue(double d) {
  return floor(d + 0.5);
}

float scaleValue(float input, int input_start, int input_end, int output_start, int output_end) {

  double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
  float output = output_start + roundValue(slope * (input - input_start));

  return output;
}

void loop()
{
  //--------------------------------------------READ FROM MPU------------------------------------
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();
  
  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;

  //--------------------------------------------PID-----------------------------------------------
  float A = 0.9;
  
  angle += gyr.YAxis * 0.001; //6.07968E-5
  angle = angle * A + accPitch * (1 - A); 

  float dAngle = 3 * (angle) - 4 * lastAngle + 1 *lastLastAngle; // 3 -4 1
  lastLastAngle = lastAngle;
  lastAngle = angle; 

  float sum = 3  * (angle +1.8) + 10 * dAngle ; 

  float Action;

  //--------------------------------------------SCALING VALUES------------------------------------

  if (sum > 2) {
    Action = scaleValue(sum, 0, 150, 0, 255);
  } else if (sum < -2) {
    Action = scaleValue(-sum, 0, 150, 0, 255);
    Action *= -1;
  } else {
    Action = 0;
  }

  //--------------------------------------------MAKE MOVE-----------------------------------------

  if (!DEBUG) {
    if (Action > 0) {
      forward(Action);
    }
    else if (Action < 0) {
      backward(Action);
    }
    else {
      nothing();
    }
  }
}


void forward(float velocity) {
  digitalWrite(DIR_1_B, HIGH);
  digitalWrite(DIR_2_B, LOW);
  analogWrite(PWM_B, velocity);

  digitalWrite(DIR_1_A, HIGH);
  digitalWrite(DIR_2_A, LOW);
  analogWrite(PWM_A, velocity);
}

void backward(float velocity) {
  digitalWrite(DIR_1_B, LOW);
  digitalWrite(DIR_2_B, HIGH);
  analogWrite(PWM_B, 0 - velocity);

  digitalWrite(DIR_1_A, LOW);
  digitalWrite(DIR_2_A, HIGH);
  analogWrite(PWM_A, 0 - velocity);
}

void nothing() {
  analogWrite(PWM_B, 0);
  analogWrite(PWM_A, 0);  
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
