#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>
 
MPU6050 mpu;
 
// Konfiguracja filtru Kalmana dla osi X i Y (kat, odchylka, pomiar)
KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);
 
// Obliczone wartosci Pitch i Roll tylko z akcelerometru
float accPitch = 0;
float accRoll = 0;
 
// Obliczone wartosci Pitch i Roll z uwzglednieniem filtru Kalmana i zyroskopu
float kalPitch = 0;
float kalRoll = 0;

const int 
PWM_A   = 5,DIR_1_A   = 10,DIR_2_A   = 9,BRAKE_A = 9,
PWM_B   = 3,DIR_1_B   = 7,DIR_2_B   = 8,BRAKE_B = 8;

float DEBUG =1;   // 0 false, 1 true

float angle=0, lastAngle=0, lastLastAngle=0;
 
void setup(){
  Serial.begin(115200);
  Serial.println("Inicjalizacja MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Nie znaleziono MPU6050!");
    delay(500);
  }
  mpu.calibrateGyro();

  if(!DEBUG) delay(10000);

  pinMode(DIR_1_B, OUTPUT);    // Direction pin on channel B  
  pinMode(DIR_2_B, OUTPUT);    // Direction pin on channel B 
  pinMode(DIR_1_A, OUTPUT);    // Direction pin on channel A  
  pinMode(DIR_2_A, OUTPUT);    // Direction pin on channel A 
  pinMode(LED_BUILTIN, OUTPUT);
}

double roundValue(double d){
    return floor(d + 0.5);
}

float scaleValue(float input, int input_start, int input_end, int output_start, int output_end){ 

  double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
  float output = output_start + roundValue(slope * (input - input_start));

  return output;
}
 
void loop()
{
  //--------------------------------------------READ FROM MPU------------------------------------
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI;
  kalRoll = kalmanX.update(accRoll, gyr.XAxis);

  //--------------------------------------------PID-----------------------------------------------
  float A= 0.99;
  angle += gyr.XAxis * 0.001;
  angle = angle * A + accRoll*(1-A);

  float dAngle = 3*angle - 4*lastAngle + lastLastAngle;
  lastLastAngle = lastAngle;
  lastAngle = angle;
  
  float sum = 0.2 * angle + 3 * dAngle;
  // accRoll 
  // accRoll + gyr.XAxis
  // accRoll + deriv(gyr.XAxis)
  // ... + kalman
  
  float Action;

  //--------------------------------------------SCALING VALUES------------------------------------
  
  if(sum>0){
    Action = scaleValue(sum, 0, 15, 85, 255);
  }else if(sum<0){
    Action = scaleValue(-sum, 0, 15, 85, 255);
    Action *=-1;
  }else{
    Action =0;
  }
  Serial.println(sum);
  Serial.print("      angle");
  Serial.println(angle);
  //--------------------------------------------MAKE MOVE-----------------------------------------

  if(!DEBUG){
    if (Action > 0) {
      digitalWrite(LED_BUILTIN, HIGH);
      forward(Action);
    }
    else if (Action < 0) {
      digitalWrite(LED_BUILTIN, HIGH);
      backward(Action);
    }
    else {
      digitalWrite(LED_BUILTIN, LOW);
      nothing();
    }
  }  

  //--------------------------------------------DEBUG---------------------------------------------
  
  //kalRoll - zwraca dobry kąt, gyr.XAxis - zwraca poprawną prędkośc ( dla dodatniego kąta przechylenie wbrew niemu jest z minusem)
  // accRoll zwraca kąt o wiele szybciej, kalRoll jest dużo wolniejszy - może używa go tylko do poprawiania
  
  if(DEBUG){
//    Serial.print("GYRO = ");  
//    Serial.print(gyr.XAxis);
//    
//    Serial.print("ACC = ");  
//    Serial.print(acc.XAxis);
//    Serial.print("ACC 2= ");  
//    Serial.print(accRoll);
//    
//    Serial.print(" (K)Roll = ");
//    Serial.print(kalRoll); 
//    Serial.println();
  }
  
}


void forward(float velocity){
  digitalWrite(DIR_1_B, HIGH); 
  digitalWrite(DIR_2_B, LOW); 
  analogWrite(PWM_B, velocity);

  digitalWrite(DIR_1_A, HIGH); 
  digitalWrite(DIR_2_A, LOW); 
  analogWrite(PWM_A, velocity);
}

void backward(float velocity){
  digitalWrite(DIR_1_B, LOW); 
  digitalWrite(DIR_2_B, HIGH); 
  analogWrite(PWM_B, 0 - velocity);

  digitalWrite(DIR_1_A, LOW); 
  digitalWrite(DIR_2_A, HIGH); 
  analogWrite(PWM_A, 0 - velocity);
}

void nothing(){
  analogWrite(PWM_B, 0);
  analogWrite(PWM_A, 0);
} 
