#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
#define turnspeed 100
#define Kp 0.1 // bunu belirlemek için deney yapın, botunuzun çizgiyi yavaş bir hızda takip etmesini sağlayan küçük bir şeyle başlayın
#define Kd 0.7 // bunu belirlemek için deney yapın, hızları yavaşça artırın ve bu değeri ayarlayın. ( Not: Kp < Kd)
#define rightMaxSpeed 85 // max speed of the robot
#define leftMaxSpeed 85 // max speed of the robot
#define rightBaseSpeed 60 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 60  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

#define solMotorPWM 6
#define sagMotorPWM 11
#define solOnIleriPin 2
#define solOnGeriPin 3
#define sagOnIleriPin 10        
#define sagOnGeriPin 9
#define solArkaIleriPin 5
#define solArkaGeriPin 4
#define sagArkaIleriPin 8
#define sagArkaGeriPin 7




void setup() 
{
  pinMode(solMotorPWM,OUTPUT);
  pinMode(sagMotorPWM,OUTPUT);
  pinMode(solOnIleriPin,OUTPUT);
  pinMode(solOnGeriPin,OUTPUT);
  pinMode(sagOnIleriPin,OUTPUT);
  pinMode(sagOnGeriPin,OUTPUT);
  pinMode(solArkaIleriPin,OUTPUT);
  pinMode(solArkaGeriPin,OUTPUT);
  pinMode(sagArkaIleriPin,OUTPUT);
  pinMode(sagArkaGeriPin,OUTPUT);

  
  //Sensor ayarları
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  //qtr.setTimeout(TIMEOUT);
  //qtr.setEmitterPin(2);


  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // kalibrasyon modunda olduğumuzu belirtmek için Arduino'nun LED'ini açın

  // 2,5 ms RC okuma zaman aşımı (varsayılan) * calibrate() çağrısı başına 10 okuma
  // = calibrate() çağrısı başına ~25 ms.
  // Kalibrasyonun yaklaşık 10 saniye sürmesi için calibrate()'i 400 kez çağırın.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // kalibrasyonun bittiğini belirtmek için Arduino'nun LED'ini kapatın
  delay(500);
  Serial.begin(9600);
  /*for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // emitörler açıkken ölçülen kalibrasyon maksimum değerlerini yazdırın
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();*/
  delay(1000);
}

int lastError = 0;
int sola_donus_say = 0;
int saga_donus_say = 0;

void loop() {
  // kalibre edilmiş sensör değerlerini okuyun ve hat konumunun bir ölçüsünü elde edin
  // 0'dan 5000'e kadar (beyaz bir çizgi için bunun yerine readLineWhite() kullanın)
  uint16_t position = qtr.readLineWhite(sensorValues);
  if(sensor_say(sensorValues) <= 3)//Düz yolda gitme
  {
    // sensör değerlerini 0'dan 1000'e kadar sayılar olarak yazdırın, burada 0 maksimum anlamına gelir
    // yansıma ve 1000 minimum yansıma anlamına gelir, ardından satır gelir
    // konum
    /*for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println(position);
    delay(100);
    return;*/

    //PID Hesaplama
    int error = position - 3500;
  
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
  
    int rightMotorSpeed = rightBaseSpeed + motorSpeed;
    int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  
    /*Serial.print("Sol Motor:");
    Serial.print(leftMotorSpeed);
    Serial.print("Sağ Motor:");
    Serial.print(rightMotorSpeed);
    Serial.println();*/
  
    if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
    if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
    if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
    if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  
    digitalWrite(sagOnIleriPin, HIGH);
    digitalWrite(sagOnGeriPin, LOW);
    digitalWrite(sagArkaIleriPin, HIGH);
    digitalWrite(sagArkaGeriPin, LOW);
    analogWrite(sagMotorPWM, rightMotorSpeed);
  
    
    digitalWrite(solOnIleriPin, HIGH);
    digitalWrite(solOnGeriPin, LOW);
    digitalWrite(solArkaIleriPin, HIGH);
    digitalWrite(solArkaGeriPin, LOW);
    analogWrite(solMotorPWM, leftMotorSpeed);
  }
  
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------

  else if(sensor_say(sensorValues)>3 and solmu_sagmi(sensorValues) == "sol")//Sola dönme kodu
  {
    if (sola_donus_say > 0)
    {
      digitalWrite(sagOnIleriPin, HIGH);
      digitalWrite(sagOnGeriPin, LOW);
      digitalWrite(sagArkaIleriPin, HIGH);
      digitalWrite(sagArkaGeriPin, LOW);
      analogWrite(sagMotorPWM, turnspeed);
    
      
      digitalWrite(solOnIleriPin, LOW);
      digitalWrite(solOnGeriPin, HIGH);
      digitalWrite(solArkaIleriPin, LOW);
      digitalWrite(solArkaGeriPin, HIGH);
      analogWrite(solMotorPWM, turnspeed);
      delay(350);
    }
    sola_donus_say += 1;
    return;
  }

  

  else if (sensor_say(sensorValues)>3 and solmu_sagmi(sensorValues) == "sag")//Sağ dönme kodu
   {
    digitalWrite(sagOnIleriPin, LOW);
    digitalWrite(sagOnGeriPin, HIGH);
    digitalWrite(sagArkaIleriPin, LOW);
    digitalWrite(sagArkaGeriPin, HIGH);
    analogWrite(sagMotorPWM, turnspeed);
  
    
    digitalWrite(solOnIleriPin, HIGH);
    digitalWrite(solOnGeriPin, LOW);
    digitalWrite(solArkaIleriPin, HIGH);
    digitalWrite(solArkaGeriPin, LOW);
    analogWrite(solMotorPWM, turnspeed);
    delay(350);
    saga_donus_say += 1;
    return;
  }

//--------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------- 

}//Loop ana Program sonu



//Alt Programlar
uint16_t sensor_say(uint16_t sV[SensorCount])
{
  uint16_t say = 0;
  for (uint8_t i = 0; i < SensorCount; i++)
    if(sV[i] < 500) say++;
  return say;
}

String solmu_sagmi(uint16_t sV[SensorCount])
{
  String string_yon = "";
  int sol_toplam = 0;
  int sag_toplam = 0;
  
  
  for (uint8_t i = 0; i < int(SensorCount/2); i++)
    sol_toplam += sV[i];
  for (uint8_t i = (SensorCount/2); i < (SensorCount); i++)
    sag_toplam += sV[i];
    
  if (sol_toplam < sag_toplam)
    string_yon = "sag";
    
  else if (sol_toplam > sag_toplam)
    string_yon = "sol";
  return string_yon;
}
