/* Apparature for photography.
    Usage: This apparature allows for taking photos perfectly parallel
    to a surface, such a wall or a cloth

    Created by Evangelos C. Fotopoulos
    on 2019/07/14

*/

// Includes
#include <Servo.h> // Adding library for servo control
#include <DHT.h>
#include <DHT_U.h>

// Defining pins for easy reference
//#define dht_pin A0
#define DHTPIN 2
#define trigPin1 4
#define echoPin1 5
#define trigPin2 6
#define echoPin2 7
#define trigPin3 8
#define echoPin3 9
#define trigPin4 10
#define echoPin4 11
#define horizPinS 12
#define vertPinS 13

#define DHTTYPE DHT22

#define TOL 0.5

Servo horizontal, vertical; // Creating servos
int servov = 90, servoh = 90;

// Initialize DHT sensor.
DHT ths(DHTPIN, DHTTYPE);

// Decalring variables
int pos;
float temp, humid;

// Setting up the variables for the fromula 
// from http://gsd.ime.usp.br/~yili/SpeedOfSound/Speed.html 
// to evaluate speed of sound
float p = 101000;     //Set atmospheric pressure to 101.000 kPa
float a0 = 331.5024;
float a1 = 0.603055;
float a2 = -0.000528;
float a3 = 51.471935;
float a4 = 0.1495874;
float a5 = -0.000782;
float a6 = -1.82e-7;       
float a7 = 3.73e-8;         
float a8 = -2.93e-10;     
float a9 = -85.20931;
float a10 = -0.228525;
float a11 = 5.91e-5;  
float a12 = -2.835149;
float a13 = -2.15e-13; 
float a14 = 29.179762;
float a15 = 0.000486;
float T, h, f, Psv, Xw, c, Xc, speedOfSound;

// Distances metered
float distBL, distTL, distTR, distBR;
float avg_top, avg_btm, avg_left, avg_right, dvert, dhoriz;

void setup() {

  // Setting up the sensors
  ths.begin();

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);
  
  // Setting up the servos
  horizontal.attach(horizPinS);
  vertical.attach(vertPinS);
  horizontal.write(servoh);
  vertical.write(servov);

  // Starting serial
  Serial.begin(9600);
}

void loop() {
  temp = ths.readTemperature();   //Get temperature from sensor  
  humid = ths.readHumidity();  //Get humidity from sensor

  // Calculate the speed of sound at the moment
  T = temp + 273.15;
  h = humid /100.0;
  f = 1.00062 + 0.0000000314 * p + 0.00000056 * temp * temp;
  Psv = exp(0.000012811805 * T * T - 0.019509874 * T + 34.04926034 - 6353.6311 / T);
  Xw = h * f * Psv / p;
  c = 331.45 - a0 - p * a6 - a13 * p * p;
  c = sqrt(a9 * a9 + 4 * a14 * c);
  Xc = ((-1) * a9 - c) / ( 2 * a14);
  speedOfSound = a0 + a1 * temp + a2 * temp * temp + (a3 + a4 * temp + a5 * temp * temp) * Xw + (a6 + a7 * temp + a8 * temp * temp) * p + (a9 + a10 * temp + a11 * temp * temp) * Xc + a12 * Xw * Xw + a13 * p * p + a14 * Xc * Xc + a15 * Xw * p * Xc;
  
  // Read values of the distance sensors
  distBL = readDistancefromSensor(trigPin1, echoPin1);
  distTL = readDistancefromSensor(trigPin2, echoPin2);
  distTR = readDistancefromSensor(trigPin3, echoPin3);
  distBR = readDistancefromSensor(trigPin4, echoPin4);

  // Calculate averages
  avg_top = (distTL + distTR)/2;
  avg_btm = (distBL + distBR)/2;
  avg_left = (distTL + distBL)/2;
  avg_right = (distTR + distBR)/2;

  // Check the differences
  dvert = avg_top - avg_btm; // check the diffirence of up and down
  dhoriz = avg_left - avg_right;// check the diffirence og left and right

  // Moving vertically
  if (-1*TOL > dvert || dvert > TOL){ // check if the diffirence is in the tolerance else change vertical angle
    if (avg_top > avg_btm){
      servov++;
      if (servov > 180){
        servov = 180;
      }
    }
    else if (avg_top < avg_btm){
      servov--;
      if (servov < 0){
        servov = 0;
      }
    }
    vertical.write(servov);
  }

  // Moving vertically
  if (-1*TOL > dhoriz || dhoriz > TOL){ // check if the diffirence is in the tolerance else change vertical angle
    if (avg_left > avg_right){
      servoh++;
      if (servoh > 180){
        servoh = 180;
      }
    }
    else if (avg_left < avg_right){
      servoh--;
      if (servoh < 0){
        servoh = 0;
      }
    }
    horizontal.write(servoh);
  }
  
  Serial.print(temp);
  Serial.print(", ");
  Serial.print(humid);
  Serial.print(", ");
  Serial.print(avg_top);
  Serial.print(", ");
  Serial.print(avg_btm);
  Serial.print(", ");
  Serial.print(avg_left);
  Serial.print(", ");
  Serial.print(avg_right);
  Serial.print(", ");
  Serial.print(dvert);
  Serial.print(", ");
  Serial.print(dhoriz);
  Serial.print("\n");

  delay(10);

}

float readDistancefromSensor(int T_PIN, int E_PIN){
    //Send a short (10 microseconds) ultrasonic burst 
  digitalWrite(T_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(T_PIN, LOW);
  
  float microseconds = pulseIn(E_PIN, HIGH, 100000); //Mesure the duration of a HIGH pulse in echo pin in microseconds. Timeout in 0,1 seconds
  float seconds = microseconds / 1000000;            //Convert microseconds to seconds
  float meters = seconds * speedOfSound;             //Get the distance in meters using the speed of sound calculated earlier
  float cm = meters * 100;                           //Convert meters to cm
  cm = cm/2;    //We only want the distance to the obstacle and not the roundtrip
  return cm;
}
