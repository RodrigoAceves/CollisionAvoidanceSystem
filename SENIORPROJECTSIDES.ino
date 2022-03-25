#include <SoftwareSerial.h> //header file of software serial port
#define RLED 4
#define YLED 5
#define SPEAKER 6
#define KLAXONctrl 7

SoftwareSerial Serial1(2,3);

// Second order Butterworth filter with cutoff frequency of 0.75
float a[3] = {1000.0, 942.80904158206336586779248280647, 333.33333333333333333333333333333};
float b[3] = {569.03559372884926492730528480024, 1138.0711874576985298546105696005, 569.03559372884926492730528480024};

//// Second order Butterworth filter with cutoff frequency of 0.9
//float a[3] = {1.0, 1.5610180758007183854374488873873, 0.641351538057563286265860824642};
//float b[3] = {0.80059240346457039017025181237841, 1.6011848069291407803405036247568, 0.80059240346457039017025181237841};

//beta = 4.2
float kaiser[2] = {0064756.5318041967515173524816418649,
 0870486.92391606485863064790464705};

float willStop;
float willStopin[3];
//Sample Period
int deltaTout[2];
float resolutionFactor = 1000;

//Store the starting time
int startTime = 0;
float dist[3]; //store Distance in cm
float distin[3];

float vel; //store velocity and velocity history
float acc;  //store acceleration
int strength; //store signal strength of LiDAR
float temprature; // store the temperature of Lidar;
int check; //save check value
int i;
int uart[9]; //save data measured by LiDAR
const int HEADER=0x59; //frame header of data package
void setup() {
  Serial.begin(9600); //Computer Serial Monitor
  Serial1.begin(115200); //LiDAR bitrate
  pinMode(RLED,OUTPUT);
  pinMode(YLED,OUTPUT);
  pinMode(SPEAKER,OUTPUT);
  pinMode(KLAXONctrl,INPUT);
  Serial.println("Program Start");
}

void loop() {
//  Serial.println(millis());
//  delay(millis()%50);//Set the time step to align with the internal clock for every 50 ms
  if (Serial1.available()) { //check if serial port has data input
    if(Serial1.read() == HEADER) { //assess data package frame header 0x59
      uart[0] = HEADER;
    if (Serial1.read() == HEADER) { //assess data package frame header 0x59
      uart[1] = HEADER;
      for (i = 2; i < 9; i++) { //save data in array
        uart[i] = Serial1.read();
      }
      check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
      if (uart[8] == (check & 0xff)){ //verify the received data as per protocol

        distin[0] = distin[1];
        distin[1] = distin[2];
        distin[2] = resolutionFactor*(uart[2] + uart[3] * 256); //distance in cm filtered multiplied by resolution

        dist[0] = dist[1];
        dist[1] = dist[2];
        dist[2] = dist[3];
        dist[3] = (distin[0]*b[0] + distin[1]*b[1] + distin[2]*b[2] - (dist[2]*a[0] + dist[1]*a[1] + dist[0]*a[2]))/1000;
        
        vel = ((dist[2] - dist[1])/(deltaTout[1])); //speed cm/s
        acc = ((dist[2] - dist[1])/(deltaTout[1]*deltaTout[1])-(dist[1] - dist[0])/(deltaTout[1]*deltaTout[0])); //acceleration cm / s^2

        //use SUVAT equations to determine stop distance...
        willStopin[0] = willStopin[1];
        willStopin[1] = willStopin[2];
        willStopin[2] = -vel*vel/(2*acc);

        willStop = (willStopin[2]*kaiser[0] + willStopin[1]*kaiser[1] + willStopin[0]*kaiser[0])/1000000;

        plotData();

        int boundary = 10; // Stoping centimeters behind the bicycle 

        if(digitalRead(KLAXONctrl)){
          digitalWrite(SPEAKER,HIGH);
          delay(500);
          digitalWrite(SPEAKER,LOW);
        }
        if((!isnan(willStop))){
          if(dist[2]- boundary*resolutionFactor - willStop > 0){// if vehicle can stop before the boundary line
            if(dist[2] >= 200*resolutionFactor){ // If the car is more than 2 meters away turn off yellow light
              digitalWrite(YLED,LOW);
            }else{
              digitalWrite(YLED,HIGH);
            }
            digitalWrite(RLED,LOW);
          } else{ //if the vehicle cannot slow down in time then turn on the red light and the speaker at 1khz for 100 milliseconds
            digitalWrite(YLED,LOW);
            digitalWrite(SPEAKER,HIGH);
            digitalWrite(RLED,HIGH);
            delay(500);
            digitalWrite(SPEAKER,LOW);
            }
        }
        samplePeriod(millis());
        delay(5);
        }
      }
    }
  }
}

void samplePeriod(int ClockTime){
  deltaTout[0] = deltaTout[1];
  deltaTout[1] = ClockTime - startTime;
  startTime = ClockTime;
}


void plotData(void){
//  strength = uart[4] + uart[5] * 256; //calculate signal strength value
//  temprature = uart[6] + uart[7] *256;//calculate chip temprature
//  temprature = temprature/8 - 256;
//  Serial.print("Strength = ");
//  Serial.print(strength); //output signal strength value
//  Serial.print("\t ChipTemprature = ");
//  Serial.print(temprature);
if(digitalRead(KLAXONctrl)){
  Serial.println("ALL KLAXONS ON");
}
  Serial.print("\t Distance: ");
  Serial.println(dist[2]/resolutionFactor);
//  Serial.print("\t Velocity: ");
//  Serial.print(vel);
//  Serial.print("\t Acceleration: ");
//  Serial.print(acc);
//  Serial.print("\t WillStop: ");
//  Serial.print(willStop);
//  Serial.print("\t SamplePeriod: ");
//  Serial.println(deltaTout[1]);
  
}
