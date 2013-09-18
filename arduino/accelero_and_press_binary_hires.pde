/*
Ths Code reads data from the  ADXL335 axis accelerometer
and at the same time from the FSR pressure sensor

Based on http://michaelabrahamsen.com/10/2010/using-the-adxl335-accelerometer-with-arduino/
and http://www.ladyada.net/learn/sensors/fsr.html

*/
const int groundpin = 18;             // analog input pin 4 -- ground
const int powerpin = 19;              // analog input pin 5 -- voltage
const int xpin = 3;                  // x-axis of the accelerometer  (in the original source this used to read xpin=A3 but that gave an error
const int ypin = 2;                  // y-axis
const int zpin = 1;                  // z-axis
const int pushpin = 0;                // where the pressure sensor is (bad pun, I know)


int x=0;
int y=0;
int z=0;
int p=0; // the push force
unsigned long t=0;
unsigned long prevt=-1; // the previous time value

void setup()
{
  // initialize the serial communications:
  Serial.begin(1000000); // very high speed, fasten your seatbelts
  
  // Power and ground the necessary pins. Providing power to both
  // the analog and digital pins allow me to just use the breakout
  // board and not have to use the normal 5V and GND pins
  pinMode (groundpin, OUTPUT);
  pinMode (powerpin,  OUTPUT);
  digitalWrite(groundpin, LOW);
  digitalWrite(powerpin, HIGH);
  analogReference(DEFAULT); // use the default reference (5V)
  
}


void sendBinary(int value) 
// Send a binary value directly (without conversion to string)
// based on http://my.safaribooksonline.com/book/hobbies/9781449399368/serial-communications/sending_binary_data_from_arduino#X2ludGVybmFsX0ZsYXNoUmVhZGVyP3htbGlkPTk3ODE0NDkzOTkzNjgvMTAy
{
  Serial.print(lowByte(value));
  Serial.print(highByte(value));
}


void loop()
{
  // print values that are recieved from the sensors and put a tab between
  // the values
  

  t = micros(); // we use arduino's timekeeper
  // Thanks to http://balau82.wordpress.com/2011/03/26/capturing-an-analog-signal-with-arduino-and-python/

  // based on informal observation, it seems I get about
  // 3000Hz this way.

  //x = analogRead(xpin);
  //y = analogRead(ypin);
  z = analogRead(zpin);
  p = analogRead(pushpin);

  Serial.print("B"); // indication that we start
  //Serial.print(z);
  //Serial.print(p);
  sendBinary(t);
  sendBinary(z);     // this will take two bytes
  sendBinary(p);     // this will take two bytes
  Serial.print("E"); // indication of end

  //Serial.println();
  
}
