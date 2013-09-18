/*
 Here I'm using DDS (direct digital synthesis)
 to make a metronome that actually sounds okay.
 And then we capture a signal as usual (e.g. acceleration and pressure)
 
 Much of the DDS part comes from:
 http://mouro.info/signal-generator-using-arduino-and-dds/
*/


/*
* DDS - Direct digital synthesis example for sinewave generation.
*
* The implementation of the DDS relies upon integer
* arithmetic. The size of accumulator is N-bits.
* Assuming that the period of the output signal is
* 2*pi rad, the maximum phase is represented by 2^N.
* During one sample period the phase increases by the
* phase increment which lead us to:
* - outputFrequency(phaseIncr) = (samplingFrequency / 2^N) * phaseIncr  (1)
*
* The later can be rewritten:
* - phaseIncr = outputFrequency * resolution
*
* where resolution is:
* - resolution = 2^N / samplingFrequency  (2)
*
* Also:
* - N=log2(samplingFrequency / stepFrequency)+0.5 (3)
* - maximumOutputFrequency = samplingFrequency / 2
*
* For a better analog reconstruction of the signal we use:
* - maximumOutputFrequency=(samplingFrequency / 4) (4)
*
* According to (4) and for a maximum output frequency of
* 15KHz, a 62.5KHz sampling frequency was chosen.
* with (3) we can derive the accumulator as 16 bits wide
* (for sake of commodity), and according to (2) we'll
* have a resolution of 1.0486. Since we're using fixed
* point arithmetic, we'll scale the resolution by a
* factor of 2^16 (to maintain some accuracy):
* - N=16, resolution=(2^16 * maximumOutputFrequency)  /  (samplingFrequency)
*
* The minimum frequency generated is for phaseIncr=1 :
* - minimumFrequency = samplingFrequency/2^N
*/

#include <avr/pgmspace.h>
#include <avr/sleep.h>





// Sinewave lookup table for waveform generation
static const uint8_t  sineTable[] PROGMEM =
{
0x80,0x83,0x86,0x89,0x8c,0x8f,0x92,0x95,
0x98,0x9c,0x9f,0xa2,0xa5,0xa8,0xab,0xae,
0xb0,0xb3,0xb6,0xb9,0xbc,0xbf,0xc1,0xc4,
0xc7,0xc9,0xcc,0xce,0xd1,0xd3,0xd5,0xd8,
0xda,0xdc,0xde,0xe0,0xe2,0xe4,0xe6,0xe8,
0xea,0xec,0xed,0xef,0xf0,0xf2,0xf3,0xf5,
0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfc,
0xfd,0xfe,0xfe,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xfe,0xfe,
0xfd,0xfc,0xfc,0xfb,0xfa,0xf9,0xf8,0xf7,
0xf6,0xf5,0xf3,0xf2,0xf0,0xef,0xed,0xec,
0xea,0xe8,0xe6,0xe4,0xe2,0xe0,0xde,0xdc,
0xda,0xd8,0xd5,0xd3,0xd1,0xce,0xcc,0xc9,
0xc7,0xc4,0xc1,0xbf,0xbc,0xb9,0xb6,0xb3,
0xb0,0xae,0xab,0xa8,0xa5,0xa2,0x9f,0x9c,
0x98,0x95,0x92,0x8f,0x8c,0x89,0x86,0x83,
0x80,0x7c,0x79,0x76,0x73,0x70,0x6d,0x6a,
0x67,0x63,0x60,0x5d,0x5a,0x57,0x54,0x51,
0x4f,0x4c,0x49,0x46,0x43,0x40,0x3e,0x3b,
0x38,0x36,0x33,0x31,0x2e,0x2c,0x2a,0x27,
0x25,0x23,0x21,0x1f,0x1d,0x1b,0x19,0x17,
0x15,0x13,0x12,0x10,0x0f,0x0d,0x0c,0x0a,
0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x03,
0x02,0x01,0x01,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,
0x02,0x03,0x03,0x04,0x05,0x06,0x07,0x08,
0x09,0x0a,0x0c,0x0d,0x0f,0x10,0x12,0x13,
0x15,0x17,0x19,0x1b,0x1d,0x1f,0x21,0x23,
0x25,0x27,0x2a,0x2c,0x2e,0x31,0x33,0x36,
0x38,0x3b,0x3e,0x40,0x43,0x46,0x49,0x4c,
0x4f,0x51,0x54,0x57,0x5a,0x5d,0x60,0x63,
0x67,0x6a,0x6d,0x70,0x73,0x76,0x79,0x7c
};

// PWM output (OCR1A)
int pwmPin = 9;

// 16 bit accumulator
uint16_t phaseAccumulator = 0;
// 16 bit delta
uint16_t phaseIncrement = 0;
// DDS resolution
const uint32_t resolution =  68719;
// wavetable lookup index(upper 8 bits of the accumulator)
uint8_t index = 0;
// Desired output frequency (let's set it to 440Hz)
uint16_t frequency = 440;


// TIMER1 will overflow at a 62.5KHz(Sampling frequency).
// Updates the OCR1A value and the accumulator.
// Computes the next sample to be sent to the PWM.
ISR(TIMER1_OVF_vect)
{
  static uint8_t osc = 0;

  // Send oscillator output to PWM
  OCR1A = osc;

  // Update accumulator
  phaseAccumulator += phaseIncrement;
  index = phaseAccumulator >> 8;

  // Read oscillator value for next interrupt
  osc = pgm_read_byte( &sineTable[index] );

}

// Configures TIMER1 to fast PWM non inverted mode.
// Prescaler set to 1, which means that timer overflows
// every 16MHz/256 = 62.5KHz
void initPWM(void)
{
  // Set PORTB1 pin as output
  pinMode(pwmPin, OUTPUT);

  // 8-bit Fast PWM - non inverted PWM
  TCCR1A= _BV(COM1A1) | _BV(WGM10);

  // Start timer without prescaler
  TCCR1B = _BV(CS10) | _BV(WGM12);

  // Enable overflow interrupt for OCR1A
  TIMSK1 = _BV(TOIE1);

}

// Translates the desired output frequency to a phase
// increment to be used with the phase accumulator.
// The 16 bit shift is required to remove the  2^16
// scale factor of the resolution.
void setFrequency( uint32_t frequency )
{
  uint64_t phaseIncr64 =  resolution * frequency;
  phaseIncrement = phaseIncr64 >> 16;
}






/* Stuff related to sending the binary */


void sendBinary(int value) 
// Send a binary value directly (without conversion to string)
// based on http://my.safaribooksonline.com/book/hobbies/9781449399368/serial-communications/sending_binary_data_from_arduino#X2ludGVybmFsX0ZsYXNoUmVhZGVyP3htbGlkPTk3ODE0NDkzOTkzNjgvMTAy
{
  Serial.print(lowByte(value));
  Serial.print(highByte(value));
}








const int groundpin = 18;             // analog input pin 4 -- ground
const int powerpin = 19;              // analog input pin 5 -- voltage
const int xpin = 3;                  // x-axis of the accelerometer  (in the original source this used to read xpin=A3 but that gave an error
const int ypin = 2;                  // y-axis
const int zpin = 1;                  // z-axis
const int pushpin = 0;                // where the pressure sensor is (bad pun, I know)




// The duration of one metronome tick (in microsec)
const unsigned long TICKDURATION = 20000;   // 10msec


// The interval of the metronome (in microsec)
//const unsigned long TICKINTERVAL = 190000; // 1 sec

// Here we need to be careful. The tick interval is the interval between the offset of
// one tick and the onset of the next. That means, we need to subtract the tick
// duration to find the real IOI.
const unsigned long TICKINTERVAL = 900000-TICKDURATION; // 1 sec
//const unsigned long TICKINTERVAL = 190000; // 1 sec



//const int TICKINTERVAL = 1000;

// WARNING! This is the interval between the off- of one tick and the on of the next. 


// The time in microsec
unsigned long t;

// The pressure sensor reading
int p=0;

// The accelerometer reading
int x=0;
int y=0;
int z=0;



// This is part of a "hack" to make sure the right thing
// happens when time passes zero (i.e. when the int size overflows)
boolean armed;


boolean isOn;


// When next to tick (in microsec)
long nexttickt=0;


void tickOn(){
  setFrequency( 440 );
}


void tickOff() {
  setFrequency( .05 ); // so that you practically don't hear anything
}




void setup(void)
{

  // Enable us to read in the FSR input
  pinMode(pushpin, INPUT);
  
  // initialize the serial communications:
  //  Serial.begin(1000000); // very high speed, fasten your seatbelts
  Serial.begin(1000000);
  //Serial.begin(9600); // for debug

  
  // To make sure that the groundpin where the accelerometer is connected (pin 4 I think)
  // is actually ground (i.e. no voltage).
  pinMode(groundpin, OUTPUT);
  digitalWrite(groundpin, LOW);


  // Initialise fast PWM
  initPWM();

  // Enable sleep mode
  set_sleep_mode(SLEEP_MODE_IDLE);

  // Set phase increment according to
  // desired output frequency
  tickOff();

  // Enable global interrupts
  sei();
  
  //  armedOn=true;
  //armedOff=true;
  nexttickt = micros()+TICKINTERVAL;
  //Serial.print("Next at ");
  //Serial.println(nexttickt);
  
  isOn = false;
  armed = true;


}



int intVal(bool b){
  if (b) { return 1; }
  return 0;
}



void loop()
{

  t = micros();
  //  Serial.println(t);
  
  // Below is a little hack that makes sure the right thing happens
  // when time goes through zero (i.e. because we overflow the int range)
  //  if (t<nexttickt) { armed=true; }
  
  if (armed && t>=nexttickt) {
    if (isOn) {
      // switch off
      tickOff();
      isOn=false;
      nexttickt  = nexttickt+TICKINTERVAL;
      
    } else {
      // switch on
      tickOn();
      isOn=true;
      nexttickt  = nexttickt+TICKDURATION;
    
    }
    //Serial.print("Next at ");
    //Serial.println(nexttickt);
    //armed=false;
    
  }

  // Read the pressure
  p = analogRead(pushpin);

  x = analogRead(xpin);
  y = analogRead(ypin);
  z = analogRead(zpin);


  // For debug output
  /*
  Serial.print("B"); // indication that we start
  Serial.print(" ");
  Serial.print(t);
  Serial.print(" ");
  Serial.print(p);
  Serial.print(" ");
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(z);
  Serial.print(" ");
  if (isOn) { Serial.print("1"); } else { Serial.print("0"); }
  Serial.print(" ");
  Serial.print("E");
  Serial.println();
  */

  // For production output
  Serial.print("B");
  sendBinary(t);
  sendBinary(p);     // this will take two bytes
  sendBinary(x);
  sendBinary(y);
  sendBinary(z);
  if (isOn) { Serial.print("1"); } else { Serial.print("0"); }
  Serial.print("E"); // indication of end
  
  //delay(1);
  
}

