// Keith Marsh April 2015
// Use two microphones to detect the Interaural Phase Difference and calculate the direction.
// I use an Arduino mega because of the memory usage, but reduce SAMPLE_LEN to 150 and it'll
// work okay on an Uno.  You can also increase the Prescaler to 32 if you want to get more 
// samples in at lower resolution.
// http://en.wikipedia.org/wiki/Interaural_time_difference
// http://www.adafruit.com/products/1713

// If you don't have a 16x2 i2c LCD, change the 1 to a 0 below
#define USE_LCD  (1)

#if USE_LCD
// Reference: https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#endif


// Various ADC prescalers
const unsigned char PS_16  = (1 << ADPS2);
const unsigned char PS_32  = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64  = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

const unsigned int SAMPLE_LEN = 200;
const unsigned int CROSS_COUNT = 5;
const unsigned int AVG_COUNT = 10;

const int mid = 290;
const unsigned int LOOK_BACK = 4;
const unsigned int LOOK_FWD  =4;
const float MIN_SLOPE = 0.2f;
const float MAX_IPD_USEC = 500;

// Sample data and timestamp in micro sec
short left[SAMPLE_LEN];
short right[SAMPLE_LEN];
unsigned long time[SAMPLE_LEN];

// timestamps of rising crossing of x-axis
float lcross[CROSS_COUNT];
float rcross[CROSS_COUNT];

// last n differences between left and right crossings
float ipd[AVG_COUNT];

// holds the index for the next crossing trimestamps in idp
unsigned int gIpdIx;

short min;
short max;

// https://arduino-info.wikispaces.com/LCD-Blue-I2C#v1
LiquidCrystal_I2C lcdi2c(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
LCD *lcd = &lcdi2c;


// Setup the serial port and pin 2
void setup() {
  Serial.begin(115200);
  
#if USE_LCD
  lcd->begin(16,2);
  lcd->clear();
#endif

  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(13, OUTPUT);
  
  // My thanks go to Guy van den Berg for a useful tutorial on faster sampling
  // http://www.microsmart.co.za/technical/2014/03/01/advanced-arduino-adc/
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library
  ADCSRA |= PS_16;

  // clear down the average array
  for (unsigned int aIx = 0; aIx < AVG_COUNT; aIx++) {
    ipd[aIx] = 0.0f;
  }
}

void loop() {
  digitalWrite(13, HIGH);
  captureStereo(); 
  digitalWrite(13, LOW);
  int amplitude = cleanStereoData();
  detectRisingCrossing();
  if (amplitude > 150) {
    printStereoCSV();
  }
  float average = calcAverageIPD();
  if (amplitude > 150) {
    printIPDs();
    Serial.println(average);
  }
  displayLCD(average, amplitude);
  delay(250);
}

void captureStereo() {
  // capture the values to memory
  short *l = left;
  short *r = right;
  unsigned long *t = time;
  for ( unsigned int ix = 0; ix < SAMPLE_LEN; ix++ ) {
    *l++ = analogRead(1);
    *r++ = analogRead(2);
    *t++ = micros();
  }
}

int cleanStereoData() {
  min = max = 0;
  unsigned long start = time[0];
  for ( unsigned int ix = 0; ix < SAMPLE_LEN; ix++ ) {
    time[ix] = time[ix] - start;  // make rime relative to start;
    left[ix] = left[ix] - mid;
    right[ix] = right[ix] - mid;
    if (left[ix] > max) max = left[ix];
    if (right[ix] > max) max = right[ix];
    if (left[ix] < min) min = left[ix];
    if (right[ix] < min) min = right[ix];
  }
  for ( unsigned int ix = 0; ix < CROSS_COUNT; ix++ ) {
    lcross[ix] = 0.0f;
    rcross[ix] = 0.0f;
  }
  return max - min;
}

void detectRisingCrossing() {
    // detect crossing x
  float slope, item;
  unsigned int lcrossIx = 0;
  unsigned int rcrossIx = 0;
  for ( unsigned int ix = LOOK_BACK; ix < SAMPLE_LEN-LOOK_FWD; ix++ ) {
    // look for the crossing by checking an earlier sample is -ve and a later one is +ve to filter out noise
    if ( lcrossIx < CROSS_COUNT && left[ix-LOOK_BACK] < 0 && left[ix] <= 0 && left[ix+1] > 0 && left[ix+LOOK_FWD] > 0 ) {
      // take the slope of the line from the earler to the later samples.
      slope = (float)(left[ix+LOOK_FWD] - left[ix-LOOK_BACK]) / (float)(time[ix+LOOK_FWD] - time[ix-LOOK_BACK]);
      // ignore slow rising crossings.  They're normally glitches
      if (slope > MIN_SLOPE) {
        // work out the precise point on the x-axis where the wave crosses ( that's 0 = m.x + c )
        lcross[lcrossIx] = time[ix] - left[ix] / slope;
        // if the other channel has a crossing, make sure they're close to each other.
        if ( rcross[lcrossIx] != 0.0f && abs(rcross[lcrossIx] - lcross[lcrossIx]) > MAX_IPD_USEC ) {
          // they're not so ignore them both
          rcrossIx = lcrossIx;
          rcross[rcrossIx] = lcross[lcrossIx] = 0.0f;
        } else {
          lcrossIx++;
        }
      }
    }
    if ( rcrossIx < CROSS_COUNT && right[ix-LOOK_BACK] < 0 && right[ix] <= 0 && right[ix+1] > 0 && right[ix+LOOK_FWD] > 0 ) {
      slope = (float)(right[ix+LOOK_FWD] - right[ix-LOOK_BACK]) / (float)(time[ix+LOOK_FWD] - time[ix-LOOK_BACK]);
      if (slope > MIN_SLOPE) {
        rcross[rcrossIx] = time[ix] - right[ix] / slope - 20; // Correct the fact that right is sampled after left;
        if ( lcross[rcrossIx] != 0.0f && abs(lcross[rcrossIx] - rcross[rcrossIx]) > MAX_IPD_USEC ) {
          // gash
          lcrossIx = rcrossIx;
          lcross[lcrossIx] = rcross[rcrossIx] = 0.0f;
        } else {
          rcrossIx++;
        }
      }
    }
  }
}

void printStereoCSV() {
  Serial.println("t\tL\tR"); 
  for(unsigned int ix = 0; ix < SAMPLE_LEN; ix++ ) {
    Serial.print(time[ix]);
    Serial.print(".0\t");
    Serial.print(left[ix]);
    Serial.print("\t");
    Serial.println(right[ix]);
  }
}

float calcAverageIPD() {
  for ( unsigned int cIx = 0; cIx < CROSS_COUNT; cIx++) {
    if (lcross[cIx] > 0.0f && rcross[cIx] > 0.0f) {
      ipd[gIpdIx++] = lcross[cIx] - rcross[cIx];
      if (gIpdIx > AVG_COUNT) {
        gIpdIx = 0;
      }
    }
  }
  float sum = 0.0f;
  for (unsigned int aIx = 0; aIx < AVG_COUNT; aIx++) {
    sum += ipd[aIx];
  }
  return sum / AVG_COUNT;
}

void printIPDs() {
  for ( unsigned int cIx = 0; cIx < CROSS_COUNT; cIx++) {
    if (lcross[cIx] > 0.0f && rcross[cIx] > 0.0f) {
      Serial.print(lcross[cIx] - rcross[cIx]);
      Serial.print("\t");
      Serial.print(lcross[cIx]);
      Serial.print("\t");
      Serial.println(rcross[cIx]);
    }
  }
}

void displayLCD(float averageIPD, int amplitude) {
#if USE_LCD
  lcd->clear();
  lcd->print(averageIPD);
  lcd->setCursor(9, 0);
  lcd->print(amplitude);
  lcd->setCursor(0, 1);
  int col = constrain((255 + (int)averageIPD) >> 5, 0, 15);
  for (int colIx = 0; colIx < 16; colIx++) {
    if (colIx == col) {
      lcd->print("V");
    } else {
      lcd->print(" ");
    }
  }
#endif
}


