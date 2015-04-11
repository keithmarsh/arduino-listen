#define SHOW_CROSS 1
// Arrays to save our results in
const unsigned int COUNT = 180;
const unsigned int ICOUNT = 5;
const unsigned int ACOUNT = 10;
short left[COUNT];
short right[COUNT];
unsigned long time[COUNT];
float lcross[ICOUNT];
float rcross[ICOUNT];
float diff[ACOUNT];
unsigned int diffIx;
const int mid = 290;
const unsigned int LOOK_BACK = 4;
const unsigned int LOOK_FWD  =4;
const float MIN_SLOPE = 0.2f;
const float MAX_DIFF_USEC = 500;

// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

short min;
short max;

// Setup the serial port and pin 2
void setup() {
  Serial.begin(57600);
  
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(13, OUTPUT);
  
    // set up the ADC
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library

  // you can choose a prescaler from above.
  // PS_16, PS_32, PS_64 or PS_128
  ADCSRA |= PS_32;    // set our own prescaler to 64 

  for (unsigned int aIx = 0; aIx < ACOUNT; aIx++) {
    diff[aIx] = 0.0f;
  }
}

void loop() { 
  digitalWrite(13, HIGH);

  // capture the values to memory
  short *l = left;
  short *r = right;
  unsigned long *t = time;
  for ( unsigned int ix = 0; ix < COUNT; ix++ ) {
    *l++ = analogRead(1);
    *r++ = analogRead(2);
    *t++ = micros();
  }
  
  digitalWrite(13, LOW);

  // clean the data
  min = max = 0;
  unsigned long start = time[0];
  for ( unsigned int ix = 0; ix < COUNT; ix++ ) {
    time[ix] = time[ix] - start;  // make rime relative to start;
    left[ix] = left[ix] - mid;
    right[ix] = right[ix] - mid;
    if (left[ix] > max) max = left[ix];
    if (right[ix] > max) max = right[ix];
    if (left[ix] < min) min = left[ix];
    if (right[ix] < min) min = right[ix];
  }
  for ( unsigned int ix = 0; ix < ICOUNT; ix++ ) {
    lcross[ix] = 0.0f;
    rcross[ix] = 0.0f;
  }
  
  // detect crossing x
  float slope, item;
  unsigned int lcrossIx = 0;
  unsigned int rcrossIx = 0;
  for ( unsigned int ix = LOOK_BACK; ix < COUNT-LOOK_FWD; ix++ ) {
    if ( lcrossIx < ICOUNT && left[ix-LOOK_BACK] < 0 && left[ix] <= 0 && left[ix+LOOK_FWD] > 0 ) {
      slope = (float)(left[ix+LOOK_FWD] - left[ix-LOOK_BACK]) / (float)(time[ix+LOOK_FWD] - time[ix-LOOK_BACK]);
      if (slope > MIN_SLOPE) {
        lcross[lcrossIx] = time[ix] - left[ix] / slope;
        if ( rcross[lcrossIx] != 0.0f && abs(rcross[lcrossIx] - lcross[lcrossIx]) > MAX_DIFF_USEC ) {
          // gash
          rcrossIx = lcrossIx;
          rcross[rcrossIx] = lcross[lcrossIx] = 0.0f;
        } else {
          lcrossIx++;
        }
      }
    }
    if ( rcrossIx < ICOUNT && right[ix-LOOK_BACK] < 0 && right[ix] <= 0 && right[ix+LOOK_FWD] > 0 ) {
      slope = (float)(right[ix+LOOK_FWD] - right[ix-LOOK_BACK]) / (float)(time[ix+LOOK_FWD] - time[ix-LOOK_BACK]);
      if (slope > MIN_SLOPE) {
        rcross[rcrossIx] = time[ix] - right[ix] / slope;
        if ( lcross[rcrossIx] != 0.0f && abs(lcross[rcrossIx] - rcross[rcrossIx]) > MAX_DIFF_USEC ) {
          // gash
          lcrossIx = rcrossIx;
          lcross[lcrossIx] = rcross[rcrossIx] = 0.0f;
        } else {
          rcrossIx++;
        }
      }
    }
  }

  // print out the results

  if (max - min > 150) {
#if SHOW_WAVE  
    Serial.println("t\tL\tR"); 
    for(unsigned int ix = 0; ix < COUNT; ix++ ) {
      Serial.print(time[ix]);
      Serial.print(".0\t");
      Serial.print(left[ix]);
      Serial.print("\t");
      Serial.println(right[ix]);
    }
#endif    
    for ( unsigned int cIx = 0; cIx < rcrossIx && cIx < lcrossIx; cIx++) {
      //sanity check
      if (lcross[cIx] > 0.0f && rcross[cIx] > 0.0f) {
        diff[diffIx++] = lcross[cIx] - rcross[cIx];
        if (diffIx > ACOUNT) {
          diffIx = 0;
        }
      }
#if SHOW_CROSS
      Serial.print(lcross[cIx] - rcross[cIx]);
      Serial.print("\t");
      Serial.print(lcross[cIx]);
      Serial.print("\t");
      Serial.println(rcross[cIx]);
#endif
    }
    float sum = 0.0f;
    for (unsigned int aIx = 0; aIx < ACOUNT; aIx++) {
      sum += diff[aIx];
    }
    Serial.println(sum / ACOUNT);
  }

  delay(250);
}
