/* theory
 * sampling speed is arduino clock speed/prescaler
 * resolution of each bin is sampling speed/# of bins
 * lower sampling speed + more bins = max resolution
 * max number of bins is 256, limiting resolution, so a higher prescaler should be used for better resolution to reduce sampling speed
 * using prescaler of 64 gives a frequency of 19,231 Hz
 * this means all frequences > ~9kHz must be filtered out to avoid aliasing (using LPF)
 * this will still capture the most apparent frequencies in music so all good, resolution of ~75 Hz/bin
 * run code should be kept to a minimum to improve processing times
 * 
 * 
 *  
 *  5k pot w/ 1k inline resistor = 0 to 870
 *  
 */

//FHT definitions
#define LOG_OUT 1     //use the log output function
#define FHT_N 256     //set N point fht
#define BINS_USED 66  //number of bins actually used
#define SCALE_MIN 25  //signal threshold
#define SCALE_MAX 100 //signal max
#define BIN_0_ADJ 10  //for excess noise in bin 0
#define BIN_1_ADJ 5   //for excess noise in bin 1

//LED definitions
#define LED_PIN 7
#define PWR_LED_PIN 8
#define NUM_LEDS 192  //number of LEDs
#define COLUMNS 24    //number of LED strip columns
#define L_PER_C 8     //number of LEDs per column
#define TEST_SW_PIN 22
//hue pot A1
//brightness pot A2
//gain pot A3
//colour mode A4
//mode A5

//include
#include <FHT.h> // include FHT library
#include <FastLED.h> //include LED library

int runState = 0;

//FHT variables
int binAdj[FHT_N/2];
int freqScale[COLUMNS];
byte mode = 0;

//LED variables
CRGB leds[NUM_LEDS];
uint8_t hue_def = 96;
byte brightness = 100;
byte gain = 4;
byte colourStyle = 0;
int h_pot = 0;
int b_pot = 0;
int g_pot = 0;
int c_pot = 0;
int m_pot = 0; 

void setup() {
  
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(brightness);
  pinMode(PWR_LED_PIN, OUTPUT);
  digitalWrite(PWR_LED_PIN, HIGH);    //turn on power button LED

  //TIMSK0 = 0; // turn off timer0 for lower jitter  //BREAKS FASTLED.SHOW
  ADCSRA = 0xe6; // set adc to free running mode, prescaler to 64
  read_settings(); //get initial settings
  ADMUX = 0x00; // use adc0 and aref
  DIDR0 = 0x01; // turn off the digital input for adc0
  spectrum_init();   
  pinMode(TEST_SW_PIN, INPUT);
}

void loop() {
  static uint16_t hue_16 = 0;
  static uint8_t hue = 0;
  
  while(1) { // reduces jitter
    runState = digitalRead(TEST_SW_PIN);

    //RUN MODE
    if(runState == HIGH)
    {
      cli();  // UDRE interrupt slows this way down on arduino1.0
      
      //ADC SECTION ------
      for (int i = 0 ; i < FHT_N ; i++) { // save N samples
        while(!(ADCSRA & 0x10)); // wait for adc to be ready
        ADCSRA = 0xf6; // restart adc
        byte l = ADCL; // fetch adc data
        byte h = ADCH;
        int k = (h << 8) | l; // form into an int
        k -= 0x0200; // form into a signed int  (-512)
        k <<= gain; // form into a 16b signed int  (*2^gain)    //acts as gain - if too high clipping occurs and data gets fucky
        fht_input[i] = k; // put real data into bins
      }
  
      //FHT SECTION ------
      fht_window(); // window the data for better frequency response
      fht_reorder(); // reorder the data before doing the fht
      fht_run(); // process the data in the fht
      fht_mag_log(); // take the output of the fht
      sei();
      for(int i = 0; i < BINS_USED; i++)  //adjust for noise
      {
        if(fht_log_out[i] > binAdj[i])
          fht_log_out[i] -= (byte)binAdj[i];
        else
          fht_log_out[i] = 0;
      }  
      
      
      //LED SECTION ------
      scale_freq();
      scale_mag();
      
      fadeToBlackBy(leds, NUM_LEDS, 50); 
      switch(colourStyle)
      {
        case 1: //wheel
          for (byte i = 1; i <= COLUMNS ; i++) {
            int scaled_out = map(freqScale[i-1], SCALE_MIN, SCALE_MAX, 0, L_PER_C);
            for (byte j = ((i-1) * L_PER_C); j < (scaled_out + ((i-1) * L_PER_C)); j++) //fill colour
            {
              hue_16++; //for colour wheel
              hue = hue_16/128; //for colour wheel                  
              leds[j] = CHSV(hue,255,brightness);                          
            }    
          }
          break;  
        case 2: //per channel
          hue = hue_def;
          for (byte i = 1; i <= COLUMNS ; i++) {
            int scaled_out = map(freqScale[i-1], SCALE_MIN, SCALE_MAX, 0, L_PER_C);
            for (byte j = ((i-1) * L_PER_C); j < (scaled_out + ((i-1) * L_PER_C)); j++) //fill colour
            {
              leds[j] = CHSV(hue,255,brightness);                          
            }
            hue += 10;    
          }
          break;
        case 3: //solid
          for (byte i = 1; i <= COLUMNS ; i++) {
            int scaled_out = map(freqScale[i-1], SCALE_MIN, SCALE_MAX, 0, L_PER_C);
            for (byte j = ((i-1) * L_PER_C); j < (scaled_out + ((i-1) * L_PER_C)); j++) //fill colour
            {                 
              leds[j] = CHSV(hue_def,255,brightness);                          
            }    
          }
          break;
        case 4: //range
          hue = hue_def;
          for (byte i = 1; i <= COLUMNS ; i++) {
            int scaled_out = map(freqScale[i-1], SCALE_MIN, SCALE_MAX, 0, L_PER_C);
            for (byte j = ((i-1) * L_PER_C); j < (scaled_out + ((i-1) * L_PER_C)); j++) //fill colour
            {                 
              leds[j] = CHSV(hue,255,brightness);                          
            }
            if(i == 3)        hue += 51;
            else if(i == 6)   hue += 51;
            else if(i == 13)  hue += 51;
            else if(i == 20)  hue += 51;                  
          }
          break;
        default:  //"classic"
          for (byte i = 1; i <= COLUMNS ; i++) {
            int scaled_out = map(freqScale[i-1], SCALE_MIN, SCALE_MAX, 0, L_PER_C);
            for (byte j = ((i-1) * L_PER_C); j < (scaled_out + ((i-1) * L_PER_C)); j++) //fill colour
            {                
              leds[j] = CHSV(hue,255,brightness);
              hue -= 12;       
            }
            hue = hue_def;
        }
          break;
      }      
      FastLED.show();
    }

    //SETTINGS MODE
    else
    {

      while(runState == LOW)
      {
        read_settings();

        fadeToBlackBy(leds, NUM_LEDS, 255);

        switch(colourStyle)
        {
          case 1: //wheel
            for (byte i = 2; i <= COLUMNS ; i++) {
              for (byte j = ((i-1)* L_PER_C); j < (i * L_PER_C) + (gain - 5); j++) //all LEDs on for demo
              //for (byte j = ((i-1)* L_PER_C); j < (i * L_PER_C); j++) //all LEDs on for demo
              {
                hue_16++; //for colour wheel
                hue = hue_16/128; //for colour wheel                  
                leds[j] = CHSV(hue,255,brightness);                          
              }    
            }
            break;  
          case 2: //per channel
            hue = hue_def;
            for (byte i = 2; i <= COLUMNS ; i++) {
              for (byte j = ((i-1)* L_PER_C); j < (i * L_PER_C) + (gain - 5); j++) //all LEDs on for demo
              //for (byte j = ((i-1)* L_PER_C); j < (i * L_PER_C); j++) //all LEDs on for demo
              {
                leds[j] = CHSV(hue,255,brightness);                          
              }
              hue += 10;    
            }
            break;
          case 3: //solid
            for (byte i = 2; i <= COLUMNS ; i++) {
              for (byte j = ((i-1)* L_PER_C); j < (i * L_PER_C) + (gain - 5); j++) //all LEDs on for demo
              //for (byte j = ((i-1)* L_PER_C); j < (i * L_PER_C); j++) //all LEDs on for demo
              {                 
                leds[j] = CHSV(hue_def,255,brightness);                          
              }    
            }
            break;
          case 4: //range
            hue = hue_def;
            for (byte i = 2; i <= COLUMNS ; i++) {
              for (byte j = ((i-1)* L_PER_C); j < (i * L_PER_C) + (gain - 5); j++) //all LEDs on for demo
              //for (byte j = ((i-1)* L_PER_C); j < (i * L_PER_C); j++) //all LEDs on for demo
              {                 
                leds[j] = CHSV(hue,255,brightness);                          
              }
              if(i == 3)        hue += 51;
              else if(i == 6)   hue += 51;
              else if(i == 13)  hue += 51;
              else if(i == 20)  hue += 51;                  
            }
            break;
          default:  //"classic"
            for (byte i = 2; i <= COLUMNS ; i++) {
              for (byte j = ((i-1)* L_PER_C); j < (i * L_PER_C) + (gain - 5); j++) //all LEDs on for demo
              //for (byte j = ((i-1)* L_PER_C); j < (i * L_PER_C); j++) //all LEDs on for demo
              {                
                leds[j] = CHSV(hue,255,brightness);
                hue -= 12;       
              }
              hue = hue_def;
          }
            break;
        }
        for (byte i = 0; i <= mode; i++) {  //first column shows mode number
          leds[i] = CHSV(hue,255,brightness);
        }
        FastLED.show();
        runState = digitalRead(TEST_SW_PIN);
      }
      ADMUX = 0x00;      
    }
  }
}

void scale_freq()
{
  switch(mode)
  {
    //RANGE BASED
    case 0:
      //bass
      freqScale[0] = fht_log_out[1];
      freqScale[1] = fht_log_out[2];
      freqScale[2] = fht_log_out[3];
      //low midrange
      freqScale[3] = fht_log_out[4];
      freqScale[4] = fht_log_out[5];
      freqScale[5] = fht_log_out[6];
      //midrange
      freqScale[6] = (fht_log_out[7] + fht_log_out[8])/2;
      freqScale[7] = (fht_log_out[9] + fht_log_out[10] + fht_log_out[11])/2;
      freqScale[8] = (fht_log_out[12] + fht_log_out[13] + fht_log_out[14])/2;
      freqScale[9] = (fht_log_out[15] + fht_log_out[16] + fht_log_out[17])/3;
      freqScale[10] = (fht_log_out[18] + fht_log_out[19] + fht_log_out[20])/3;
      freqScale[11] = (fht_log_out[21] + fht_log_out[22] + fht_log_out[23])/3;
      freqScale[12] = (fht_log_out[24] + fht_log_out[25] + fht_log_out[26])/3;
      //upper midrange
      freqScale[13] = (fht_log_out[27] + fht_log_out[28] + fht_log_out[29] + fht_log_out[30])/4;
      freqScale[14] = (fht_log_out[31] + fht_log_out[32] + fht_log_out[33] + fht_log_out[34])/4;
      freqScale[15] = (fht_log_out[35] + fht_log_out[36] + fht_log_out[37] + fht_log_out[38])/4;
      freqScale[16] = (fht_log_out[39] + fht_log_out[40] + fht_log_out[41] + fht_log_out[42])/4;
      freqScale[17] = (fht_log_out[43] + fht_log_out[44] + fht_log_out[45] + fht_log_out[46])/4;
      freqScale[18] = (fht_log_out[47] + fht_log_out[48] + fht_log_out[49] + fht_log_out[50])/4;
      freqScale[19] = (fht_log_out[51] + fht_log_out[52] + fht_log_out[53])/3;
      //presence
      freqScale[20] = (fht_log_out[54] + fht_log_out[55] + fht_log_out[56])/3;
      freqScale[21] = (fht_log_out[57] + fht_log_out[58] + fht_log_out[59])/3;
      freqScale[22] = (fht_log_out[60] + fht_log_out[61] + fht_log_out[62])/2;
      freqScale[23] = (fht_log_out[63] + fht_log_out[64] + fht_log_out[65])/2;
      break;
      
    //PIANO
    case 1:
      freqScale[0] = fht_log_out[1];
      freqScale[1] = fht_log_out[2];
      freqScale[2] = fht_log_out[3];
      freqScale[3] = fht_log_out[4];
      freqScale[4] = fht_log_out[5];
      freqScale[5] = fht_log_out[6];
      freqScale[6] = fht_log_out[7];
      freqScale[7] = fht_log_out[8];
      freqScale[8] = fht_log_out[9];
      freqScale[9] = fht_log_out[10];
      freqScale[10] = fht_log_out[11];
      freqScale[11] = fht_log_out[12];
      freqScale[12] = fht_log_out[13];
      freqScale[13] = fht_log_out[14];
      freqScale[14] = fht_log_out[15];
      freqScale[15] = fht_log_out[16];
      freqScale[16] = fht_log_out[17];
      freqScale[17] = fht_log_out[18];
      freqScale[18] = (fht_log_out[19] + fht_log_out[20])/2;
      freqScale[19] = (fht_log_out[21] + fht_log_out[22])/2;
      freqScale[20] = (fht_log_out[23] + fht_log_out[24])/2;
      freqScale[21] = (fht_log_out[25] + fht_log_out[26])/2;
      freqScale[22] = (fht_log_out[27] + fht_log_out[28])/2;
      freqScale[23] = (fht_log_out[29] + fht_log_out[30])/2;
      break;

    //BASS FOCUS
    case 2:
      freqScale[0] = fht_log_out[1];
      freqScale[1] = fht_log_out[2];
      freqScale[2] = fht_log_out[3];
      freqScale[3] = fht_log_out[4];
      freqScale[4] = fht_log_out[5];
      freqScale[5] = fht_log_out[6];
      freqScale[6] = fht_log_out[7];
      freqScale[7] = fht_log_out[8];
      freqScale[8] = fht_log_out[9];
      freqScale[9] = fht_log_out[10];
      freqScale[10] = fht_log_out[11];
      freqScale[11] = fht_log_out[12];
      freqScale[12] = fht_log_out[13];
      freqScale[13] = fht_log_out[14];
      freqScale[14] = fht_log_out[15];
      freqScale[15] = fht_log_out[16];
      freqScale[16] = fht_log_out[17];
      freqScale[17] = fht_log_out[18];
      freqScale[18] = fht_log_out[19];
      freqScale[19] = fht_log_out[20];
      freqScale[20] = fht_log_out[21];
      freqScale[21] = fht_log_out[22];
      freqScale[22] = fht_log_out[23];
      freqScale[23] = fht_log_out[24];
      break;

    //MID FOCUS
    case 3:
      freqScale[0] = fht_log_out[7];
      freqScale[1] = fht_log_out[8];
      freqScale[2] = fht_log_out[9];
      freqScale[3] = fht_log_out[10];
      freqScale[4] = fht_log_out[11];
      freqScale[5] = fht_log_out[12];
      freqScale[6] = fht_log_out[13];
      freqScale[7] = fht_log_out[14];
      freqScale[8] = fht_log_out[15];
      freqScale[9] = fht_log_out[16];
      freqScale[10] = fht_log_out[17];
      freqScale[11] = fht_log_out[18];
      freqScale[12] = fht_log_out[19];
      freqScale[13] = fht_log_out[20];
      freqScale[14] = fht_log_out[21];
      freqScale[15] = fht_log_out[22];
      freqScale[16] = fht_log_out[23];
      freqScale[17] = fht_log_out[24];
      freqScale[18] = fht_log_out[25];
      freqScale[19] = fht_log_out[26];
      freqScale[20] = fht_log_out[27];
      freqScale[21] = fht_log_out[28];
      freqScale[22] = fht_log_out[29];
      freqScale[23] = fht_log_out[30];
      break;

    //HIGH FOCUS
    case 4:
      freqScale[0] = fht_log_out[27];
      freqScale[1] = fht_log_out[28];
      freqScale[2] = fht_log_out[29];
      freqScale[3] = fht_log_out[30];
      freqScale[4] = fht_log_out[31];
      freqScale[5] = fht_log_out[32];
      freqScale[6] = fht_log_out[33];
      freqScale[7] = fht_log_out[34];
      freqScale[8] = fht_log_out[35];
      freqScale[9] = fht_log_out[36];
      freqScale[10] = fht_log_out[37];
      freqScale[11] = fht_log_out[38];
      freqScale[12] = fht_log_out[39];
      freqScale[13] = fht_log_out[40];
      freqScale[14] = fht_log_out[41];
      freqScale[15] = fht_log_out[42];
      freqScale[16] = fht_log_out[43];
      freqScale[17] = fht_log_out[44];
      freqScale[18] = fht_log_out[45];
      freqScale[19] = fht_log_out[46];
      freqScale[20] = fht_log_out[47];
      freqScale[21] = fht_log_out[48];
      freqScale[22] = fht_log_out[49];
      freqScale[23] = fht_log_out[50];
      break;

    
    //case 5:
      //break;

    
    default:
      //bass
      freqScale[0] = fht_log_out[1];
      freqScale[1] = fht_log_out[2];
      freqScale[2] = fht_log_out[3];
      //low midrange
      freqScale[3] = fht_log_out[4];
      freqScale[4] = fht_log_out[5];
      freqScale[5] = fht_log_out[6];
      //midrange
      freqScale[6] = (fht_log_out[7] + fht_log_out[8])/2;
      freqScale[7] = (fht_log_out[9] + fht_log_out[10] + fht_log_out[11])/2;
      freqScale[8] = (fht_log_out[12] + fht_log_out[13] + fht_log_out[14])/2;
      freqScale[9] = (fht_log_out[15] + fht_log_out[16] + fht_log_out[17])/3;
      freqScale[10] = (fht_log_out[18] + fht_log_out[19] + fht_log_out[20])/3;
      freqScale[11] = (fht_log_out[21] + fht_log_out[22] + fht_log_out[23])/3;
      freqScale[12] = (fht_log_out[24] + fht_log_out[25] + fht_log_out[26])/3;
      //upper midrange
      freqScale[13] = (fht_log_out[27] + fht_log_out[28] + fht_log_out[29] + fht_log_out[30])/4;
      freqScale[14] = (fht_log_out[31] + fht_log_out[32] + fht_log_out[33] + fht_log_out[34])/4;
      freqScale[15] = (fht_log_out[35] + fht_log_out[36] + fht_log_out[37] + fht_log_out[38])/4;
      freqScale[16] = (fht_log_out[39] + fht_log_out[40] + fht_log_out[41] + fht_log_out[42])/4;
      freqScale[17] = (fht_log_out[43] + fht_log_out[44] + fht_log_out[45] + fht_log_out[46])/4;
      freqScale[18] = (fht_log_out[47] + fht_log_out[48] + fht_log_out[49] + fht_log_out[50])/4;
      freqScale[19] = (fht_log_out[51] + fht_log_out[52] + fht_log_out[53])/3;
      //presence
      freqScale[20] = (fht_log_out[54] + fht_log_out[55] + fht_log_out[56])/3;
      freqScale[21] = (fht_log_out[57] + fht_log_out[58] + fht_log_out[59])/3;
      freqScale[22] = (fht_log_out[60] + fht_log_out[61] + fht_log_out[62])/2;
      freqScale[23] = (fht_log_out[63] + fht_log_out[64] + fht_log_out[65])/2;
      break;
  }
}

void scale_mag()
{
  //fht_log_out typically returns values from 0 to ~150, with a normal of ~64-128
  //~0 to 20 is just noise
    
  for(int i = 0; i < COLUMNS; i++)
  {
    if(freqScale[i] < SCALE_MIN) //set min
      freqScale[i] = SCALE_MIN;
    else if(freqScale[i] > SCALE_MAX)  //set max
      freqScale[i] = SCALE_MAX;
  }
}

void read_settings()
{
  //HUE
  ADMUX = 0x01;
  delay(10);
  while(!(ADCSRA & 0x10)); // wait for adc to be ready
  ADCSRA = 0xf6; // restart adc
  byte l = ADCL; // fetch adc data
  byte h = ADCH;
  h_pot = (h << 8) | l; // form into an int
  hue_def = map(h_pot, 0, 750, 0, 255);

  //BRIGHTNESS
  ADMUX = 0x02;
  delay(10);
  while(!(ADCSRA & 0x10)); // wait for adc to be ready
  ADCSRA = 0xf6; // restart adc
  l = ADCL; // fetch adc data
  h = ADCH;
  b_pot = (h << 8) | l; // form into an int
  brightness = map(b_pot, 0, 870, -160, -10);

  //COLOUR STYLE
  ADMUX = 0x03;
  delay(10);
  while(!(ADCSRA & 0x10)); // wait for adc to be ready
  ADCSRA = 0xf6; // restart adc
  l = ADCL; // fetch adc data
  h = ADCH;
  c_pot = (h << 8) | l; // form into an int
  colourStyle = map(c_pot, 0, 500, 0, 4);
  
  //GAIN
  ADMUX = 0x04;
  delay(10);
  while(!(ADCSRA & 0x10)); // wait for adc to be ready
  ADCSRA = 0xf6; // restart adc
  l = ADCL; // fetch adc data
  h = ADCH;
  g_pot = (h << 8) | l; // form into an int
  gain = map(g_pot, 850, 0, 5, 0);

  //MODE
  ADMUX = 0x05;
  delay(10);
  while(!(ADCSRA & 0x10)); // wait for adc to be ready
  ADCSRA = 0xf6; // restart adc
  l = ADCL; // fetch adc data
  h = ADCH;
  m_pot = (h << 8) | l; // form into an int
  mode = map(m_pot, 0, 500, 0, 5);
}

void spectrum_init()
{
  //clean out ADC after setting reference to external
  for (int i = 0 ; i < 10 ; i++) { // save 256 samples
      while(!(ADCSRA & 0x10)); // wait for adc to be ready
      ADCSRA = 0xf5; // restart adc
      byte l = ADCL; // fetch adc data
      byte h = ADCH;
  }

  //adjust for average noise
  for(int i = 0; i < 10; i++)
  {
    cli();  // UDRE interrupt slows this way down on arduino1.0
    for (int i = 0 ; i < FHT_N ; i++) { // save 256 samples
      while(!(ADCSRA & 0x10)); // wait for adc to be ready
      ADCSRA = 0xf6; // restart adc
      byte l = ADCL; // fetch adc data
      byte h = ADCH;
      int k = (h << 8) | l; // form into an int
      k -= 0x0200; // form into a signed int
      k <<= 4; // form into a 16b signed int      //acts as gain - if too high clipping occurs and data gets fucky
      fht_input[i] = k; // put real data into bins
    }
    fht_window(); // window the data for better frequency response
    fht_reorder(); // reorder the data before doing the fht
    fht_run(); // process the data in the fht
    fht_mag_log(); // take the output of the fht
    sei();
    for(int i = 0; i < FHT_N/2; i++)
    {
      binAdj[i] = binAdj[i] + (int)fht_log_out[i];
    }
  }
  for(int i = 0; i < FHT_N/2; i++)
  {
    binAdj[i] = binAdj[i]/10;
  }
  //binAdj[0] += BIN_0_ADJ;
  binAdj[1] += BIN_1_ADJ;

  //test LEDS
  static uint8_t hue = 0;
  for(int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CHSV(hue++, 255, brightness);
    FastLED.show();
    delay(10);
  }
  delay(300);
  //set LEDS to off
  FastLED.clear();
  FastLED.show();
}

