#include <Arduino.h>
#include <Wire.h>
#include <arduino-timer.h>
#include <EEPROM.h>
//#include <Adafruit_I2CDevice.h>

// #define I2C_ADDRESS 0x60
// Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice(I2C_ADDRESS);

#define	LED_CLOCK(...) digitalWrite(5, __VA_ARGS__)
#define	LED_DATA(...)	digitalWrite(6, __VA_ARGS__)
#define	LED_LOAD(...)	digitalWrite(7, __VA_ARGS__)

#define	ENCA	digitalRead(2)
#define	ENCB	digitalRead(3)
#define	ENCSWITCH	digitalRead(4)

#define	RLY_ATTENUATOR 0x22
#define	RLY_INPUTS 0x25
#define	RLY_OUTPUTS 0x24
#define	RLY_MISC 0x23

#define	DEBOUNCE 3 // 5-10 is abs max .. 3 should be fine
#define	LONG_PRESS 50 //

/////////////////////////////Global Vars////////////////////////////////////// 

signed char enc_states[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0}; 
uint8_t BUTTON_LAT[] = {0, 4, 8, 13, 1, 5, 9, 15, 2, 6, 10, 14, 3, 7, 11, 12};
uint8_t debounce_flag = 0;
auto timer = timer_create_default(); // create a timer with default settings
uint16_t pressedButtonLED = 0;
signed char POT_LEVEL = 1; 

// uint8_t holdButton = 0; // byte for hold button more than 3 sec
// uint8_t relayPending = 0; // bit field for relay to be updated
// uint8_t buttonPressFlag = 0; // flag containing button id
uint8_t blink = 0;
uint8_t hold_state = 0;
uint16_t LED_snapshot = 0;

union MONIMOUR_STATES_UNION{
  struct {
    uint32_t led_word;
    uint32_t input_trim_word;
    uint32_t output_trim_word;
    uint8_t dim_trim_byte;
  };
  struct {
    uint8_t input; // 8
    uint8_t output; // 8
    char volume; // 8
    unsigned mono:1;  // 1
    unsigned mute:1; // 1
    unsigned dim:1; // 1
    unsigned talkback:1;  // 1
    unsigned mutes:2; // 2
    unsigned solos:2; // 2
    uint8_t input_trim[4]; // 32 (-8 to 8)
    uint8_t output_trim[4]; // 32 (-8 to 8)
    uint8_t dim_trim = 16;  // 8 (percentage)
  };
};

union {
	uint32_t word;
	struct {
    uint16_t ring;
    union {
      uint16_t buttons_word;
      struct {
        unsigned right_buttons:4;
        unsigned outputs:4;
        unsigned inputs:4;
        unsigned mutes:2;
        unsigned solos:2;
      };
      struct {
        unsigned int talk_back:1;
        unsigned int dim:1;
        unsigned int mute:1;
        unsigned int mono:1;
        unsigned int output_4:1;
        unsigned int output_3:1;
        unsigned int output_2:1;
        unsigned int output_1:1;
        unsigned int input_4:1;
        unsigned int input_3:1;
        unsigned int input_2:1;
        unsigned int input_1:1;
        unsigned int mute_right:1;
        unsigned int mute_left:1;
        unsigned int solo_right:1;
        unsigned int solo_left:1;
      };
    };
	};
} LEDs;

#define HOLD_BUTTON_MASK 0b0000111111111100
union BUTTONS_UNION{
	uint16_t word;
  struct {
    unsigned mono:1;
    unsigned mute:1;
    unsigned dim:1;
    unsigned talkback:1;
    unsigned output:4;
    unsigned input:4;
    unsigned mutes:2;
    unsigned solos:2;
  };
} BUTTONs;


BUTTONS_UNION pressedButton;

/*
  config state notes
  - each config must have an id, should be same as button bit position?
  - each config must have a bit mask with relevant buttons and relevant states
  as in outputs have mono button as a relevant button.
  - inputs and outputs should have + / - value in db as well as mono
*/

/*
  2 - volume
  3 - misc
  4 - outputs
  5 - inputs
*/
uint8_t miscRelay = 0;

MONIMOUR_STATES_UNION MONIMOUR_STATES;
MONIMOUR_STATES_UNION lastState;

/////////////////////////////Prototypes////////////////////////////////////// 
void checkEnc(void);
void calcVolume(unsigned char);
void calcLEDring(void);
void setLEDs (void);
void scanButtons(void);
void setRelays(uint8_t, uint8_t);
void timerTick(void);
unsigned char reverse(unsigned char);
int findPosition(unsigned);
void calibrateVolume(void);

void setup() {
  // set pins
  /*
    LEDs
    D2 - 2 - CLK
    D3 - 3 - DATA
    D4 - 4 - LATCH

    Encoder
    D5 - 5 - A
    D6 - 6 - B
    D7 - 7 - Switch
  */
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  pinMode(19, INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);

  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);

  EEPROM.get(0, MONIMOUR_STATES);
  
  MONIMOUR_STATES.mono = 0;
  MONIMOUR_STATES.mute = 0;
  MONIMOUR_STATES.dim = 0;
  MONIMOUR_STATES.talkback = 0;
  MONIMOUR_STATES.input = 1;
  MONIMOUR_STATES.output = 1;
  MONIMOUR_STATES.volume = 1;

  // for now, set this in EEPROM later
  // MONIMOUR_STATES.dim_trim = 32;

  while (!Serial) { delay(10); }

  Serial.begin(9600);
  Serial.println();
  Wire.begin(); // join i2c bus (address optional for master)

  uint8_t i;
  for (i=0;i<6;i++){
    Wire.beginTransmission(0x20+i);   // 0b0100000 + addr
    Wire.write(0x00);                 // GPIO pins 
    Wire.write(0x00);                 // sends one byte  
    Wire.endTransmission();           // stop transmitting    
  }

  attachInterrupt(digitalPinToInterrupt(2), checkEnc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), checkEnc, CHANGE);

  timer.every(500, timerTick);
  
}

void loop() {
  static unsigned char n = 1;
  static uint16_t debounce_cnt = DEBOUNCE;

  calcLEDring();
  calibrateVolume();
  setLEDs();
  
  //  when debounce is done
  if(debounce_flag){
    if(debounce_cnt == DEBOUNCE){
      if(BUTTONs.input){
        MONIMOUR_STATES.input = findPosition(BUTTONs.input);
        pressedButton.word = BUTTONs.word;
      }
      if(BUTTONs.output){
        MONIMOUR_STATES.output = findPosition(BUTTONs.output);
        pressedButton.word = BUTTONs.word;
      }
      if(BUTTONs.dim){
        MONIMOUR_STATES.dim = !MONIMOUR_STATES.dim;
        pressedButton.word = BUTTONs.word;
      }
      if(BUTTONs.mute){
        MONIMOUR_STATES.mute = !MONIMOUR_STATES.mute;
      }
      if(BUTTONs.mono){
        MONIMOUR_STATES.mono = !MONIMOUR_STATES.mono;
      }
      if(BUTTONs.talkback){
        MONIMOUR_STATES.talkback = !MONIMOUR_STATES.talkback;
        pressedButton.word = BUTTONs.word;
      }
  //    hold_state = 0;
    }
    debounce_cnt--;
    if(debounce_cnt == 0) {
      debounce_cnt = DEBOUNCE;
      debounce_flag = 0;
    }
  }
  // continue scan 
  else {
    scanButtons();
  }

  if(hold_state){
    if(pressedButton.dim == 1){
      MONIMOUR_STATES.dim = 1;
      MONIMOUR_STATES.dim_trim = POT_LEVEL;
    }
    if(pressedButton.input) {
      MONIMOUR_STATES.input_trim[MONIMOUR_STATES.input - 1] = POT_LEVEL;
      pressedButtonLED = 0x0800 >> (MONIMOUR_STATES.input - 1);
    }
    if(pressedButton.output) {
      MONIMOUR_STATES.output_trim[MONIMOUR_STATES.output - 1] = POT_LEVEL;
      pressedButtonLED = 0x0080 >> (MONIMOUR_STATES.output - 1);
    }
    
    LEDs.buttons_word = 0;
    LEDs.buttons_word = pressedButtonLED * blink;
  }
  else{

    MONIMOUR_STATES.volume = POT_LEVEL;

    if(MONIMOUR_STATES.led_word != lastState.led_word){
      if(MONIMOUR_STATES.input != lastState.input){
        setRelays(RLY_INPUTS, 1 << (MONIMOUR_STATES.input - 1));
        LEDs.inputs = 16 >> MONIMOUR_STATES.input;
    //    pressedButtonLED = LEDs.buttons_word & 0x0F00;
        Serial.print("INPUT SELECT: ");
        Serial.println(MONIMOUR_STATES.input);
      }
      if(MONIMOUR_STATES.output != lastState.output){
        setRelays(RLY_OUTPUTS, 1 << (MONIMOUR_STATES.output - 1 ));
        LEDs.outputs = 16 >> MONIMOUR_STATES.output;
    //    pressedButtonLED = LEDs.buttons_word & 0x00F0;
        Serial.print("OUTPUT SELECT: ");
        Serial.println(MONIMOUR_STATES.output);
      }
      if(MONIMOUR_STATES.dim != lastState.dim){
        LEDs.dim = MONIMOUR_STATES.dim;
        pressedButtonLED = 0x0002;
        Serial.print("DIM: ");
        Serial.println(MONIMOUR_STATES.dim);
      }
      if(MONIMOUR_STATES.mute != lastState.mute){
        LEDs.mute = MONIMOUR_STATES.mute;
        setRelays(RLY_OUTPUTS, (1 << MONIMOUR_STATES.output -1) * !MONIMOUR_STATES.mute);
        Serial.print("MUTE: ");
        Serial.println(MONIMOUR_STATES.mute);
      }
      if(MONIMOUR_STATES.mono != lastState.mono){
        LEDs.mono = MONIMOUR_STATES.mono;
        miscRelay = (miscRelay & 0b11011111) + (0x20 * MONIMOUR_STATES.mono);
        setRelays(RLY_MISC, miscRelay);
        Serial.print("MONO: ");
        Serial.println(MONIMOUR_STATES.mono);
      }
      if(MONIMOUR_STATES.talkback != lastState.talkback){
        LEDs.talk_back = MONIMOUR_STATES.talkback;
        pressedButtonLED = 0x0001;
        miscRelay = (miscRelay & 0b11110111) + (0x04 * MONIMOUR_STATES.talkback);
        setRelays(RLY_MISC, miscRelay);
        Serial.print("TALK BACK: ");
        Serial.println(MONIMOUR_STATES.talkback);
      }
      lastState.led_word = MONIMOUR_STATES.led_word;
    }
  }
  delay(30);
  timer.tick(); // tick the timer
}

///////////////////////////// Functions ///////////// 
void setLEDs (void)
{
	unsigned char i;
	uint32_t tempLEDs = LEDs.word;

	for(i=0;i<32;i++)
	{
		digitalWrite(6, tempLEDs&0x01);
		tempLEDs>>=1;
		digitalWrite(5, 1);
		digitalWrite(5, 0);
	}
	digitalWrite(7, 1);			// LED Release
	digitalWrite(7, 0);			// LED Load
}

///////////////////////////// Encoder Function - Repeat as required ///////////// 

void checkEnc(void){ 
  cli(); //stop interrupts happening before we read pin values
  unsigned char encState = (ENCB << 1) + ENCA;
  static signed char enc_position = 0;
  enc_position <<= 2;
  enc_position |= (encState);
  POT_LEVEL += enc_states[( enc_position & 0x0f )];
  if( POT_LEVEL < 0 ){ 
      POT_LEVEL = 0; 
  }
  if( POT_LEVEL > 63 ){ 
      POT_LEVEL = 63; 
  }    

//  calcVolume(encState);
/*
  if( MONIMOUR_STATES.volume < 0 ){ 
      MONIMOUR_STATES.volume = 0; 
  }
  if( MONIMOUR_STATES.volume > 63 ){ 
      MONIMOUR_STATES.volume = 63; 
  }    
*/
  sei(); //restart interrupts
}

void calcVolume(unsigned char encState){ 
    static signed char enc_position = 0;
    enc_position <<= 2;
    enc_position |= (encState);
    MONIMOUR_STATES.volume += enc_states[( enc_position & 0x0f )];   
} 

void calibrateVolume(void){
  static unsigned int lastVolume = MONIMOUR_STATES.volume;
  static unsigned int lastCalVolume = 0;
  unsigned int calVol;

  if(MONIMOUR_STATES.dim){
    unsigned temp = (MONIMOUR_STATES.dim_trim * 158) / 100;
    calVol = (MONIMOUR_STATES.volume * temp) / 100;
  }
  else {
    calVol = MONIMOUR_STATES.volume;
  }
  // calibrate with in & out trims here
  signed char deltaVol = MONIMOUR_STATES.input_trim[MONIMOUR_STATES.input-1] - 32; 
  deltaVol += MONIMOUR_STATES.output_trim[MONIMOUR_STATES.output-1] - 32; 
  calVol = calVol + deltaVol;

  
  if( calVol < 0 ){ 
      calVol = 0; 
  }
  if( calVol > 63 ){ 
      calVol = 63; 
  }  
  if(lastVolume != MONIMOUR_STATES.volume){
  //  calcLEDring();
    lastVolume = MONIMOUR_STATES.volume;
  }  
  if(lastCalVolume != calVol){
  //  calcLEDring();
    lastCalVolume = calVol;
    uint8_t vol_shift = reverse(calVol);
    setRelays(RLY_ATTENUATOR, vol_shift);
  }
}

void calcLEDring(void){
    int pot;
	  float rem;
	  unsigned char ledNo;
//  	pot = MONIMOUR_STATES.volume - 2;
  	pot = POT_LEVEL - 2;
    LEDs.ring = 1;
		rem = pot % 4;
		if (rem >1) LEDs.ring += 2;
		ledNo = (pot / 4);
		LEDs.ring = LEDs.ring << ledNo;
}

void scanButtons(void){
  uint8_t col, row;
  uint16_t button_temp = 0;
  static int longPressCnt = LONG_PRESS;

  digitalWrite(10, 1);
  digitalWrite(11, 1);
  digitalWrite(12, 1);
  digitalWrite(13, 1);

  // loop thru columns
  row = 0;
  while(row < 4){ 
    digitalWrite(10 + row, 0);
    col = 0;
    while(col < 4){ 
    //  delay(10);
      if(!digitalRead(14 + col)){
        button_temp = (1 << (BUTTON_LAT[(col + (row * 4))]));
      }
      col++;
    }
    digitalWrite(10 + row, 1);
    row++;
  }

  if(BUTTONs.word != button_temp){
    longPressCnt = LONG_PRESS;
    debounce_flag = 1;
    BUTTONs.word = button_temp;
    if(BUTTONs.word && hold_state) {
      // clear hold state & cancel button press
      BUTTONs.word = 0;
      hold_state = 0;
      POT_LEVEL = MONIMOUR_STATES.volume;
      pressedButton.word = 0;
      LEDs.buttons_word = LED_snapshot;
      EEPROM.put(0, MONIMOUR_STATES);
      Serial.println("STORE CONFIG");
    }
  }
  else if(BUTTONs.word & HOLD_BUTTON_MASK){
    // count long press
    if(longPressCnt){
      longPressCnt--;
    }
    else if((BUTTONs.word & HOLD_BUTTON_MASK) && !hold_state){
      hold_state = 1;
      LED_snapshot = LEDs.buttons_word;
      if(BUTTONs.dim) POT_LEVEL = MONIMOUR_STATES.dim_trim;
      if(BUTTONs.input == 1) POT_LEVEL = MONIMOUR_STATES.input_trim[0];
      if(BUTTONs.input == 2) POT_LEVEL = MONIMOUR_STATES.input_trim[1];
      if(BUTTONs.input == 4) POT_LEVEL = MONIMOUR_STATES.input_trim[2];
      if(BUTTONs.input == 8) POT_LEVEL = MONIMOUR_STATES.input_trim[3];
      if(BUTTONs.output == 1) POT_LEVEL = MONIMOUR_STATES.output_trim[0];
      if(BUTTONs.output == 2) POT_LEVEL = MONIMOUR_STATES.output_trim[1];
      if(BUTTONs.output == 4) POT_LEVEL = MONIMOUR_STATES.output_trim[2];
      if(BUTTONs.output == 8) POT_LEVEL = MONIMOUR_STATES.output_trim[3];
      Serial.print("ENTER CONFIG: ");
      Serial.println(BUTTONs.word);
    }
  }
}

void setRelays(uint8_t addr, uint8_t data){
    Wire.beginTransmission(addr); // 0b0100000 + addr
    Wire.write(0xf09);                // GPIO pins 
    Wire.write(data);    // sends one byte  
    Wire.endTransmission();           // stop transmitting 
}

void timerTick(void){
  if(blink) blink = 0;
  else blink = 1;
//  blink = !blink;
//  Serial.print("Blink: ");
//  Serial.println(blink);
}
unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

int isPowerOfTwo(unsigned n)
{
    return n && (!(n & (n - 1)));
}
 
// Returns position of the only set bit in 'n'
int findPosition(unsigned n)
{
    if (!isPowerOfTwo(n))
        return -1;
 
    unsigned i = 1, pos = 1;
 
    // Iterate through bits of n till we find a set bit
    // i&n will be non-zero only when 'i' and 'n' have a set bit
    // at same position
    while (!(i & n)) {
        // Unset current bit and set the next bit in 'i'
        i = i << 1;
 
        // increment position
        ++pos;
    }
 
    return pos;
}
