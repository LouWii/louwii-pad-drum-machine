#include <avr/pgmspace.h>

// Workaround for wrong definitions in "iom32u4.h".
// This should be fixed in the AVR toolchain.
#undef UHCON
#undef UHINT
#undef UHIEN
#undef UHADDR
#undef UHFNUM
#undef UHFNUML
#undef UHFNUMH
#undef UHFLEN
#undef UPINRQX
#undef UPINTX
#undef UPNUM
#undef UPRST
#undef UPCONX
#undef UPCFG0X
#undef UPCFG1X
#undef UPSTAX
#undef UPCFG2X
#undef UPIENX
#undef UPDATX
#undef TCCR2A
#undef WGM20
#undef WGM21
#undef COM2B0
#undef COM2B1
#undef COM2A0
#undef COM2A1
#undef TCCR2B
#undef CS20
#undef CS21
#undef CS22
#undef WGM22
#undef FOC2B
#undef FOC2A
#undef TCNT2
#undef TCNT2_0
#undef TCNT2_1
#undef TCNT2_2
#undef TCNT2_3
#undef TCNT2_4
#undef TCNT2_5
#undef TCNT2_6
#undef TCNT2_7
#undef OCR2A
#undef OCR2_0
#undef OCR2_1
#undef OCR2_2
#undef OCR2_3
#undef OCR2_4
#undef OCR2_5
#undef OCR2_6
#undef OCR2_7
#undef OCR2B
#undef OCR2_0
#undef OCR2_1
#undef OCR2_2
#undef OCR2_3
#undef OCR2_4
#undef OCR2_5
#undef OCR2_6
#undef OCR2_7

#define NUM_DIGITAL_PINS  30
#define NUM_ANALOG_INPUTS 12

#define TX_RX_LED_INIT	DDRD |= (1<<5), DDRB |= (1<<0)
#define TXLED0			PORTD |= (1<<5)
#define TXLED1			PORTD &= ~(1<<5)
#define RXLED0			PORTB |= (1<<0)
#define RXLED1			PORTB &= ~(1<<0)

const uint8_t ASDA = 2;
const uint8_t ASCL = 3;

// Map SPI port to 'new' pins D14..D17
static const uint8_t ASS   = 17;
static const uint8_t AMOSI = 16;
static const uint8_t AMISO = 14;
static const uint8_t ASCK  = 15;

// Mapping of analog pins as digital I/O
// A6-A11 share with digital pins
static const uint8_t AA0 = 18;
static const uint8_t AA1 = 19;
static const uint8_t AA2 = 20;
static const uint8_t AA3 = 21;
static const uint8_t AA4 = 22;
static const uint8_t AA5 = 23;
static const uint8_t AA6 = 24;	// D4
static const uint8_t AA7 = 25;	// D6
static const uint8_t AA8 = 26;	// D8
static const uint8_t AA9 = 27;	// D9
static const uint8_t AA10 = 28;	// D10
static const uint8_t AA11 = 29;	// D12

#define digitalPinToPCICR(p)    ((((p) >= 8 && (p) <= 11) || ((p) >= 14 && (p) <= 17) || ((p) >= AA8 && (p) <= AA10)) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) 0
#define digitalPinToPCMSK(p)    ((((p) >= 8 && (p) <= 11) || ((p) >= 14 && (p) <= 17) || ((p) >= AA8 && (p) <= AA10)) ? (&PCMSK0) : ((uint8_t *)0))
#define digitalPinToPCMSKbit(p) ( ((p) >= 8 && (p) <= 11) ? (p) - 4 : ((p) == 14 ? 3 : ((p) == 15 ? 1 : ((p) == 16 ? 2 : ((p) == 17 ? 0 : (p - AA8 + 4))))))

//	__AVR_ATmega32U4__ has an unusual mapping of pins to channels
extern const uint8_t PROGMEM analog_pin_to_channel_PGM[];
#define analogPinToChannel(P)  ( pgm_read_byte( analog_pin_to_channel_PGM + (P) ) )

#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEL ATMEGA32U4 / ARDUINO LEONARDO
//
// D0				PD2					RXD1/INT2
// D1				PD3					TXD1/INT3
// D2				PD1		ASDA			ASDA/INT1
// D3#				PD0		PWM8/ASCL	OC0B/ASCL/INT0
// D4		A6		PD4					ADC8
// D5#				PC6		???			OC3A/#OC4A
// D6#		A7		PD7		FastPWM		#OC4D/ADC10
// D7				PE6					INT6/AIN0
//
// D8		A8		PB4					ADC11/PCINT4
// D9#		A9		PB5		PWM16		OC1A/#OC4B/ADC12/PCINT5
// D10#		A10		PB6		PWM16		OC1B/0c4B/ADC13/PCINT6
// D11#				PB7		PWM8/16		0C0A/OC1C/#RTS/PCINT7
// D12		A11		PD6					T1/#OC4D/ADC9
// D13#				PC7		PWM10		CLK0/OC4A
//
// A0		D18		PF7					ADC7
// A1		D19		PF6					ADC6
// A2		D20 	PF5					ADC5
// A3		D21 	PF4					ADC4
// A4		D22		PF1					ADC1
// A5		D23 	PF0					ADC0
//
// New pins D14..D17 to map SPI port to digital pins
//
// AMISO		D14		PB3					AMISO,PCINT3
// ASCK		D15		PB1					ASCK,PCINT1
// AMOSI		D16		PB2					AMOSI,PCINT2
// SS		D17		PB0					RXLED,SS/PCINT0
//
// TXLED			PD5
// RXLED		    PB0
// HWB				PE2					HWB

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
	(uint16_t) &PINF,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	PD, // D0 - PD2
	PD,	// D1 - PD3
	PD, // D2 - PD1
	PD,	// D3 - PD0
	PD,	// D4 - PD4
	PC, // D5 - PC6
	PD, // D6 - PD7
	PE, // D7 - PE6
	
	PB, // D8 - PB4
	PB,	// D9 - PB5
	PB, // D10 - PB6
	PB,	// D11 - PB7
	PD, // D12 - PD6
	PC, // D13 - PC7
	
	PB,	// D14 - AMISO - PB3
	PB,	// D15 - ASCK - PB1
	PB,	// D16 - AMOSI - PB2
	PB,	// D17 - SS - PB0
	
	PF,	// D18 - A0 - PF7
	PF, // D19 - A1 - PF6
	PF, // D20 - A2 - PF5
	PF, // D21 - A3 - PF4
	PF, // D22 - A4 - PF1
	PF, // D23 - A5 - PF0
	
	PD, // D24 / D4 - A6 - PD4
	PD, // D25 / D6 - A7 - PD7
	PB, // D26 / D8 - A8 - PB4
	PB, // D27 / D9 - A9 - PB5
	PB, // D28 / D10 - A10 - PB6
	PD, // D29 / D12 - A11 - PD6
	PD, // D30 - PD5
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	_BV(2), // D0 - PD2
	_BV(3),	// D1 - PD3
	_BV(1), // D2 - PD1
	_BV(0),	// D3 - PD0
	_BV(4),	// D4 - PD4
	_BV(6), // D5 - PC6
	_BV(7), // D6 - PD7
	_BV(6), // D7 - PE6
	
	_BV(4), // D8 - PB4
	_BV(5),	// D9 - PB5
	_BV(6), // D10 - PB6
	_BV(7),	// D11 - PB7
	_BV(6), // D12 - PD6
	_BV(7), // D13 - PC7
	
	_BV(3),	// D14 - AMISO - PB3
	_BV(1),	// D15 - ASCK - PB1
	_BV(2),	// D16 - AMOSI - PB2
	_BV(0),	// D17 - SS - PB0
	
	_BV(7),	// D18 - A0 - PF7
	_BV(6), // D19 - A1 - PF6
	_BV(5), // D20 - A2 - PF5
	_BV(4), // D21 - A3 - PF4
	_BV(1), // D22 - A4 - PF1
	_BV(0), // D23 - A5 - PF0
	
	_BV(4), // D24 / D4 - A6 - PD4
	_BV(7), // D25 / D6 - A7 - PD7
	_BV(4), // D26 / D8 - A8 - PB4
	_BV(5), // D27 / D9 - A9 - PB5
	_BV(6), // D28 / D10 - A10 - PB6
	_BV(6), // D29 / D12 - A11 - PD6
	_BV(5), // D30 PD5
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER,	
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	TIMER0B,		/* 3 */
	NOT_ON_TIMER,
	TIMER3A,		/* 5 */
	TIMER4D,		/* 6 */
	NOT_ON_TIMER,	
	
	NOT_ON_TIMER,	
	TIMER1A,		/* 9 */
	TIMER1B,		/* 10 */
	TIMER0A,		/* 11 */
	
	NOT_ON_TIMER,	
	TIMER4A,		/* 13 */
	
	NOT_ON_TIMER,	
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,

	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
};

const uint8_t PROGMEM analog_pin_to_channel_PGM[] = {
	7,	// A0				PF7					ADC7
	6,	// A1				PF6					ADC6	
	5,	// A2				PF5					ADC5	
	4,	// A3				PF4					ADC4
	1,	// A4				PF1					ADC1	
	0,	// A5				PF0					ADC0	
	8,	// A6		D4		PD4					ADC8
	10,	// A7		D6		PD7					ADC10
	11,	// A8		D8		PB4					ADC11
	12,	// A9		D9		PB5					ADC12
	13,	// A10		D10		PB6					ADC13
	9	// A11		D12		PD6					ADC9
};

#endif /* ARDUINO_MAIN */

// Louwii-pad setup
// -------------------------------------
// Buttons: 2 lines * 4 columns
// MATRIX_ROW_PINS { D7, B4 }
// MATRIX_COL_PINS { B6, C6, C7, F5 }
#define BTN_ROW_NUM 2
#define BTN_COL_NUM 4

#define BTN_ROW0_PIN AA7 // PD7
#define BTN_ROW1_PIN AA8 // PB4
const uint8_t buttonRowsPins[] = {BTN_ROW0_PIN, BTN_ROW1_PIN};

#define BTN_COL0_PIN AA10 // PB6 D10
#define BTN_COL1_PIN 5 // PC6 D5
#define BTN_COL2_PIN 13 // PC7 D13
#define BTN_COL3_PIN AA2 // PF5 D20
const uint8_t buttonColsPins[] = {BTN_COL0_PIN, BTN_COL1_PIN, BTN_COL2_PIN, BTN_COL3_PIN};

uint8_t rowsState[BTN_ROW_NUM];
uint8_t colsState[BTN_COL_NUM];
uint8_t buttonsState[BTN_ROW_NUM][BTN_COL_NUM];
uint8_t col0State = -1;
uint8_t row0State = -1;

uint8_t matrixState[BTN_ROW_NUM][BTN_COL_NUM];

// -------------------------------------

// Rotary encoders: 2 lines * 3 columns
// ENCODERS_PAD_B { D4, F7, D3, F4, B0, F0 }
// ENCODERS_PAD_A { D6, F6, D2, F1, B7, E6 }
#include <Encoder.h>

// Might cause problems if other libs or code uses attachInterrupt()
//#define ENCODER_OPTIMIZE_INTERRUPTS

// (4 is col0/col2 or 12 is col0/2)
// 12 is row 0 - PD7 - 27

#define ENCODER0_PIN0 AA11 // ATmega pin 25
#define ENCODER0_PIN1 AA6 // ATmega pin 26

#define ENCODER1_PIN0 AA1
#define ENCODER1_PIN1 AA0

// If the control is reversed, inverse the 2 pins here or directly on your board
Encoder encoder0(ENCODER0_PIN0, ENCODER0_PIN1);
Encoder encoder1(ENCODER1_PIN0, ENCODER1_PIN1);

const int debounceDelay = 40;

long encoderCurrentValue;
long encoderLastChange = 0;
// The step allows us to change the "sensibility" of the encoder, we "detect" a change only if the encoder has changed n steps
//  The more steps, the least sensible
const int step = 1;

long currentTime;
long encoderNewValue;

// -------------------------------------

// Rotary encoder buttons
// Encoder 0 (top left)
// Pin D5

// -------------------------------------

// RGB LEDs
// 4 RGB LEDs
// Pin B5

// -------------------------------------

// OLED Screen
// ASDA pin D1
// ASCL pin D0

#define OLED_SDA 2
#define OLED_SCL 3
#define DISPLAY_MAX_TIME_MS 25000

byte displayOn = 0;
long displayStateChangeTime = 0;
uint8_t beatMapDisplayed = 0;

//#include <SSD1306Ascii.h>
//#include <SSD1306AsciiAvrI2c.h>

//#define I2C_ADDRESS 0x3C

//SSD1306AsciiAvrI2c oled;

#include <oled.h>

OLED display(OLED_SDA, OLED_SCL, NO_RESET_PIN, OLED::W_128, OLED::H_32, OLED::CTRL_SSD1306, 0x3C);

// -------------------------------------

#define BPM_DEFAULT 60

int currentBpm = BPM_DEFAULT;

// Number of sections in a measure
#define BEAT_SECTIONS 32
#define INSTRUMENTS_NUM 4

#define INST_IDX_KICK 0
#define INST_IDX_SNARE 1
#define INST_IDX_HIHAT_CL 2
#define INST_IDX_HIHAT_OP 3
char instrumentNames[INSTRUMENTS_NUM][13] = {"Kick", "Snare", "Hi-hat closed", "Hi-hat open"};
// uint8_t instKick[BEAT_SECTIONS];        // C1
// uint8_t instSnare[BEAT_SECTIONS];       // C#1
// uint8_t instHiHatClosed[BEAT_SECTIONS]; // D1
// uint8_t instHiHatOpened[BEAT_SECTIONS]; // D#1
uint8_t instrumentTracks[INSTRUMENTS_NUM][BEAT_SECTIONS];
uint8_t instrumentChannels[] = {10, 10, 10, 10};
uint8_t instrumentNotes[] = {
  24, // C1
  25, // C#1
  26, // D1
  27  // D#1
};
uint8_t instrumentVelocity[] = {127, 127, 127, 127};

uint8_t previousBeatSection = 0;
uint8_t currentBeatSection = 0;
long lastSectionPlayedMicros = 0;
long beatSectionTimingMicros = 0;

// -------------------------------------

#include <USB-MIDI.h>
// Create and bind the MIDI interface to the default hardware Serial port
USBMIDI_CREATE_DEFAULT_INSTANCE();



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);

  MIDI.begin();

  pinMode(ENCODER0_PIN0, INPUT_PULLUP);
  pinMode(ENCODER0_PIN1, INPUT_PULLUP);

  pinMode(ENCODER1_PIN0, INPUT_PULLUP);
  pinMode(ENCODER1_PIN1, INPUT_PULLUP);

  // Setting up the button matrix pins
  for (int i = 0; i < BTN_ROW_NUM; i ++) {
    pinMode(buttonRowsPins[i], INPUT_PULLUP);
    Serial.print("Setting up INPUT_PULLUP on pin ");
    Serial.println(buttonRowsPins[i]);
  }
  for (int i = 0; i < BTN_COL_NUM; i ++) {
    pinMode(buttonColsPins[i], INPUT);
    Serial.print("Setting up INPUT on pin ");
    Serial.println(buttonColsPins[i]);
  }

  display.begin();
  //oled.begin(&Adafruit128x32, I2C_ADDRESS);
  //oled.setFont(System5x7);

  beatSectionTimingMicros = calculateBeatSectionTimingMicros();
  instrumentTracks[0][0] = 1;
  instrumentTracks[1][10] = 1;
  instrumentTracks[2][20] = 1;

  Serial.println("Arduino ready.");
}

void loop() {
  // handleBeat();
  readMatrix();
  // handleDisplay();
  // delay(5000);
  currentTime = millis();

  encoderNewValue = encoder0.read();
  if ( abs((encoderCurrentValue - encoderNewValue)) >= step ) {
    // Debouncing: consider a change of state is valid if it happens after x milliseconds
    //  If not, we consider it as noise and do nothing
    if (currentTime - encoderLastChange > debounceDelay) {
      //oled.print("Time: ");
      //oled.println(currentTime);
      
      if (encoderCurrentValue < encoderNewValue) {
        // Going up
        Serial.println("Up!");
      } else {
        // Going down
        Serial.println("Down!");
      }
      Serial.print("Time: ");
      Serial.print(currentTime);
      Serial.print("; Last time: ");
      Serial.print(encoderLastChange);
      Serial.print("; Encoder value: ");
      Serial.print(encoderNewValue);
      Serial.print("; Encoder last value: ");
      Serial.println(encoderCurrentValue);
      encoderCurrentValue = encoderNewValue;
      encoderLastChange = currentTime;
    }
  }
}

void handleBeat() {
  currentTime = micros();

  // Check if it's time to run a section
  if (currentTime - lastSectionPlayedMicros >= beatSectionTimingMicros) {

    // Check each instrument

    for (int i = 0; i < INSTRUMENTS_NUM; ++i) {
      // If note was already on from previous section, send note off first
      if (1 == instrumentTracks[i][previousBeatSection]) {
        MIDI.sendNoteOff(instrumentNotes[i], instrumentVelocity[i], instrumentChannels[i]);
      }
    
      if (1 == instrumentTracks[i][currentBeatSection]) {
        MIDI.sendNoteOn(instrumentNotes[i], instrumentVelocity[i], instrumentChannels[i]);
        Serial.print("-- ");
        Serial.print(instrumentNames[i]);
        Serial.print("! --    (theoretical timing ");
        Serial.print(lastSectionPlayedMicros + beatSectionTimingMicros);
        Serial.print("; actual timing ");
        Serial.print(currentTime);
        Serial.print("; Timing gap: ");
        Serial.print(beatSectionTimingMicros);
        Serial.println(")");
      }
    }

    // If note was already on from previous section, send note off first
    // if (1 == instKick[previousBeatSection]) {
    //   MIDI.sendNoteOff(instrumentNotes[0], instrumentVelocity[0], instrumentChannels[0]);
    // }

    // if (1 == instKick[currentBeatSection]) {
    //   MIDI.sendNoteOn(instrumentNotes[0], instrumentVelocity[0], instrumentChannels[0]);
    //   Serial.print("-- Kick! --    (theoretical timing ");
    //   Serial.print(lastSectionPlayedMicros + beatSectionTimingMicros);
    //   Serial.print("; actual timing ");
    //   Serial.print(currentTime);
    //   Serial.print("; Timing gap: ");
    //   Serial.print(beatSectionTimingMicros);
    //   Serial.println(")");
    // } else {
      
    // }

    // drawBeatMap();

    // Instead of using the actual current time to know when we last played the note, we use the theoretical timing it was supposed to play on
    //lastSectionPlayedMicros = currentTime;
    lastSectionPlayedMicros += beatSectionTimingMicros;
    previousBeatSection = currentBeatSection;
    if ((BEAT_SECTIONS - 1) == currentBeatSection) {
      // Return to the begining of beat
      currentBeatSection = 0;
    } else {
      currentBeatSection++;
    }

    
  }
}

void readMatrix() {
  for (int i = 0; i < BTN_ROW_NUM; i ++) {
    uint8_t curRow = buttonRowsPins[i];
    pinMode(curRow, OUTPUT);
		digitalWrite(curRow, LOW);

    for (int j = 0; j < BTN_COL_NUM; j ++) {
      uint8_t colRow = buttonColsPins[j];
      pinMode(colRow, INPUT_PULLUP);
      int btnValue = digitalRead(colRow);
      // Serial.print("Reading row ");
      // Serial.print(i);
      // Serial.print(" col ");
      // Serial.print(j);
      // Serial.print(": value ");
      // Serial.println(btnValue);
      pinMode(colRow, INPUT);
      if (btnValue != matrixState[i][j]) {
        if (LOW == btnValue) {
          Serial.print("Btn press - row ");
          Serial.print(i);
          Serial.print(" col ");
          Serial.println(j);
          handleButtonPress(i, j);
        }
        matrixState[i][j] = btnValue;
      }
    }

    // disable the column
		pinMode(curRow, INPUT);
  }
}

void handleDisplay() {
  currentTime = millis();
  
  if (0 == displayStateChangeTime) {
    // Set something on the screen
    // display.draw_string(6,8,"Hello World");
    // display.display();
    //drawBeatMap();
    displayOn = 1;
  }

  if (1 == displayOn && (currentTime - displayStateChangeTime) >= DISPLAY_MAX_TIME_MS) {
    // Display has been on for too long, time to clear it to avoid burn-in
    displayOn = 0;
    displayStateChangeTime = currentTime;
    display.set_power(0);
    Serial.println("Turn display off");
  } else if (0 == displayOn && (currentTime - displayStateChangeTime) >= DISPLAY_MAX_TIME_MS) {
    // Has been off for n seconds, turn it on again
    displayOn = 1;
    displayStateChangeTime = currentTime;
    display.set_power(1);
    Serial.println("Turn display on");
  }
}

void drawBeatMap() {
  // Screen is 1 measure, containing 4 beats
  // Display 4 beats on screen
  if (0 == beatMapDisplayed) {
    display.clear();
    display.draw_rectangle(0, 0, 1, 3, OLED::SOLID);
    display.draw_rectangle(32, 0, 33, 3, OLED::SOLID);
    display.draw_rectangle(64, 0, 65, 3, OLED::SOLID);
    display.draw_rectangle(96, 0, 97, 3, OLED::SOLID);
    beatMapDisplayed = 1;
  }

  // Draw current beat position
  display.draw_rectangle(
    (currentBeatSection*4),
    5,
    (currentBeatSection*4)+3,
    8,
    OLED::SOLID
  );

  display.display();
}

void handleButtonPress(int row, int col) {
  switch (row) {
    case 0:
      switch (col) {
        case 0:
          // E1
          MIDI.sendNoteOn(28, 127, 10);
          delay(100);
          MIDI.sendNoteOff(28, 127, 10);
          break;
        case 1:
          // C2
          MIDI.sendNoteOn(36, 127, 10);
          delay(100);
          MIDI.sendNoteOff(36, 127, 10);
          break;
        case 2:
          // F#2
          MIDI.sendNoteOn(42, 127, 10);
          delay(100);
          MIDI.sendNoteOff(42, 127, 10);
          break;
        case 3:
          // D#3
          MIDI.sendNoteOn(50, 127, 10);
          delay(100);
          MIDI.sendNoteOff(50, 127, 10);
          break;
      }
  }
}

long calculateBeatSectionTimingMicros() {
  // return 60000 / currentBpm / (BEAT_SECTIONS / 4);  FOR MILLIS
  return 60000000 / currentBpm / (BEAT_SECTIONS / 4);
}
