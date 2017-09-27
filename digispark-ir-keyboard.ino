/*************************************
*
* Microcontroller: ATtiny85
* Dev board:       Digispark
*
* Assumptions:
* a single byte read is always atomic
*************************************/

//#define DEBUG

#ifdef DEBUG
  #include <DigiUSB.h>
#else
  #include "TrinketHidCombo.h"
#endif

#define SEC       1000000
#define MS        1000
#define US        1
#define TOLERANCE 20
#define PRESS_DELAY 200

#define STATUS_LED 1

#define VOL_UP   224
#define VOL_DOWN 208
#define MUTE     240

#define STATE_NEED_START  0
#define STATE_NEED_START2 1
#define STATE_DATA        2
#define STATE_DATA_READY  3

// timestamp type (for micros() / millis())
typedef unsigned long TS;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;

volatile TS lastRead = 0;
volatile uint8 pulseNum = 0;
volatile uint8 data = 0;
volatile uint8 state = STATE_NEED_START;
TS lastPress = 0;

void reset(void);

void DEFAULT_setupMisc(void);
void DEBUG_setupMisc(void);

void DEFAULT_loopMisc(void);
void DEBUG_loopMisc(void);

void DEFAULT_handleReceivedData(void);
void DEBUG_handleReceivedData(void);

#ifndef DEBUG
#define setupMisc DEFAULT_setupMisc
#define loopMisc DEFAULT_loopMisc
#define handleReceivedData DEFAULT_handleReceivedData

void DEFAULT_setupMisc(void) {
  TrinketHidCombo.begin();
}

void DEFAULT_handleReceivedData(void) {
  TS now = millis();
  if (lastPress != 0) {

    // overflow
    if (lastPress > now) {
      lastPress = 0;
      return;
    }

    // a single press, every 150ms max
    if (now - lastPress < PRESS_DELAY)
      return;
  }

  switch(data) {
    case VOL_UP:
      TrinketHidCombo.pressMultimediaKey(MMKEY_VOL_UP);
      break;
    case VOL_DOWN:
      TrinketHidCombo.pressMultimediaKey(MMKEY_VOL_DOWN);
      break;
    case MUTE:
      TrinketHidCombo.pressMultimediaKey(MMKEY_MUTE);
      break;
    default:
      return;
  }

  lastPress = now;
}

void DEFAULT_loopMisc(void) {
  TrinketHidCombo.poll();
}

#else

  #define setupMisc DEBUG_setupMisc
  #define loopMisc DEBUG_loopMisc
  #define handleReceivedData DEBUG_handleReceivedData

void DEBUG_setupMisc(void) {
  DigiUSB.begin();
}

void DEBUG_loopMisc(void) {
  DigiUSB.refresh();
}

void DEBUG_handleReceivedData(void) {
  switch(data) {
    default:
      DigiUSB.print("received data: ");
      DigiUSB.println(data, DEC);
      DigiUSB.println(pulseNum, DEC);
      DigiUSB.println(state, DEC);
      break;
  }
}
#endif

void setup(void) {
  attachInterrupt(0, readIR, FALLING);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  setupMisc();
}

void loop(void) {
  TS now = micros();

  // we managed to get called while already reading the IR signal, wait for it
  if (state == STATE_DATA_READY) {
    handleReceivedData();
    reset();
  }

  loopMisc();
}

void readIR(void) {
  TS now = micros();

  // overflow, our last read should not be in the future
  if (lastRead != 0 && lastRead > now) {
    reset();
    return;
  }

  uint16 delta = now - lastRead;

  // timeout
  if (lastRead != 0 && delta > 40*MS)
    reset();

  switch(state) {
    case STATE_NEED_START:
      // just check and prepare the state, the next pulse has to arrive in
      // a specific interval, checking that will be the next state
      if (lastRead == 0) {
        lastRead = now;
        state = STATE_NEED_START2;
        return;
      }

      break;

    case STATE_NEED_START2:
      // the signal "start" is represented by pulling the pin low for ~5ms
      // than pulling it high for ~5ms
      // (we wont see it because this is falling edge triggered),
      // so the next pulse must be in the range 7ms > delta < 11ms
      // any other timing signals an error, we go back to the start state
      if (delta > 7*MS && delta < 11*MS) {
        pulseNum = 0;
        state = STATE_DATA;
      } else {
        reset();
        return;
      }
      break;

    case STATE_DATA:
    // the block is necessary to be able to declare the dataBit variable here
    {
      // if the last pulse was more than 1.5ms ago than it must be a "1"
      // because the duration of the pulldown is ~500us, near the beginning
      // we receive the interrupt, the next pulldown will be after that one
      // by about ~600us-re at a minimum,
      // if it is so, than the value is 0,
      // at a maximum we miss the ~500us pulldown (signaling a "1"),
      // but there has to be a pulldown after ~2ms, if there isn't its irregular
      // ergo we have about ~2.5ms windows where we count the number of pulldowns
      // if there is only a single pulldown in that time, it gets the value "1"
      // if there are two pulldowns (the max), its value is "0" (arbitrarily)
      // any pulses outside of these is unexpected, and it resets the state

      uint8 dataBit = 0;
      if (delta > 1*MS + 500*US - TOLERANCE && delta < 2*MS + 500*US + TOLERANCE)
        dataBit = 1;
      else if (delta > 1*MS - TOLERANCE && delta < 1*MS + 500*US + TOLERANCE)
        ;
      else {
        // if the pulse is irregular, just reset the state
        reset();
        return;
      }

      // the first 16 bits is the address we dont care about
      // the 8 bits after that is the code,
      // the last 8 bits is the code inverted, again we dont care
      if (pulseNum >= 16 && pulseNum < 24) {
        data |= dataBit;

        // shift everything over except when on the last bit
        if (pulseNum < 23)
          data <<= 1;
      }

      // cannot overflow
      ++pulseNum;
      if (pulseNum >= 32)
        state = STATE_DATA_READY;

      break;
    }

    default:
      // basically the DATA_READY state,
      // just ignore it until loop() handles the data
      break;
  }

  lastRead = now;
}

void inline reset(void) {
  noInterrupts();
  lastRead = 0;
  interrupts();

  data = pulseNum = 0;
  state = STATE_NEED_START;
}
