#include "Mouse.h"

#if !defined(__AVR_ATmega32U4__)
  #define _printf Serial.printf
#endif

#define POINTING_DEVICE true //either mouse or tackball enabled
#define SCROLL_WHEEL true //scroll wheel enabled
#define BUTTONS true //buttons enabled
#define MOUSE_SCALE 8L //scale-down of mouse or trackball
#define TRACKBALL true //flips x axis for trackball
#define SCROLL_SCALE 2L //scale-down of scroll wheel


//debug modes
#define MOUSE_DEBUG false
#define SCROLL_DEBUG false
#define BUTTON_DEBUG false
#define DEBUG MOUSE_DEBUG||SCROLL_DEBUG||BUTTON_DEBUG

// PMW3610 register addresses
#define REG_REVID 0x01 //revision
#define REG_MOTION 0x02 //motion flags
#define REG_DELTA_X_L 0x03 //dx lower 8 bits
#define REG_DELTA_Y_L 0x04 //dy lower 8 bits
#define REG_DELTA_XY_H 0x05 //dxdy upper 4 bits

#if POINTING_DEVICE
  #define SDIO SDA //data I/O pin
  #define SCK SCL //clock pin
  #define CS D5 //chip select
#endif

#if SCROLL_WHEEL
  #define SCROLL_PIN_A A2
  #define SCROLL_PIN_B A3
  #define SCROLL_SCALE 2L
  #define SCROLL_DEBOUNCE_TIME 1L //assume no one can scroll > 1000 lines per second

  byte scrollState=0; //bits 3&2: last quadrature reading; 1&0: current reading
  long scrollValue=0; //accumulated scroll since last report
  unsigned long lastScrollReadingTime = 0;
#endif


#if POINTING_DEVICE
  int16_t xValue=0; //accumulated dx
  int16_t yValue=0; //accumulated dy
#endif

#if BUTTONS
  //button arrays
  uint8_t buttonPins[] = {D9,D10,D11,D12,D13}; //pins for mouse buttons: left, right, middle, back. forward
  uint8_t buttonBits[] = {1,2,4,8,16};
  uint8_t buttonStates[] = {HIGH,HIGH,HIGH,HIGH,HIGH};
  uint8_t buttonLastStates[] = {HIGH,HIGH,HIGH,HIGH,HIGH};
#endif

//set up pins and mouse
void setup()
{
  if(SCROLL_DEBUG || BUTTON_DEBUG || MOUSE_DEBUG) Serial.begin(115200);
  Mouse.begin();
  #if BUTTONS
    //set up mouse button pins
    for(int i=0;i<5;i++)
    {
      pinMode(buttonPins[i], INPUT_PULLUP);
    }
  #endif

  delay(500);

  #if POINTING_DEVICE
    //set up PMW3610 interface
    pinMode(SCK, OUTPUT); //clock
    pinMode(SDIO, OUTPUT); //data in/out
    pinMode(CS, OUTPUT);  //chip select
  #endif


  //set up serial debugging
  #if DEBUG
    while (!Serial) delay(500);
  #endif

  #if SCROLL_WHEEL
    //set up scroll wheel quadrature pins
    pinMode(SCROLL_PIN_A, INPUT_PULLUP);
    pinMode(SCROLL_PIN_B, INPUT_PULLUP);
    delay(5);
    scrollInt(); //prime scrollstate with current value
    //attach interrupts to scroll wheel quadrature pins
    attachInterrupt(digitalPinToInterrupt(SCROLL_PIN_B),scrollInt,CHANGE);
    attachInterrupt(digitalPinToInterrupt(SCROLL_PIN_A),scrollInt,CHANGE);
  #endif
}

void loop()
{

  #if POINTING_DEVICE
    //get the PMW3610 motion flag
    uint8_t motion = readRegister(REG_MOTION); 
  #endif

  #if SCROLL_WHEEL
    //scale down the scroll value
    long sv=scrollValue/SCROLL_SCALE; 
    //subtract the scaled value from the scroll value, leaving the remainder
    scrollValue-=sv*SCROLL_SCALE; 
  #else
    long sv=0;
  #endif

  #if POINTING_DEVICE && !SCROLL_WHEEL
    if (motion & 0x80) // Check if motion occurred (bit 7 = 1)
  #endif
  #if !POINTING_DEVICE && SCROLL_WHEEL
    if (sv!=0 ) // Check if scroll occurred 
  #endif
  #if POINTING_DEVICE && SCROLL_WHEEL
    if ((motion & 0x80) || sv!=0 ) // Check if motion occurred (bit 7 = 1) or scroll occurred
  #endif
    { 
      #if POINTING_DEVICE
        if(MOUSE_DEBUG && (motion & 0x80)) _printf("Mouse motion: %02x \n", motion);
        uint8_t dxl = readRegister(REG_DELTA_X_L); //get lower 8 bits of x
        uint8_t dyl = readRegister(REG_DELTA_Y_L); //get lower 8 bits of y
        uint8_t dxy = readRegister(REG_DELTA_XY_H); //get upper 4 bits of x&y
        if(MOUSE_DEBUG && (motion & 0x80)) _printf("Mouse dxy,dxl,dyl: %02x, %02x, %02x\n", dxy, dxl, dyl);
        int16_t dx = ((uint16_t)dxy & 0xF0) << 4;
        if (dx > 0x7FF) //if negative, set upper bits
          dx |= 0xF000;
        int16_t dy = ((uint16_t)dxy & 0x0F) << 8;
        if (dy > 0x7FF) //if negative, set upper bits
          dy |= 0xF000;
        dx = dx | dxl; //add lower 8 bits of X
        dy = dy | dyl; //add lower 8 bits of Y
        //accumulate dx and dy
        xValue+=dx;
        yValue+=dy;
        //scale down the xy values
        int16_t xv=xValue/MOUSE_SCALE; 
        int16_t yv=yValue/MOUSE_SCALE; 
        //subtract the scaled values from the xy values, leaving the remainders
        xValue-=xv*MOUSE_SCALE; 
        yValue-=yv*MOUSE_SCALE; 
      #else
        int16_t xv=0;
        int16_t yv=0;
      #endif
      #if POINTING_DEVICE || SCROLL_WHEEL
        //send x,y and scroll to USB
        Mouse.move(constrain(TRACKBALL?-xv:xv, -127, 127),constrain(-yv, -127, 127),constrain(sv, -127, 127));
      #endif
      #if POINTING_DEVICE && MOUSE_DEBUG
        if(MOUSE_DEBUG) _printf("Mouse dxdy: %04x, %04x (%08d, %08d)\n", dx, dy, dx, dy);
        if(MOUSE_DEBUG) _printf("Mouse scaled dxdy: %04x, %04x (%08d, %08d)\n", xv, yv, xv, yv);
      #endif
    }
  #if BUTTONS
    //send any button events
    scanButtons();
  #endif
  delay(20); 
}

#if POINTING_DEVICE
  //read a PMW3610 register
  uint8_t readRegister(uint8_t addr)
  {
    digitalWrite(SCK, HIGH); //clock high
    digitalWrite(CS, LOW); //enable chip
    uint8_t _addr = addr & 0x7F; //ensure you're doing a read operation
    shiftOut(_addr); //send the register address
    delayMicroseconds(1); //wait a bit before reading the results
    uint8_t dataRead = shiftIn(); //get the register data
    digitalWrite(CS, HIGH); //disable chip
    return dataRead; //return result
  }

  //serialize a byte to the PMW3610
  void shiftOut(uint8_t b)
  {
    uint8_t d=b;
    pinMode(SDIO, OUTPUT); //sending bits to PMW3610
    for(int i=0;i<8;i++) //loop through 8 bits
    {
      digitalWrite(SCK, LOW); //falling clock edge
      digitalWrite(SDIO, ((b&0x80)!=0)?HIGH:LOW); //send next bit
      delayMicroseconds(1); //wait for a bit before latching the data
      digitalWrite(SCK, HIGH); //rising clock edge
      delayMicroseconds(1); //wait a bit before sending next bit
      b=b<<1; //shift next bit into the MSB
    }
  }

  //deserialize a byte from the PMW3610
  uint8_t shiftIn()
  {
    uint8_t b=0; //start with 0
    pinMode(SDIO, INPUT); //receiving bits from PMW3610
    delayMicroseconds(4); //wait for data to be available
    for(int i=0;i<8;i++)
    {
      digitalWrite(SCK, LOW); //falling edge of clock
      delayMicroseconds(1); //wait for bit to settle
      b=(b<<1)|((digitalRead(SDIO)==HIGH)?1:0); //shift the bit into the LSB
      digitalWrite(SCK, HIGH); //rising edge of clock
      delayMicroseconds(1); //wait a bit before retrieving next
    }
    return b; //return result
  }
#endif

#if SCROLL_WHEEL
  //interrupt handler for scroll wheel
  //called on any change in pin states
  void scrollInt()
  {
    unsigned long time=millis();
    if(SCROLL_DEBUG) _printf("Time since last scroll sample: %10d ms\n", time-lastScrollReadingTime);
    if(time>lastScrollReadingTime+SCROLL_DEBOUNCE_TIME)
    {
      //shift quadrature state into lower 2 bits of scrollState
      //and previous state into upper 2 bits
      byte reading = (digitalRead(SCROLL_PIN_A)<<1)|digitalRead(SCROLL_PIN_B);
      if(reading!=(scrollState&3)) //process only actual state changes
      {
        scrollState=((scrollState&0x3)<<2)|reading;
        switch (scrollState) 
        {
          case 0x1: case 0x7: case 0x8: case 0xE: scrollValue++; break; //quadrature readings indicate an increment
          case 0x2: case 0x4: case 0xB: case 0xD: scrollValue--; break; //quadrature readings indicate a decrement
          //all other combinations indicate no change
        }
        lastScrollReadingTime=time;
        if(SCROLL_DEBUG) _printf("Scroll state: %02x, Scroll value %10d \n",scrollState,scrollValue);
      }
    }
  }
#endif

#if BUTTONS
  //sample each button pin; if different from last state, send mouse button event
  void scanButtons()
  {
    //get current button states
    for(int i=0;i<5;i++)
    {
      buttonStates[i]=digitalRead(buttonPins[i]);
    }
    //send mouse action on change
    for(int i=0;i<5;i++)
    {
      if(buttonStates[i]!=buttonLastStates[i])
      {
        if(BUTTON_DEBUG) _printf("Button %4d changed\n",i);
        if(buttonStates[i]==LOW)
        {
          Mouse.press(buttonBits[i]);
          if(BUTTON_DEBUG) _printf("Button %4d pressed\n",i);
        }
        else
        {
          Mouse.release(buttonBits[i]);
          if(BUTTON_DEBUG) _printf("Button %4d released\n",i);
        }
        buttonLastStates[i]=buttonStates[i];
      }
    }
  }
#endif

#if defined(__AVR_ATmega32U4__)
  void _printf(char *fmt, ...) {
    va_list va;
    va_start(va, fmt);
    char buf[vsnprintf(NULL, 0, fmt, va) + 1];
    vsprintf(buf, fmt, va);
    Serial.print(buf);
    va_end(va);
  }
#endif
