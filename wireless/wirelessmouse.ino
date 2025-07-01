#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"

//32u4 doesn't have Serial.printf
#if !defined(__AVR_ATmega32U4__)
  #define _printf Serial.printf
#endif

#define POINTING_DEVICE true //either mouse or tackball enabled
#define SCROLL_WHEEL true //scroll wheel enabled
#define BUTTONS true //buttons enabled
#define MOUSE_SCALE 1L //scale-down of mouse or trackball
#define TRACKBALL false //flips x axis for trackball
#define SCROLL_SCALE 2L //scale-down of scroll wheel

//command for naming the device
#define NAME_PREFIX "SD "
#define NAME_CMD "AT+GAPDEVNAME="
#if POINTING_DEVICE && TRACKBALL
  #define BT_NAME NAME_CMD NAME_PREFIX "Trackball"
#elif POINTING_DEVICE && !TRACKBALL
  #define BT_NAME NAME_CMD NAME_PREFIX "Mouse"
#elif SCROLL_WHEEL
  #define BT_NAME NAME_CMD NAME_PREFIX "Scroll Wheel"
#else
  #define BT_NAME NAME_CMD NAME_PREFIX "Mouse Buttons"
#endif

//debug modes
#define MOUSE_DEBUG false
#define SCROLL_DEBUG false
#define BUTTON_DEBUG false
#define BLE_DEBUG false

#define BLUEFRUIT_SPI_CS 8 //BTLE chip select
#define BLUEFRUIT_SPI_IRQ 7 //BTLE interrupt
#define BLUEFRUIT_SPI_RST 4 //BTLE reset

#define DEBUG MOUSE_DEBUG||SCROLL_DEBUG||BUTTON_DEBUG||BLE_DEBUG
#define MINIMUM_FIRMWARE_VERSION    "0.8.1"

// PMW3610 register addresses
#define REG_REVID 0x01 //revision
#define REG_MOTION 0x02 //motion flags
#define REG_DELTA_X_L 0x03 //dx lower 8 bits
#define REG_DELTA_Y_L 0x04 //dy lower 8 bits
#define REG_DELTA_XY_H 0x05 //dxdy upper 4 bits

//PMW3610 pins
#define MSDIO A0 //data I/O pin
#define MSCK A1 //clock pin
#define MCS A4 //chip select

//scroll wheel pins; must be interrupt inputs
#if SCROLL_WHEEL
  #if defined(__AVR_ATmega32U4__)
    #define SCROLL_PIN_A 2
    #define SCROLL_PIN_B 3
  #else
    #define SCROLL_PIN_A A2
    #define SCROLL_PIN_B A3
  #endif
  #define SCROLL_DEBOUNCE_TIME 1L //assume no one can scroll > 1000 lines per second
#endif

byte scrollState=0; //bits 3&2: last quadrature reading; 1&0: current reading
long scrollValue=0; //accumulated scroll since last report
unsigned long lastScrollReadingTime = 0;

#if POINTING_DEVICE
  int16_t xValue=0; //accumulated dx
  int16_t yValue=0; //accumulated dy
#endif

#if BUTTONS
  //button arrays
  int buttonPins[] = {9,10,11,12,13}; //pins for mouse buttons: left, right, middle, back, forward
  char buttonNames[] = {'L','R','M','B','F'};
  byte buttonStates[] = {HIGH,HIGH,HIGH,HIGH,HIGH};
  byte buttonLastStates[] = {HIGH,HIGH,HIGH,HIGH,HIGH};
#endif

//create bluefruit object
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

//setup
void setup(void)
{
  if(SCROLL_DEBUG || BUTTON_DEBUG || MOUSE_DEBUG) Serial.begin(115200);

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
    pinMode(MSCK, OUTPUT); //clock
    pinMode(MSDIO, OUTPUT); //data in/out
    pinMode(MCS, OUTPUT);  //chip select
  #endif

  //set up serial debugging
  #if DEBUG
    while (!Serial) delay(500);
    Serial.print("Initialising the Bluefruit module: ");
  #endif

  if ( !ble.begin(DEBUG))
  {
  #if DEBUG
    Serial.println("Couldn't find Bluefruit.");
  #endif
  }

  #if DEBUG
    Serial.println("OK");
  #endif

  //Disable command echo
  ble.echo(false);

  #if DEBUG
    Serial.println("Requesting Bluefruit info:");
    ble.info();
  #endif

  //Set device name
  if (! ble.sendCommandCheckOK(F(BT_NAME)) ) 
  {
  #if DEBUG
    Serial.println("Could not set device name.");
  #endif
  }

  // Enable HID Service
  #if DEBUG
    Serial.println("Enable HID Service: ");
  #endif
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    if ( !ble.sendCommandCheckOK(F( "AT+BleHIDEn=On" ))) {
    #if DEBUG
      Serial.println("Could not enable HID");
    #endif
    }
  }else
  {
  #if DEBUG
    Serial.print("Firmware must be at least ");
    Serial.println(MINIMUM_FIRMWARE_VERSION);
  #endif
 }

  #if DEBUG
    Serial.println("SW reset.");
  #endif
  if (!ble.reset() ) {
  #if DEBUG
    Serial.println("Couldn't reset.");
  #endif
  }

  //Turn off mode LED
  #if !DEBUG
    ble.sendCommandCheckOK(F( "AT+HWMODELED=0" )); 
  #endif

  ble.setMode(BLUEFRUIT_MODE_DATA);
  ble.echo(false);
  ble.verbose(false);

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

char cmd[50]; //BLE command

void loop(void)
{
  #if POINTING_DEVICE
    //get the PMW3610 motion flag
    byte motion = readRegister(REG_MOTION); 
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
        byte dxl = readRegister(REG_DELTA_X_L); //get lower 8 bits of x
        byte dyl = readRegister(REG_DELTA_Y_L); //get lower 8 bits of y
        byte dxy = readRegister(REG_DELTA_XY_H); //get upper 4 bits of x&y
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
        //send x,y and scroll to bluetooth
        #if BLE_DEBUG 
          unsigned long bleStart=millis();
        #endif
        sprintf(cmd,"AT+BLEHIDMOUSEMOVE=%d,%d,%d,", constrain(TRACKBALL?-xv:xv, -127, 127),constrain(-yv, -127, 127),constrain(sv, -127, 127)); 
        bool blst=ble.sendCommandCheckOK(cmd);
        #if BLE_DEBUG 
          _printf("BLE transmit time: %d ms. Command: %s. Response: %x\n",millis()-bleStart,cmd,blst);
        #endif
      #endif

      #if POINTING_DEVICE && MOUSE_DEBUG
        _printf("Mouse dxdy: %04x, %04x (%08d, %08d)\n", dx, dy, dx, dy);
        _printf("Mouse scaled dxdy: %04x, %04x (%08d, %08d)\n", xv, yv, xv, yv);
      #endif
    }
  #if BUTTONS
    //send any button events
    scanButtons();
  #endif
  delay(10);
}

#if BUTTONS
  //sample each button pin; if at least one is different from last state, send mouse button event
  void scanButtons()
  {
    //get current button states
    for(int i=0;i<5;i++)
    {
      buttonStates[i]=digitalRead(buttonPins[i]);

    }
    boolean chg=false; //at least one button has changed
    boolean pressed=false; //at least one button is pressed
    String pressedButtons="";
    //gather button states
    for(int i=0;i<5;i++) //loop through all 5 buttons
    {
      if(buttonStates[i]==LOW) //if button is pressed
      {
        pressedButtons+=buttonNames[i]; //add to list of pressed buttons
        pressed=true; //at least one is pressed
      }
      if(buttonStates[i]!=buttonLastStates[i]) //if the button state has changed (either up or down)
      {
        chg=true; //at least one change
        buttonLastStates[i]=buttonStates[i]; //update previous state
      }
    }
    if(chg) //if any change in mouse button state, send an update
    {
      #if BUTTON_DEBUG
        Serial.print("chg=");
        Serial.print(chg);
        Serial.print(" pressed=");
        Serial.print(pressed);
        Serial.print(" buttons=");
        Serial.println(pressedButtons);
      #endif
      if(pressed) //if at least one pressed, send the list of pressed buttons
      {
        sprintf(cmd, "AT+BLEHIDMOUSEBUTTON=%s", pressedButtons.c_str());
        ble.sendCommandCheckOK(cmd);
      }
      else //if no buttons pressed, send zero
      {
        ble.sendCommandCheckOK("AT+BLEHIDMOUSEBUTTON=0");
      }
    }
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

#if POINTING_DEVICE
  //read a PMW3610 register
  byte readRegister(byte addr)
  {
    digitalWrite(MSCK, HIGH); //clock high
    digitalWrite(MCS, LOW); //enable chip
    byte _addr = addr & 0x7F; //ensure you're doing a read operation
    shiftOut(_addr); //send the register address
    delayMicroseconds(1); //wait a bit before reading the results
    byte dataRead = shiftIn(); //get the register data
    digitalWrite(MCS, HIGH); //disable chip
    return dataRead; //return result
  }

  //serialize a byte to the PMW3610
  void shiftOut(byte b)
  {
    byte d=b;
    pinMode(MSDIO, OUTPUT); //sending bits to PMW3610
    for(int i=0;i<8;i++) //loop through 8 bits
    {
      digitalWrite(MSCK, LOW); //falling clock edge
      digitalWrite(MSDIO, ((b&0x80)!=0)?HIGH:LOW); //send next bit
      delayMicroseconds(1); //wait for a bit before latching the data
      digitalWrite(MSCK, HIGH); //rising clock edge
      delayMicroseconds(1); //wait a bit before sending next bit
      b=b<<1; //shift next bit into the MSB
    }
  }

  //deserialize a byte from the PMW3610
  byte shiftIn()
  {
    byte b=0; //start with 0
    pinMode(MSDIO, INPUT); //receiving bits from PMW3610
    delayMicroseconds(4); //wait for data to be available
    for(int i=0;i<8;i++)
    {
      digitalWrite(MSCK, LOW); //falling edge of clock
      delayMicroseconds(1); //wait for bit to settle
      b=(b<<1)|((digitalRead(MSDIO)==HIGH)?1:0); //shift the bit into the LSB
      digitalWrite(MSCK, HIGH); //rising edge of clock
      delayMicroseconds(1); //wait a bit before retrieving next
    }
    return b; //return result
  }
#endif

//32u4 doesn't have Serial.printf
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

