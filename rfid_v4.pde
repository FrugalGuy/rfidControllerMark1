/*
  RFID Door Lock
  10APR2010, 27Aug2010
  Ron Craig
  roncraig007@gmail.com
  
  This sketch implements a controller for an RFID door lock system.
  It reads 125 KHz RFID cards and sends a signal to unlock a 
  door if the presented card is a valid card stored in EEPROM.
  
  Additional features:
  1) System begins in 'factory default' mode. This is a 'virgin'
  state with no stored cards. When in this mode, lights flash in 
  a frenetic way. The first RFID card read after installation
  will become the 'master card' and factory default mode is exited.
  This little feature means you can burn tons of these chips and
  you don't need to hard-code any RFIDs. Suitable for shipping!
  
  2) Master card (first card read after installation) also opens
  the door, so you can just use that card for every day access
  instead of having to stash it somewhere.
  
  3) You can enter programming mode at any time with the Master card.
  To do this, you swipe the master card twice within a few seconds.
  First time opens the door, and second time (if door is still 
  unlocked) puts you in programming mode. You get sequential flashing 
  lights to indicate you are in programming mode.
  
  4) While in programming mode you can add any number of additional
  cards to the EEPROM. You can also remove any number of 
  cards from EEPROM while in this mode. You get a blinking green for
  adds and a blinking red for deletes. Previously unauthorized cards 
  will be added and authorized cards will be deleted. You exit 
  programming mode by swiping the master card again or after a period
  of inactivity.
  
  5) Master card and other cards get a green/red light indicating
  pass/fail when trying to open the door.
  
  OPTIONAL FEATURES
  1) Master Swap option allows you to change the Master without
  resetting to factory default mode and erasing the additional
  cards. This is useful if you lose the Master card. Hold optional
  controller reset button for around 5 seconds.
  2) Device can be returned to factory default mode by pressing the 
  optional reset button for 30 seconds.
  2) Stealth option allows you to add your own RFID card to the
  controller without having the Master card. Warning: Evil
  
  DEVELOPMENT FEATURES
  1) Define a debug flag to get serial debug output.
  
  FUTURE
  Data logging via wireless or HDSD card or enet/USB connector.
  Command line over serial to query and edit RFID database.
  IP web service interface for querying remote RFID db.
  
  HARDWARE CONFIGURATION
  I suggest two PCBs, one for the RFID reader and indicator lights 
  outside the door and a second for the controller inside. 
  Connect reader serial and light control lines to controller 
  located in a secure location (inside the access-controlled area). 
  Power for the door lock should be routed securely from the
  controller board to the door lock to prevent jumping. If you
  are providing a factory reset switch, some means of access to 
  the switch needs to be provided.
  
  COMPONENTS
  Mini SPST Push Button momentary reset switch (COM-00097 $0.35)
  
  ID-12/ID-20 RFID reader (SEN-08419 $29.95/SEN-08628 $34.95)

  Using www.sparkfun.com breakout board (SEN-08423 $0.95)
  [1] GND (Ground)
  [2] /RST (Reset Reader) Connect to +5V     
  [3] ANT (NC)  
  [4] ANT (NC)      
  [5] CP  (NC) (Card Present)     
  [6] NC
  [7] FS (Format select) (ground this for ASCII)
  [8] D1 (NC)
  [9] D0 (TTL Serial) to Arduino RX Pin
  [10] Buzzer/LED (Card read indicator) (Connect to transistor + buzzer/led to indicate card was read)
  [11] 5v+ ( +5vDC power) 

  On Arduino....
  [2] (Digital IN) to tactile push button reset switch connected to GND (optional)
  [13] (Digital OUT) to LED to indicate reset status (optional)
  [8] (Digital OUT) to lock relay+ 
  [9] (Digital OUT) to Red LED+ (fail/RFID erase indicator)
  [10] (Digital OUT) to Blue or Yellow LED+ (power indicator)
  [11] (Digital OUT) to Green LED+ (pass/RFID add indicator)
  [RX0] (Serial IN) to [9] D0 TTL serial out on ID-12/ID-20

// Expanded/modified from a version by Brett Martin www.pcmofo.com
// Based on code by BARRAGAN <http://people.interaction-ivrea.it/h.barragan> 
// and code from HC Gilje - http://hcgilje.wordpress.com/resources/rfid_id12_tagreader/
// Modified for Arudino by djmatic
// Modified for ID-12 and checksum by Martijn The - http://www.martijnthe.nl/
//
// Use the drawings from HC Gilje to wire up the ID-12/ID-20.
// Remark: disconnect the RX serial wire to the ID-12/ID-20 when uploading the sketch.


*/
  
#include <EEPROM.h>

// Version display. Visible with debug output
#define VERSION "RFID Controller V4 20100827 (c) Ron Craig 2010"

// Debug settings. Define DEBUG to get serial output. Undefine otherwise.
#define DEBUG
//#undef DEBUG
#ifdef DEBUG
#define DEBUGPRINT(...) {Serial.print(__VA_ARGS__);}
#define DEBUGPRINTLN(...) {Serial.println(__VA_ARGS__);}
#else
#define DEBUGPRINT(...) { }
#define DEBUGPRINTLN(...) { }
#endif

// outputs to reader board
#define failPin   9      // a red LED
#define powerPin 10      // a power LED
#define passPin  11      // a green LED

// Optional reset functions. Define RESET to get it. Undefine otherwise.
#define RESET
//#undef RESET
#ifdef RESET
#define resetPin  2      // optional factory reset 
#define resetLED 13      // optional reset indicator
#endif

#define doorPin   8      // signals door lock/relay

#define MAX_CARDS     30  // max cards to store in EEPROM (1 + 5*MAX_CARDS bytes required)

#define doorDelay   5000  // how long to keep door lock powered open (ms)
#define pgmDelay   30000  // how long to stay in programming mode without activity
#ifdef RESET
#define swapMin     5000  // how long you have to hold reset before you can read a new master card (ms)
#define factoryMin 30000  // how long you have to hold reset before entire EEPROM is cleared (ms)
#define resetDebounce 10  // prevent switch noise. Increase for noisy tactile switches.
#endif

boolean inited = false;   // factory mode. If this is false, next card swiped becomes master card.
boolean pgmMode = false;  // programming mode
boolean pgmPend = false;  // programming mode pending (1 swipe)
unsigned long pModeTime = 0;     // programming mode timer
unsigned long doorTime  = 0;     // door open timer
#ifdef RESET
unsigned long lastDebounceTime = 0;
int buttonState = HIGH;
int lastButtonState = HIGH;  // default state of pin is HIGH
#endif

byte storedCard[5];      // card pulled from EEPROM
byte readCard[6];        // card received from reader ([5] is checksum)
byte checksum = 0;       // independently calculated checksum

                   // overwritten from EEPROM during setup()
byte masterCard[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0xED};
byte cardCount = 0;

void setup()
{
  Serial.begin(9600);
  DEBUGPRINTLN(VERSION)
  // Uncomment to set reader to Factory Settings on powerup. 
  // Alternatively, hold controller reset button for factoryMin milliseconds.
  // EEPROM.write(0,0);        // initializes to factory settings
  //
  pinMode(powerPin, OUTPUT);
  pinMode(passPin,  OUTPUT);
  pinMode(failPin,  OUTPUT);
  pinMode(doorPin,  OUTPUT);

#ifdef RESET  
  // Optionally connect to reset switch to GND. If you don't it will stay high forever.
  pinMode(resetPin, INPUT);      // Chip has internal pullup resistor. http://www.arduino.cc/en/Tutorial/DigitalPins
  digitalWrite(resetPin, HIGH);  // This will hold level high until you ground it w/switch.
  pinMode(resetLED, OUTPUT);
  DEBUGPRINTLN("Reset feature included")
#endif
  
  inited = checkInit();       // also sets masterCard if known
  DEBUGPRINT("inited? ") DEBUGPRINTLN(inited, HEX)
}

boolean checkInit()
{
    int cnt = EEPROM.read(0);  // (0) holds number of known RFIDs
    if (cnt > 0) readMaster(); // initializes masterCard
    DEBUGPRINT(cnt, HEX)  DEBUGPRINTLN(" cards in EEPROM")
    return (cnt > 0);
}


void loop()
{
  byte val = 0;

#ifdef RESET
  int reading = digitalRead(resetPin);   // Read the reset switch. If none installed, no problem.
  checkReset(reading); // if reset held long enough, clear master, maybe other cards, enter factory mode
#endif

  if (!inited)
  {
    factoryMode();  // will set inited and masterCard when a master card is stored
    return;         // skip all other processing while not inited
  }

  if (doorTime)
  {
    checkDoor();  //resets doorTime if expired
  }
  else if (pgmPend)
  {
    DEBUGPRINTLN("pgmPend-canceled")
    pgmPend = false;
  }

  if (pgmMode)
  {
    if ( (millis() - pModeTime) > pgmDelay )
    {
      DEBUGPRINTLN("pgmMode-exit")
      pgmMode = false; pModeTime = 0;
    }//pgmDelay expired
    else  // still in programming mode
    {
      pgmModeOn(); // cycle lights once
      if (Serial.available() > 0)
      {
        if ( (val = Serial.read()) == 2 )
        {
          DEBUGPRINT("pgmMode.")
          getID();  // sets 'readCard' and 'checksum'
          pModeTime = millis(); // continue programming mode another 30 secs
          pgmPend = false;
          
          if ( ! isMasterCard(readCard) )
          {
            DEBUGPRINT("!Master.")
            if (int cardnum = findID(readCard) != -1)
            {
              DEBUGPRINTLN("known-eraseCard")
              eraseID(cardnum);
            }//known card
            else
            {
              DEBUGPRINTLN("unknown-addCard")
              addID(readCard);
            }//not known card
          }//not isMasterCard
          else
          {
            DEBUGPRINTLN("isMaster-quitPgm-openDoor")
            pgmMode = false;
            openDoor();
          }// isMasterCard
        }//STX read
      }//Serial.available
    }//pgmDelay not expired
  }//pgmMode
  
  else  // not in programming mode
  {
    normalModeOn();  // adjust lights for normal operation
    if (Serial.available() > 0)
    {
      if ( (val = Serial.read()) == 2)
      {
        DEBUGPRINT("normal.")
        getID();  // sets 'readCard' and 'checksum'
        if ( readCard[5] == checksum )
        {
          DEBUGPRINT("goodChecksum.")
          if ( isMasterCard(readCard) )
          {
            DEBUGPRINT("isMaster.")
            if (pgmPend)
            {
              DEBUGPRINTLN("pgmMode")
              pgmMode = true; pModeTime = millis();
              pgmPend = false;
            }//pgmPend
            else
            {
              DEBUGPRINTLN("pgmPend-openDoor")
              openDoor();
              pgmPend = true;
            }//not pgmPend
          }//isMasterCard
          else
          {
            DEBUGPRINT("!Master.")
            pgmPend = false;  // read a non-master. cancel pending state
            if ( findID(readCard) != -1 )
            {
              DEBUGPRINTLN("known-openDoor")
              openDoor();
            }//known card
            else
            {
              DEBUGPRINTLN("unknown-failed")
              failed();
            }//not known card
          }//not isMasterCard
        }//checksum correct
        else DEBUGPRINTLN("bad-checksum")
      }//STX read
    }//Serial.available
  }//not pgmMode
} //loop

// changes lights, opens the door and starts a timer
// sets doorTime global variable
void openDoor()
{
  digitalWrite(powerPin, LOW);
  digitalWrite(failPin,  LOW);
  digitalWrite(passPin,  HIGH);
  digitalWrite(doorPin,  HIGH);
  doorTime = millis();
}//openDoor

// if the time for open door has elapsed, 
// then lock door, turn pass LED off, and reset timer doorTime
// otherwise, turn pass LED on
void checkDoor()
{
  if (millis() - doorTime > doorDelay)
  {
    DEBUGPRINTLN("closeDoor")
    digitalWrite(doorPin, LOW);
    digitalWrite(passPin, LOW);
    doorTime = 0;
  }//expired
  else
  {
    digitalWrite(passPin, HIGH); // likely was shut off elsewhere
  }//not expired
}//checkDoor

// cycle the LEDs once per call
// this delays 600 ms in all before returning
// called while in programming mode
void pgmModeOn()
{
  digitalWrite(powerPin, LOW);
  digitalWrite(failPin,  LOW);
  digitalWrite(passPin,  HIGH);
  delay(200);
  digitalWrite(powerPin, LOW);
  digitalWrite(failPin,  HIGH);
  digitalWrite(passPin,  LOW);
  delay(200);
  digitalWrite(powerPin, HIGH);
  digitalWrite(failPin,  LOW);
  digitalWrite(passPin,  LOW);
  delay(200);
}//pgmModeOn

// read the RFID from the reader
// sets readCard and checksum
void getID()
{
  byte cnt = 0;
  byte i = 0;
  byte val = 0;
  byte temp = 0;
  checksum = 0;    // reset with each call
  while ( cnt < 12 )
  {
    if (Serial.available() > 0)
    {
      val = Serial.read();
      if ( (val == 0x0D) || (val == 0x0A) || (val == 0x03) || (val == 0x02) )
        break;
      if ( (val >= '0') && (val <= '9') )
        val = val - '0';
      else if ( (val >= 'A') && (val <= 'F') )
        val = 10 + (val - 'A');
      
      if (cnt & 1 == 1)
      {
        readCard[cnt >> 1] = (val | (temp << 4) );
        if (cnt >> 1 != 5)
        {
          checksum ^= readCard[cnt >> 1];
        }//not done
      }//full byte read
      else
      {
        temp = val;
      }//not full byte read
      cnt++;        
    }//Serial.available
  }//while
}//getID

// return boolean telling if supplied IDs are the same
boolean areEqual(byte a[], byte b[])
{
  for (int i = 0; i < 5; i++)
  {
    if (a[i] != b[i]) return false; // return truth. breaks for loop
  }//for
  return true;
}//areEqual

// return boolean telling if supplied card is the master card
boolean isMasterCard(byte m[])
{
  if ( areEqual(m, masterCard) )
    return true;
  return false;
}//isMasterCard

// set storedCard to the ID in given EEPROM slot
void readID(int num)
{
  int start = (num * 5) - 4;
  for (int j = 0; j < 5; j++)
  {
    storedCard[j] = EEPROM.read(start + j);
  }//for
}//readID

// returns index of supplied ID if in EEPROM
// otherwise returns -1
int findID(byte f[])
{
  int cnt = EEPROM.read(0); // (0) holds number of active cards
  for (int i = 1; i <= cnt; i++)
  {
    readID(i);  // sets storedCard
    if ( areEqual(f, storedCard) )
      return i;  // return index. breaks for loop
  }//for
  return -1;     // return failure. did entire loop w/out success
}//findID

// erase an ID from EEPROM, given its 1-based position
void eraseID(int i)
{
  if (i < 1) return;
  int cnt = EEPROM.read(0); // (0) holds number of active cards
  if (i > cnt) return;
  if (i < cnt)
  {
    int start = (cnt * 5) - 4;
    int finish= (cnt - 1) * 5;
    byte val;
    for ( ; start <= finish; start++)
    {
      val = EEPROM.read(start + 5);
      EEPROM.write(start, val);
    }//for
  }//i < cnt
  EEPROM.write(0, --cnt); // if i==cnt just reduce the count
  failedWrite();
}//eraseID

// add the supplied ID to the EEPROM. increase count of cards
// blinks lights to indicate success or failure
void addID(byte a[])
{
  int cnt = EEPROM.read(0); // (0) holds number of active cards
  if ( (cnt < MAX_CARDS) &&
       (-1 == findID(a)) )
  {
    int start = (cnt * 5) + 1;
    for (int j = 0; j < 5; j++)
    {
      EEPROM.write( start + j, a[j] );
    }//for
    EEPROM.write(0, ++cnt);
    successWrite();        // blink success lights
  }//new card and room
  else
  {
    failedWrite();         // blink failure lights
  }//not new card or not room
}//addID

// set LEDs for normal operation
void normalModeOn()
{
  digitalWrite(powerPin, HIGH);
  //digitalWrite(passPin,  LOW); puts door open indicator at half-bright
  digitalWrite(failPin,  LOW);
}//normalModeOn

// flash failure LED
// this delays 2,500 ms before returning!!
void failed()
{
  digitalWrite(passPin,  LOW);
  digitalWrite(powerPin, LOW);
  digitalWrite(failPin, HIGH);
  delay(500);
  digitalWrite(failPin, LOW);
  delay(500);
  digitalWrite(failPin, HIGH);
  delay(500);
  digitalWrite(failPin, LOW);
  delay(500);
  digitalWrite(failPin, HIGH);
  delay(500);
  digitalWrite(failPin, LOW);
}//failed

// flash successful write LEDs
// delays 1000 ms before returning!
void successWrite()
{
  digitalWrite(failPin,  LOW);
  digitalWrite(powerPin, LOW);
  digitalWrite(passPin, HIGH);
  delay(200);
  digitalWrite(passPin, LOW);
  delay(200);
  digitalWrite(passPin, HIGH);
  delay(200);
  digitalWrite(passPin, LOW);
  delay(200);
  digitalWrite(passPin, HIGH);
  delay(200);
}//successWrite

// flash failed write LEDs
// delays 1000 ms before returning!
void failedWrite()
{
  digitalWrite(passPin,  LOW);
  digitalWrite(powerPin, LOW);
  
  digitalWrite(failPin, HIGH);
  delay(200);
  digitalWrite(failPin, LOW);
  delay(200);
  digitalWrite(failPin, HIGH);
  delay(200);
  digitalWrite(failPin, LOW);
  delay(200);
  digitalWrite(failPin, HIGH);
  delay(200);
}//successWrite

// called each loop while not inited
// records next RFID read as the master
// and sets the state to inited
void factoryMode()
{
    byte val;
    flashFactory();  
    if (Serial.available() > 0)
    {
        if ( (val = Serial.read()) == 2)
        {
            getID();                          // sets 'readCard' and 'checksum'
            if (readCard[5] == checksum)
            {
                DEBUGPRINTLN("factoryMode.cardRead")
                writeMaster(readCard); // sets 'masterCard', optional erases all others
                if ( findID(readCard) == 1 )
                {
                    inited = true;
                    DEBUGPRINTLN("factoryMode-exit")
                }
            }
        }
    }
}//factoryMode

// reads first card from EEPROM into masterCard
void readMaster()
{
    for (int i = 0; i < 5; i++)
    {
        masterCard[i] = EEPROM.read( i+1 );
    }
}//readMaster

// sets masterCard and first card slot in EEPROM to supplied value
// if there are no other cards (factory mode), marks new master as the only card
// otherwise does not disturb the card count, so you could replace the master card
void writeMaster(byte m[])
{
    for (int i = 0; i < 5; i++)
    {
        EEPROM.write( i+1, m[i] );
        masterCard[i] = m[i];
    }
    DEBUGPRINTLN("writeMaster.RfidWritten")
    if ((EEPROM.read(0) <= 0)) {
       EEPROM.write( 0, 1 );
       DEBUGPRINTLN("writeMaster.cardCountNow1")
    }
#ifdef RESET    
    digitalWrite(resetLED, LOW);
#endif
}//writeMaster

// this delays 500 ms before returning
// it flashes the lights to indicate the reader is in factory mode
void flashFactory()
{
    digitalWrite(powerPin, HIGH);
    digitalWrite(passPin,  LOW);
    digitalWrite(failPin,  LOW);
    delay(100);
    digitalWrite(powerPin, LOW);
    delay(100);
    digitalWrite(powerPin, HIGH);
    digitalWrite(passPin,  HIGH);
    digitalWrite(failPin,  HIGH);
    delay(100);
    digitalWrite(powerPin, LOW);
    delay(100);
    digitalWrite(powerPin, HIGH);
    digitalWrite(passPin,  LOW);
    digitalWrite(failPin,  LOW);
    delay(100);
}//flashFactory


// OPTIONAL FEATURES
// Without the ability to replace the master and/or erase all the cards, you have no way to program
// the reader (add or delete users) if the master card is lost. 
// Without these features you also cannot erase all the cards without having access to every user 
// card *and* the master card.  Pretty useful features, actually.
// Even if this code is included, you can still decide later whether to add a jumper or switch on the
// controller board.  The ATMega328 has pullup resisters on the digital pins, so if you don't add a 
// switch, this code just never thinks a switch was pressed. No worries about floating inputs.
#ifdef RESET
void checkReset(int reading)
{
  if (reading != lastButtonState) {      // If the switch changed, due to noise or pressing:
    lastDebounceTime = millis();           // reset the debouncing timer
  }   
  if ((millis() - lastDebounceTime) > resetDebounce) { // value unchanged for resetDebounce interval at least
    buttonState = reading;                                // set buttonState
  }
  if ((buttonState == LOW)) {  // Reset pressed
    if ((millis() - lastDebounceTime) > factoryMin) {  // stayed HIGH for longer factoryMin interval at least
      DEBUGPRINT("Reset pressed over ") DEBUGPRINT(factoryMin, DEC)  DEBUGPRINTLN(" milliseconds. Clearing EEPROM...")
      digitalWrite(resetLED, HIGH);  // Tell tech holding button that cards are cleared
      factoryReset();  // clears EEPROM and enters factory mode
    }
    else if ((millis() - lastDebounceTime) > swapMin) { // stayed HIGH for shorter swapMin interval at least
      DEBUGPRINT("Reset pressed over ")  DEBUGPRINT(swapMin, DEC)  DEBUGPRINTLN(" milliseconds.  Waiting for a new master...")
      digitalWrite(resetLED, HIGH);  // Tell tech holding button that master is cleared
      delay(50);  
      digitalWrite(resetLED, LOW);
      inited = false;  // toss into factory mode
    }
   }
  lastButtonState = reading;
}//checkReset

// reset EEPROM card count to zero and set inited to false
void factoryReset()
{
  if (EEPROM.read(0) != 0)  // don't write if already set
     EEPROM.write(0, 0);     // effectively erases all cards
  digitalWrite(resetLED, HIGH);
  DEBUGPRINTLN("factoryReset.EepromCleared")
  DEBUGPRINTLN("factoryMode-enter")
  inited = false;       // resets init flag, setting us to factory mode
}//factoryReset
#endif
