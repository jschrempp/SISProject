/******************************************************************************************************
SIS_TEST_TX:  program to make an SIS compatible test transmitter out of a Photon or Core.
    A Photon or Core is used to send bursts of SIS compatible codes out to an SIS compatible
    OOK transmitter.  The transmitter's "data in" line is connected to Core/Photon pin D0.  Note that
    the transmitter must be powered off of the +3.3 volt power from the Core/Photon unless a level
    shifter is used to boost the D0 signal up to the power supply level of the transmitter module.

    The program registers a Particle cloud function "sendBurst()" that is accessible via the cloud
    funcKey "xmit".  Each time that this cloud function is called, a burst of BURST_SIZE code words is
    transmitted.  The code that is transmitted will be between TX_CODE_INITIAL and
    TX_CODE_INITIAL + ( NUM_CODES - 1).  The code transmitted is incremented once after
    every transmission and resets to TX_CODE_INITIAL after the code code reaches
    TX_CODE_INITIAL + (NUM_CODES - 1).  The String argument to sendBurst() is not used
    in the function; it can be anything.

    written by:  Bob Glicksman; 4/29/16
******************************************************************************************************/
// #define DEBUG

// Global constants:
const unsigned long TX_CODE_INITIAL = 321790ul;
const int TX_PIN = D0;                  // transmitter on Digital pin D0
const int LED_PIN = D7;
const int BAUD_TIME = 400;              // basic signalling unit is 400 us
const int NUM_CODES = 10;              // we will rotate through 10 codes
const int BURST_SIZE = 50;              // when tripped, send out a burst of 50 code words


void setup()
{
  pinMode(TX_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  #ifdef DEBUG
    Serial.begin(9600);
  #endif

  Particle.function("xmit", sendBurst); // register the cloud function to send a burst of code words
}

void loop()
{
    // nothing to do in loop().  Action is cloud call to sendBurst()
}


/************************************ sendBurst(String) *************************************************
sendBurst(): transmit a burst of codewords.
    arguments:  foo - does nothing; not used.  Can be set to anything
    returns: an integer which is always zero.
*********************************************************************************************************/
int sendBurst(String foo)  // the argument is not used - can be any string
{
  static unsigned long txCode = TX_CODE_INITIAL;

  digitalWrite(LED_PIN, HIGH);
  // send BURST_SIZE number of code words
  for (int i = 0; i < BURST_SIZE; i++)
  {
    sendCodeWord(txCode);
  }

  // wait for 10 seconds before sending again
  digitalWrite(LED_PIN, LOW);

  if (txCode >= (TX_CODE_INITIAL + NUM_CODES - 1))
  {
      txCode = TX_CODE_INITIAL;
  }
  else
  {
      txCode++;
  }

  return 0;
}

/************************************ sendBurst(String) *************************************************
sendCodeWord(): formats up a code word and transmits zero and one codes accordingly.
    arguments:  code - the code word to transmit
    returns: nothing
*********************************************************************************************************/
void sendCodeWord(unsigned long code)
{
  const unsigned long MASK = 0x00800000ul;  // mask off all but lsb
  const int CODE_LENGTH = 24; // a code word is 24 bits + sync

  // send the code word bits
  for (int i = 0; i < CODE_LENGTH; i++)
  {
    if ( (code & MASK) == 0)
    {
      sendZero();
      #ifdef DEBUG
        Serial.print("0");
      #endif
    }
    else
    {
      sendOne();
      #ifdef DEBUG
        Serial.print("1");
      #endif
    }
    code = code <<1;
  }
  //send the sync
  sendSync();
  #ifdef DEBUG
    Serial.println(" SYNC");
  #endif

  return;
}

/***************************************** sendZero() ***************************************************
sendZero(): transmits a "zero" symbol that is compatible with EV1527 and PT2262 data
    arguments:  none
    returns: nothing
*********************************************************************************************************/
void sendZero()
{
  // a zero is represented by one baud high and three baud low
  digitalWrite(TX_PIN, HIGH);
  delayMicroseconds(BAUD_TIME);
  for(int i = 0; i < 3; i++)
  {
      digitalWrite(TX_PIN, LOW);
      delayMicroseconds(BAUD_TIME);
  }
  return;
}


/***************************************** sendOne() ****************************************************
sendOne(): transmits a "one" symbol that is compatible with EV1527 and PT2262 data
    arguments:  none
    returns: nothing
*********************************************************************************************************/
void sendOne()
{
  // a one is represented by three baud high and one baud low
  for(int i = 0; i < 3; i++)
  {
      digitalWrite(TX_PIN, HIGH);
      delayMicroseconds(BAUD_TIME);
  }
  digitalWrite(TX_PIN, LOW);
  delayMicroseconds(BAUD_TIME);
  return;
}

/***************************************** sendSync() ***************************************************
sendSync(): transmits a "sync" symbol that is compatible with EV1527 and PT2262 data
    arguments:  none
    returns: nothing
*********************************************************************************************************/
void sendSync()
{
  // a sync is represented by one baud high and 31 baud low
  digitalWrite(TX_PIN, HIGH);
  delayMicroseconds(BAUD_TIME);
  for(int i = 0; i < 31; i++)
  {
      digitalWrite(TX_PIN, LOW);
      delayMicroseconds(BAUD_TIME);
  }
  return;
}
