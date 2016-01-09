//#define DEBUG                   // turns on Serial port messages
#define DEBUG_LED                 // enables LED toggling in toggleD7LED()
#define D7LED_DELAY 200           // how long to wait when toggling LED for debugging
#define photon044                 // when present, enables functions that only work with 0.4.4 or higher
//
/***************************************************************************************************/
// ReceiveTest.ino
//
// This software is used to gather information about 315 and 433Mhz wireless sensors. The software
// will gather information about the enviroment into a circular buffer. The buffer can then be
// read out by a web site using the particle.io restful API.
//
// This software was based originally on the SIS firmware.
//
//  Use of this software is subject to the Terms of Use, which can be found at:
//  https://github.com/TeamPracticalProjects/SISProject/blob/master/SISDocs/Terms_of_Use_License_and_Disclaimer.pdf
//
//  This software uses code extracted from the Arduino RC-Switch library:
//    https://github.com/sui77/rc-switch
//
// Portions of this software that have been extracted from RC-Switch are subject to the RC-Switch
// license, as well as the the SIS Terms of Use.
//
//  Version 1.0     1/8/2016.
const String VERSION = "1.0";   	// current firmware version
//
//  (c) 2015 by Bob Glicksman and Jim Schrempp
/***************************************************************************************************/

/************************************* Global Constants ****************************************************/

const int INTERRUPT_315 = 3;   // the 315 MHz receiver is attached to interrupt 3, which is D3 on an Spark
const int INTERRUPT_433 = 4;   // the 433 MHz receiver is attached to interrupt 4, which is D4 on an Spark
const int WINDOW = 200;    	// timing window (+/- microseconds) within which to accept a bit as a valid code
const int TOLERANCE = 60;  	// % timing tolerance on HIGH or LOW values for codes

const int BUF_LEN = 100;     	// circular buffer size.

const int MAX_SUBSTRINGS = 6;   // the largest number of comma delimited substrings in a command string
const byte NUM_BLINKS = 1;  	// number of times to blink the D7 LED when a sensor trip is received
const unsigned long BLINK_TIME = 200L; // number of milliseconds to turn D7 LED on and off for a blink

const String DOUBLEQ = "\"";

/************************************* Global Variables ****************************************************/
volatile boolean codeAvailable = false;  // set to true when a valid code is received and confirmed
volatile unsigned long receivedSensorCode; // decoded 24 bit value for a received and confirmed code from a wireless sensor
const int MAX_CODE_TIMES = 52;
volatile unsigned int codeTimes315[MAX_CODE_TIMES];  // array to hold times (microseconds) between interrupts from transitions in the received data
                         	//  times[0] holds the value of the sync interval (LOW).  HIGH and LOW times for codes are
                         	//  stored in times[1] - times[49] (HIGH and LOW pairs for 24 bits).  A few extra elements
                         	//  are provided for overflow.
volatile unsigned int codeTimes433[MAX_CODE_TIMES];  // array to hold times (microseconds) between interrupts from transitions in the received data
                         	//  times[0] holds the value of the sync interval (LOW).  HIGH and LOW times for codes are
                         	//  stored in times[1] - times[49] (HIGH and LOW pairs for 24 bits).  A few extra elements
                         	//  are provided for overflow.
volatile unsigned int *codeTimes;  // pointer to 315 MHz or 433 MHz codeTimes array based upon the interrupt.
time_t resetTime;       	// variable to hold the time of last reset

unsigned long upcount = 0L; // sequence number added to the circular buffer log entries
unsigned long validCodeCount = 0L;  // number of codes received since circularBuffReset

	// array to hold parsed substrings from a command string
String g_dest[MAX_SUBSTRINGS];

	// Strings to publish data to the cloud
String g_bufferReadout = String("");
char cloudMsg[80];  	// buffer to hold last sensor tripped message
char cloudBuf[90];  	// buffer to hold message read out from circular buffer

String cBuf[BUF_LEN];   // circular buffer to store events and messages as they happen
                        // Expected format of the string stored in cBuf is:
                        // TYPE,SEQUENCENUMBER,INDEX,EPOCTIME
                        // where
                        //    TYPE is A (advisory) or S (sensor)
                        //    SEQUENCENUMBER is a monotonically increasing global (eventNumber)
                        //    INDEX is into sensorName[] for type sensor
                        //          or into messages[] for type advisory
                        //    EPOCTIME is when the entry happened
                        // see cBufInsert(), cBufRead(), readFromBuffer(), logSensor(), logMessage()

int head = 0;       	// index of the head of the circular buffer
int tail = 0;       	// index of the tail of the buffer

char config[120];    	// buffer to hold local configuration information
long eventNumber = 0;   // grows monotonically to make each event unique



/**************************************** setup() ***********************************************/
void setup()
{

    // Use D7 LED as a test indicator.  Light it for the time spent in setup()
    pinMode(D7, OUTPUT);
    digitalWrite(D7, HIGH);


#ifdef DEBUG
    Serial.begin(9600);
#endif

    pinMode(INTERRUPT_315, INPUT);
    pinMode(INTERRUPT_433, INPUT);

    attachInterrupt(INTERRUPT_315, isr315, CHANGE);   // 315 MHz receiver on interrupt 3 => that is pin #D3
    attachInterrupt(INTERRUPT_433, isr433, CHANGE);   // 433 MHz receiver on interrupt 4 => that is pin #D4

    toggleD7LED();

    // wait for the Core to synchronise time with the Internet
    while(Time.year() <= 1970 && millis() < 30000)
    {
        delay(100);
        Spark.process();
    }

    if (Time.year() <= 1970)
    {
        reportFatalError(3);
        //never returns from here
    }

    toggleD7LED();

    // Publish local configuration information in config[]
    resetTime = Time.now();    	// the current time = time of last reset

    // Exposed to the cloud

    Spark.function("cBuffReset", circularBuffReset); // reset the circular buffer

    Spark.function("ReadBuffer", readBuffer); // put next log entry into circularBuff
    Spark.variable("circularBuff", cloudBuf, STRING);

    Spark.variable("codeCount", cloudMsg, STRING);





    toggleD7LED();

#ifdef DEBUG
    Serial.println("End of setup()");
#endif

    // turn off the D7 LED at the end of setup()
    digitalWrite(D7, LOW);
}

/************************************ end of setup() *******************************************/

/**************************************** loop() ***********************************************/
void loop()
{

    static unsigned long lastTimeSync = millis();  // to resync time with the cloud daily
    static boolean blinkReady = true;  // to know when non-blocking blink is ready to be triggered

    // Test for received code from a wireless sensor
    if (codeAvailable) // new wireless sensor code received
    {

        validCodeCount++;
        sprintf(cloudMsg, "%d", validCodeCount);

        logSensor(receivedSensorCode);

       	// code to blink the D7 LED when a sensor trip is detected
    	if(blinkReady)
        {
            blinkReady = nbBlink(NUM_BLINKS, BLINK_TIME);
        }

	    codeAvailable = false;  // reset the code available flag if it was set
    }

    if(!blinkReady)  // keep the non blocking blink going
    {
    	blinkReady = nbBlink(NUM_BLINKS, BLINK_TIME);
	}

}
/************************************ end of loop() ********************************************/

/************************************ toggleD7LED() ********************************************/
// toggleD7LED(): D7 LED is used as a test indicator. This function allows you to toggle it.
// Only actually toggle if DEBUG_LED is defined
//
void toggleD7LED(void)
{
    #ifdef DEBUG_LED
    digitalWrite(D7, LOW);
    delay(D7LED_DELAY);
    digitalWrite(D7, HIGH);
    delay(D7LED_DELAY);
    #endif
}

/************************************ end toggleD7LED loop() ***********************************/

/************************************* logSensor() *********************************************/
// logSensor(): function to create a log entry for a sensor trip
//
// Arguments:
//  sensorIndex:  index into the sensorName[] and activeCode[] arrays of sensor data
//
void logSensor(int sensorCode)
{
    // create timestamp substring
    static unsigned long lastLogTime = 0;

    unsigned long now = micros();
    unsigned long timeDelta = now - lastLogTime;
    lastLogTime = now;

	String timeStamp = "-";
    if (timeDelta < 1000000) {   // Only show delta time if it might be between code receipts
        timeStamp = timeDelta;
    }

	// create sequence number substring
	String sequence = "";
	sequence += upcount;
    if (upcount < 10){
        sequence = "0" + sequence;
    }

    upcount++;
	if (upcount > 9999)  // limit to 4 digits
	{
    	upcount = 0;
	}

	// create the log entry
	String logEntry = "S:"; // Sensor trip log entry
	logEntry += sequence;
	logEntry += ", Code:";
	logEntry += sensorCode;
	logEntry += ", DeltaMicros:";
	logEntry += timeStamp;

	// pad out to 20 characters
	while (logEntry.length() < 22)
	{
    	logEntry += "x";
	}

	cBufInsert("" + logEntry);

	return;
}

/*********************************** end of logSensor() ****************************************/

/*************************************** parser() **********************************************/
// parser(): parse a comma delimited string into its individual substrings.
//  Arguments:
//  	source:  String object to parse
//  Return: the number of substrings found and parsed out
//
//  This functions uses the following global constants and variables:
//  	const int MAX_SUBSTRINGS -- nominal value is 6
//  	String g_dest[MAX_SUBSTRINGS] -- array of Strings to hold the parsed out substrings

int parser(String source)
{
	int lastComma = 0;
	int presentComma = 0;

	//parse the string argument until there are no more substrings or until MAX_SUBSTRINGS
	//  are parsed out
	int index = 0;
	do
	{
    	presentComma = source.indexOf(',', lastComma);
    	if(presentComma == -1)
    	{
        	presentComma = source.length();
    	}
        g_dest[index++] = "" + source.substring(lastComma, presentComma);
    	lastComma = presentComma + 1;

	} while( (lastComma < source.length() ) && (index < MAX_SUBSTRINGS) );

	return (index);
}
/************************************ end of parser() ********************************************/

/************************************* circularBuffReset *****************************************/
// CALLED FROM THE CLOUD
// Called to reset the circular buffer of events to empty
int circularBuffReset(String data)
{
    tail = 0;
    head = tail;
    validCodeCount = 0;
    return 0;

}

/************************************* readBuffer() ********************************************/
// readBuffer(): read the contents of the circular buffer into the global variable "cloudBuf"
//  Arguments:
//  	String location:  numerical location of the buffer data to read.  The location is relative
//      	to the latest entry, which is "0".  The next to latest is "1", etc. back to BUF_LEN -1.
//       	BUF_LEN can be determined from the cloud variable "bufferSize".  If location exceeds
//       	BUF_LEN - 1, the value that is read out is the oldest value in the buffer, and
//       	-1 is returned by the function.  Otherwise, the value is determined by location
//       	and 0 is returned by this function.
//  Return:  0 if a valid location was specified, otherwise, -1.
//  EXPOSED TO CLOUD
int readBuffer(String location)
{
    int offset;
    int result;

    offset = location.toInt();
    result = readFromBuffer(offset, cloudBuf);
    if (result == -1) {
        String qmark  = "?";
        qmark.toCharArray(cloudBuf, qmark.length() + 1 );
        cloudBuf[qmark.length() + 2] = '\0';
    }
    return result;
}

/*********************************end of readBuffer() *****************************************/

/********************************** readFromBuffer() ******************************************/
// readFromBuffer(): utility fujction to read from the circular buffer into the
//  character array passed in as stringPtr[].
//  Arguments:
//      int offset: the offset into the circular buffer to read from. 0 is the latest entry.  The
//          next to latest entry is 1, etc. back to BUF_LEN -1.
//       	BUF_LEN can be determined from the cloud variable "bufferSize".  If location exceeds
//       	BUF_LEN - 1, the value that is read out is the oldest value in the buffer, and
//       	-1 is returned by the function.  Otherwise, the value is determined by location
//       	and 0 is returned by this function.
//      char stringPtr[]: pointer to the string that will be returned from this
//        function. The format of the string expected by the web site is one of:
//            (S:nnn) SENSORNAME tripped at DATETIME Z (epoc:EPOCTIME)
//            (S:nnn) SENSORNUMBER detected at DATETIME Z (epoc:EPOCTIME)
//
//  Return:  0 if a valid location was specified, otherwise, -1.

int readFromBuffer(int offset, char stringPtr[])
{
	int result;     	// the result code to return

	// check and fix, if necessary, the offset into the circular buffer
	if(offset >= BUF_LEN)
	{
    	offset = BUF_LEN - 1;   // the maximum offset possible
    	result = -1;        	// return the error code
	}
	else
	{
    	result = 0;         	// return no error code
	}


	// now retrieve the data requested from the circular buffer and place the result string
    // in g_bufferReadout
	g_bufferReadout = "" + cBufRead(offset);

#ifdef DEBUG
    Serial.println(g_bufferReadout);
#endif

	// create the readout string for the cloud from the buffer data
	if(g_bufferReadout  == "")  // skip empty log entries
	{
        g_bufferReadout = "?";
	}

    g_bufferReadout.toCharArray(stringPtr, g_bufferReadout.length() + 1 );
	stringPtr[g_bufferReadout.length() + 2] = '\0';

	return result;

}

/********************************** end of readFromBuffer() ****************************************/

/******************************************** cBufInsert() *****************************************/
// cBufInsert():  insert a string into the circular buffer at the current tail position.
//  Arguments:
//	String data:  the string data (string object) to store in the circular buffer
//	return:  none.

void cBufInsert(String data)
{
    static boolean fullBuf = false;	// false for the first pass (empty buffer locations)

    cBuf[tail] = data;	// write the data at the end of the buffer

    //  adjust the tail pointer to the next location in the buffer
    tail++;
    if(tail >= BUF_LEN)
    {
        tail = 0;
        fullBuf = true;
    }

    //  the first time through the buffer, the head pointer stays put, but after the
    //	buffer wraps around, the head of the buffer is the tail pointer position
    if(fullBuf)
    {
        head = tail;
    }

}
/***************************************** end of cBufInsert() **************************************/

/********************************************* cBufRead() *******************************************/
// cBufRead():  read back a String object from the "offset" location in the cirular buffer.  The
//	offset location of zero is the latest value in (tail of) the circular buffer.
//  Arguments:
//	int offset:  the offset into the buffer where zero is the most recent entry in the circular buffer
//       and 1 is the next most recent, etc.
//  Return:  the String at the offset location in the circular buffer.

String cBufRead(int offset)
{
    int locationInBuffer;

    locationInBuffer = tail -1 - offset;
    if(locationInBuffer < 0)
    {
        locationInBuffer += BUF_LEN;
    }

    return cBuf[locationInBuffer];

}
/****************************************** end of cBufRead() ***************************************/

/************************************** isr315() ***********************************************/
//This is the interrupt service routine for interrupt 3 (315 MHz receiver)
void isr315()
{
    codeTimes = codeTimes315;	// set pointer to 315 MHz array
    process315();
    return;
}

/***********************************end of isr315() ********************************************/

/************************************** isr433() ***********************************************/
//This is the interrupt service routine for interrupt 4 (433 MHz receiver)
void isr433()
{
    codeTimes = codeTimes433;	// set pointer to 433 MHz array
    process433();
    return;
}

/***********************************end of isr433() ********************************************/

/************************************* process315() ***********************************************/
//This is the code to process and store timing data for interrupt 3 (315 MHz)
//This is identical to the process433 routine

void process315()
{
    //this is right out of RC-SWITCH
    static unsigned int duration;
    static unsigned int changeCount;
    static unsigned long lastTime = 0L;
    static unsigned int repeatCount = 0;

    long time = micros();
    duration = time - lastTime;

    if (duration > 5000
        && duration > codeTimes[0] - 200
        && duration < codeTimes[0] + 200)
    {
        // we found a second sync
        repeatCount++;
        changeCount--;

	    if (repeatCount == 2)  // two successive code words found
	    {
            decode(changeCount); // decode the protocol from the codeTimes array
            repeatCount = 0;
        }
        changeCount = 0; // reset so we're ready to start a new sequence
    }
    else if (duration > 5000)
    {
        // If the duration is this long, then it could be a sync
        changeCount = 0;
    }

    if (changeCount >= MAX_CODE_TIMES) // too many bits before sync
    {
        // reset, we just had a blast of noise
        changeCount = 0;
        repeatCount = 0;
    }

    codeTimes[changeCount++] = duration;
    lastTime = time;

    return;
}
/***********************************end of process315() ********************************************/

/************************************** process433() ***********************************************/
//This is the code to process and store timing data for interrupt 4 (433 MHz)

void process433()
{
    //this is right out of RC-SWITCH
    static unsigned int duration;
    static unsigned int changeCount;
    static unsigned long lastTime = 0L;
    static unsigned int repeatCount = 0;

    long time = micros();
    duration = time - lastTime;

/*
    A pulse for a bit is between 300 and 500 microseconds. A bit always contains either
    three high or three low pulse intervals in a row. So if we have a duration that is
    longer than 1500, it is not part of the bit stream.
    When we are just processing noise, then codeTimes[0] could be anything.
    If this interrupt is:
        - the first rise of the pulse that starts the very first sync of
          the first code transmission, then duration
          could be anything. It will be stored in codeTimes at the current
          index position since we don't know it is special.
        - the fall of the first pulse that starts a sync, then duration
          will be one pulse. It will be stored in codeTimes at the current
          index position.
        - the rise at the end of the sync, then duration will be over 5000 microseconds
          and changeCount will be set to 0. The duration of this sync low time
          will be stored in changeCount[0]. The duration is 31 pulse times long.
        - the fall of the first part of a bit sequence then duration will be either
          one pulse if a 0 or 3 pulses which is the start of a 1.
        - if this is a valid code sequence, then as soon as this code transmission
          is over, a new sync and sequence will start. The new sequence will start
          with a pulse high and then 31 pulses low. This will trigger us that the
          previous sequence of interrupts was a valid code sequence. To trigger
          us this new sync low time must be longer than 5000 (of course) and be within
          +/- 200 microseconds of the sync low that we saw previously. If these
          conditions are met, then we bump repeatCount because we have now seen
          two valid sync low times within 52 changes.
        - when this all happens successfully a second time (we get a third sync
          low that is +/- 200 microseconds of the first one we saw) then we
          call the decoder for it to decide if the sequence of interrupt times
          is a valid code sequence.

    Q: Why don't we calculate codeTimes[0]/31 and check that a new duration is
       at that value +/- some tolerance? That would allow us to ignore noise.
 */
    if (duration > 5000
        && duration > codeTimes[0] - 200
        && duration < codeTimes[0] + 200)
    {
        // we found a second sync
        repeatCount++;
        changeCount--;

	    if (repeatCount == 2)  // two successive code words found
	    {
            decode(changeCount); // decode the protocol from the codeTimes array
            repeatCount = 0;
        }
        changeCount = 0; // reset so we're ready to start a new sequence
    }
    else if (duration > 5000)
    {
        // If the duration is this long, then it could be a sync
        changeCount = 0;
    }

    if (changeCount >= MAX_CODE_TIMES) // too many bits before sync
    {
        // reset, we just had a blast of noise
        changeCount = 0;
        repeatCount = 0;
    }

    codeTimes[changeCount++] = duration;
    lastTime = time;

    return;
}
/***********************************end of isr433() ********************************************/

/*************************************** decode() ************************************************/
// decode():  Function to decode data in the appropriate codeTimes array for the data that was
//  just processed.  This function supports PT2262 and EV1527 codes -- 24 bits of data where
//  0 = 3 low and 1 high, 1 = 3 high and 1 low.
// Arguments:
//  changeCount: the number of timings recorded in the codeTimes[] buffer.

void decode(unsigned int changeCount)
{

    digitalWrite(D6, HIGH);
    unsigned long code = 0L;
    unsigned long pulseTime;
    float pulseTimeThree;
    unsigned long pulseTolerance;

    pulseTime = codeTimes[0] / 31L;
    pulseTimeThree = pulseTime * 3;

    pulseTolerance = pulseTime * TOLERANCE * 0.01;

    for (int i = 1; i < changeCount ; i=i+2)
    {

	    if (codeTimes[i] > pulseTime - pulseTolerance
            && codeTimes[i] < pulseTime + pulseTolerance
            && codeTimes[i+1] > pulseTimeThree - pulseTolerance
            && codeTimes[i+1] < pulseTimeThree + pulseTolerance)
	    {
            // we have a 0 shift left one
            code = code << 1;

	    }
        else if (codeTimes[i] > pulseTimeThree - pulseTolerance
                && codeTimes[i] < pulseTimeThree + pulseTolerance
                && codeTimes[i+1] > pulseTime - pulseTolerance
                && codeTimes[i+1] < pulseTime + pulseTolerance)
  	          {
                  // we have a 1, add one to code
                  code = code + 1;
                  // shift left one
                  code = code << 1;
  	           }
               else
  	            {
                    // Failed, this sequence of interrupts did not indicate a 1 or 0
                    // so abort the decoding process.
                    i = changeCount;
                    code = 0;
                }
    }
    // in decoding we shift one too many, so shift right one
    code = code >> 1;

    if (changeCount > 6) // ignore < 4bit values as there are no devices sending 4bit values => noise
    {
        receivedSensorCode = code;
        if (code == 0)
        {
            codeAvailable = false;
        }
        else
        {
            codeAvailable = true;
        }

    }
    else	// too short -- noise
    {
        codeAvailable = false;
        receivedSensorCode = 0L;
    }

    digitalWrite(D6, LOW);
    return;
}

/************************************ end of decode() ********************************************/

/************************************** nbBlink() ************************************************/
// nbBlink():  Blink the D7 LED without blocking.  Note: setup() must declare
//          	pinMode(D7, OUTPUT) in order for this function to work
//  Arguments:
//  	numBlinks:  the number of blinks for this function to implement
//  	blinkTime:  the time, in milliseconds, for the LED to be on or off
//
//  Return:  true if function is ready to be triggered again, otherwise false
//

boolean nbBlink(byte numBlinks, unsigned long blinkTime)
{
	const byte READY = 0;
	const byte LED_ON = 1;
	const byte LED_OFF = 2;
	const byte NUM_BLINKS = 3;


	static byte state = READY;
	static unsigned long lastTime;
	static unsigned long newTime;
	static byte blinks;

	newTime = millis();

	switch(state)
	{
        case(READY):
        	digitalWrite(D7, HIGH); 	// turn the LED on
        	state = LED_ON;
        	lastTime = newTime;
        	blinks = numBlinks;
        	break;

    	case(LED_ON):
        	if( (newTime - lastTime) >= blinkTime) // time has expired
        	{
            	state = LED_OFF;
            	lastTime = newTime;
        	}
        	break;

    	case(LED_OFF):
        	digitalWrite(D7, LOW);  	// turn the LED off
        	if( (newTime - lastTime) >= blinkTime)
        	{
            	if(--blinks > 0) 	// another blink set is needed
            	{
                	digitalWrite(D7, HIGH);
                	state = LED_ON;
                	lastTime = newTime;
            	}
            	else
            	{
                	state = READY;
            	}

        	}
        	break;

    	default:
        	digitalWrite(D7, LOW);
        	state = READY;

	}

	if(state == READY)
	{
    	return true;
	}
	else
	{
    	return false;
	}
}

/*********************************** end of nbBlink() ********************************************/

/************************************ publishEvent() *********************************************/
// publishEvent():  Function to pubish a Core event to the cloud in JSON format
//  Arguments:
//  	data: the message to publish to the cloud
//

int publishTestE(String data)
{
    eventNumber++;

    // Make it JSON ex: {"eventNum":"1","eventData":"data"}
    String msg = "{";
    msg += makeNameValuePair("eventNum",String(eventNumber));
    msg += ",";
    msg += makeNameValuePair("eventData", data);
    msg += "}";
    sparkPublish("SISEvent", msg , 60);
    return 0;

}

// Keeping a separate publishEvent because we might want to send more than one
// data field.
int publishEvent(String sensorIndex)
{
    eventNumber++;

    // Make it JSON ex: {"eventNum":"1","eventData":"data"}
    String msg = "{";
    msg += makeNameValuePair("eventNum",String(eventNumber));
    msg += ",";
    msg += makeNameValuePair("sensorLocation",sensorIndex);
    msg += "}";

    sparkPublish("SISEvent", msg, 60);

}

int sparkPublish (String eventName, String msg, int ttl)
{
    bool success = true;

	if (millis() > 5000 )  // don't publish until spark has a chance to settle down
	{
#ifdef photon044
        success = Spark.publish(eventName, msg, ttl, PRIVATE);
#endif

#ifndef photon044
        //  A return code from spark.publish is only supported on photo 0.4.4 and later
        Spark.publish(eventName, msg, ttl, PRIVATE);
#endif
    }


#ifdef DEBUG
    Serial.println("sparkPublish called");

    if (success == false)
    {
        String message = "Spark.publish failed";
        Serial.print(message);

        message = " trying to publish " + eventName + ": " + msg;

        Serial.println(message);
        Spark.process();
    }

#endif

    return success;

}


String makeNameValuePair(String name, String value)
{
	String nameValuePair = "";
	nameValuePair += DOUBLEQ + name + DOUBLEQ;
	nameValuePair += ":";
	nameValuePair += DOUBLEQ + value + DOUBLEQ;
	return nameValuePair;
}

/********************************* end of publishEvent() *******************************/

/****************************** fatal error code reporting *****************************/
// Call this to flash D7 continuously. This routine never exits.
//
// Error codes
//    3 flashes:  Failed to sync time to internet. Action: power cycle
//
void reportFatalError(int errorNum)
{

#ifdef DEBUG
    String message = "unknown error code";
    Serial.print("Fatal Error: ");
    switch(errorNum)
    {
        case 3:
            message = " could not sync time to internet";
            break;
        default:
            break;
    }
    Serial.println(message);
    Spark.process();
#endif

    while(true)  // never ending loop
    {
        for (int i=0; i < errorNum; i++)
        {
            digitalWrite(D7, HIGH);
            delay(100);
            Spark.process();
            digitalWrite(D7, LOW);
            delay(100);
            Spark.process();
        }
        digitalWrite(D7, LOW);

        // Now LED off for 1500 milliseconds
        for (int i=0; i < 3; i++)
        {
            delay(500);
            Spark.process();
        }
    }

    // we will never get here.
    return;

}

/***************************** end of fatal error code reporting ****************************/
