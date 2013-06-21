//  -------------------------------------------------------------------------
//  Parking sensor that shows the distance from the car to the wall
//  It is intended for permanent installement in the garage rather than
//  sensors in the car (our cars have only back parking sensors)
//  Project is using the Maxor ultrasonic sensor:
//      LV-MaxSonar EZ1 MB1010
//  Accuracy and stability of the measurement is not very clear
//  Target platform is the Fio Arduino based ARM board
//  -------------------------------------------------------------------------
//  You may use, borrow or modify source code below. Crediting the author
//  is expected and seen as a matter of personal integrity
//  -------------------------------------------------------------------------
//  By: Zakie Mashiah, April 2011 zmashiah@gmail.com
//  -------------------------------------------------------------------------
//	Revision History
//		April 2011		Initial code
//		November 2011	Add running LEDs to conserve power, on-board LED off
//		February 2012	Remove running LEDs, change phisical case
//  -------------------------------------------------------------------------
//	ToDo
//		Remove Serial output to reduce time awake, move to baudrate of 115,200
//		reduced time substantially
//		Make configuration as buttons options
//		Allow user to calibrate distances
//  -------------------------------------------------------------------------
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <stdio.h>

// Macros used to set and clear bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

/* Wiring
 * ------
 *		sensorPin		The analog pin to which sensor is connected to
 *		Leds			The different LEDs (digital output ports)
 * Parameters/Configuration
 * ------------------------
 *		MoOnBoardLED		If true, onbard LED will not light, saving power
 *		runningLeds			If true, only one LED will light at a time, saving power
 *		milisToBlankDisplay	If reading remains the same for that long, display will be turned off and system
 *							will be put to sleep.
 *		L2, L3, L4			The different analog input levels to turn LEDs (thresholds for each) in function
 *							ReadDistanceSensor
 *		wdtTime				ARM Watchdog code for the sleep time
 *		NUM_LEDS			Number of LEDs, changing macro is not enough though...
 */
volatile boolean f_wdt	= 0;	// For the watchdog

const int ledOnBoard	= 13;	// The pin which represent also on-board LED
const int sensorPin		= A3;	// Analog sensor is connected here

// Following two variables are being used across the two threads.
static byte displayLedLevels = 0;   // This variable holds the representation of the LEDs to display.
                                    // Each bit represent an LED, 1=On, 0=Off.
                                    // It is being used across two threads, the one that samples the distance sensor
                                    // and the other thread that displays the result
static bool displayBlanked = false; // If the level remains constant for some time, it means the car is parking 
									// or there is a constant object the sensor is seeing. No reason to keep the
									// LEDs working in this case
#define NUM_LEDS	3
static int Leds[NUM_LEDS] = { 8 /*Blue*/, 7 /*Green*/, 5 /*Red*/ };

static bool onoffOnBoard = false;		// Represents the on-board LED state to blink on cycles
static bool NoOnBoardLED = true;		// If true, on-board LED will not be used
static byte lastDisplayLevel	= 0;	// value of last reading


// Some constants for timinig and our sleep period
const unsigned long milisToBlankDisplay = 1000 * 7; // 5 seconds of constant display will cause blanking
const int shortCycle 					= 200;		// Sampling every 200mSec
// Time variables
static unsigned long dtNow, dtChanged;	// Time now and time of last changed reading
int ourSleep 		= 0;				// How much to actually sleep this round
										// will either be 0 or shortCycle
const int wdtTime	= 6;				// ARM Watchdog sleep time code

//****************************************************************  
// Functions borrowed from the net for putting Arduino into sleep
// Sleep and Watchdog Functions
//****************************************************************  
// set system into the sleep state 
// system wakes up when wtchdog is timed out
void system_sleep()
{
    static unsigned long lastSleepMillis = 0;
    
    Serial.print("--> system_sleep. ");
    if (lastSleepMillis != 0)
    {
	unsigned long l = millis();
		
	Serial.print(" Awake: ");
	Serial.println(l-lastSleepMillis, DEC);
	Serial.print("ms\n");
	lastSleepMillis = l;
    }
    else
	lastSleepMillis = millis();
	
    delay(20);  // Ensure the text is really sent to terminal
	
    // Initialize watchdog and sleep
    cbi( SMCR,SE );				// sleep enable, power down mode
    cbi( SMCR,SM0 );			// power down mode
    sbi( SMCR,SM1 );			// power down mode
    cbi( SMCR,SM2 );			// power down mode
    setup_watchdog(wdtTime);	// Every second. Tried 2 seconds and was not good experience for users
    cbi(ADCSRA,ADEN);			// switch Analog to Digitalconverter OFF
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
    sleep_enable();
    sleep_mode();		// System sleeps here

    sleep_disable();	// If we had interrupts, System would continue execution here
    sbi(ADCSRA,ADEN);	// switch Analog to Digitalconverter ON
}

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii)
{
    byte bb;
    int ww;

    if (ii > 9 ) ii = 9;
    bb = ii & 7;
    if (ii > 7) bb |= (1<<5);
    bb |= (1<<WDCE);
    ww=bb;

    MCUSR &= ~(1<<WDRF);
    // start timed sequence
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    // set new watchdog timeout value
    WDTCSR = bb;
    WDTCSR |= _BV(WDIE);
}

// Watchdog Interrupt Service / is executed when  watchdog timed out
ISR(WDT_vect)
{
	f_wdt = 1;  // set global flag
}
//*******************************************************************

//****************************************************************
// Application Functions
//****************************************************************
/// <summary>
/// DisplayFunc
///      Uses the displayLevels and displayBlaned variables to turn
///      I/O ports on for lightening up the relevant LEDs
/// </summary>
static void DisplayFunc()
{
	byte b;
	int i;
	byte r;
	
	if (displayBlanked) // If display should be blanked, turn off LEDS
	{
		for (i = 0; i < NUM_LEDS; i++)
			digitalWrite(Leds[i], false);
	}
	else
	{
		// Run through the LEDs and turn them on as needed
		b = 0x01;
		for (i = 0; i < NUM_LEDS; i++, b <<= 1)  // Walk through the bits
		{
			r = (byte)(displayLedLevels & b);  // Is the i'th bit on?
			if (r != 0)
			{
				digitalWrite(Leds[i], true);	// Turn the relevant led on
			}
			else
				digitalWrite(Leds[i], false);	// Turn it off
		}
	}
}

/// <summary>
///	SetOnBoardLED
///		Set the on-board LED to new state and display it.
///		If NoOnBoardLED global variable is true, it will LED will be set off
/// </summary>
void SetOnBoardLED(bool state)
{
    if (NoOnBoardLED) return;
    
    onoffOnBoard = state;
    digitalWrite(ledOnBoard, onoffOnBoard);
}

/// <summary>
/// PutSysyemToSleep
///     Do all needed to put the system into sleep
/// </summary>
void PutSystemToSleep()
{
    SetOnBoardLED(false);
    SetupPorts(false);  // Put ports to input to save energy
    system_sleep();     // Actual sleep
    SetupPorts(true);   // Restore them to what they were
    SetOnBoardLED(true);
}

/// <summary>
/// Reads value from the sensor and sets the displayLedLevels variable
/// Thresholds for when to light up each LED is set in this function
/// </summary>
static void ReadDistanceSensor()
{
	int data;
	// constants for different levels of the sensor
	const int Lwhitenoise = 170; // Anything further than this is not interesting
	const int L2 = 120;          // Lower values than this will turn on 2nd LED
	const int L3 = 80;           // Lower values than this Will turn on 3rd LED
	const int L4 = 25;           // This is the closest one, and will cause MSB LED to light.
	//
	//  Leds output based on sensor level:
	//
	//             | Level1   | Level2  | Level3| Level4
	//  -----------|----------|---------|-------|----------> sensor value
	//             |          |         |       |
	//             /          /         /       /
	//  whitenoise           L2        L3     L4
	//
	// ======================================================================
	
	// Read data from sensor
	data = analogRead(sensorPin);
	// According to above thresholds, set the displayLevels variables for bit-wise LEDs on/off
	if (data > Lwhitenoise)
		displayLedLevels = 0;  // Ignore anything too distant. This is pure noise for this application
	else if (data < L4)
		displayLedLevels = B100;  // Red is set to ON		==> Red
	else if (data < L3)
		displayLedLevels = B110; // Red and Green set to ON	==> Yellow
	else if (data < L2)
		displayLedLevels = B011; // Blue and Green set to ON	==> Cyan
	else
		displayLedLevels = B010; // Green is set to ON		==> Green

	// one day I will fix the Serial.print to have printf style....
        char buff[30];
        sprintf(buff, "S=%d\tLED=", data);
        Serial.print(buff);
	//Serial.print("S="); Serial.print(data, DEC);
	//Serial.print("\tLED=");
        Serial.println(displayLedLevels, BIN);
}


void SetupPorts(bool oper)
{
  byte i;
  byte io;

  io = (oper == true) ? OUTPUT : INPUT;

  // Setup the LEDs as output
  for(i=0; i<NUM_LEDS; i++)
    pinMode(Leds[i], io);

  // Setup the on-board LED
  if (NoOnBoardLED == false)
    pinMode(ledOnBoard, io);
}
//****************************************************************

//****************************************************************
// Arduino typical functions
//****************************************************************
/// <summary>
/// Setup
///     Initializes the system
/// </summary>
void setup()
{
	Serial.begin(115200);		// Good to have it fast, it saves wake time for Arduino thus preserving battery life
        pinMode(ledOnBoard, OUTPUT);
        digitalWrite(ledOnBoard, 0);
	SetupPorts(true);
	// Initialize now and changed times
	dtNow = millis();
	dtChanged = millis();
	Serial.print("Starting...\n");
}



/// <summary>
/// loop
///     This the function the Arduino will execute all the time.
///     Read data from sensor, checks if something changed and if
///     nothing new goes to sleep
///     Otherwise, if something changes turn on the display
/// </summary>
void loop()
{

    if (f_wdt == 1)
    {
        // If we got here, it means the watchdog worked. Reset the flag and that's it
        wdt_disable();      // Fist, disable watchdog, if we will need it later, we will turn it on
        f_wdt = 0;          // Clear the flag
        Serial.print("<--- WD wake up\n");  // and tell the world about it
    }

    // Read what the distance sensor has now
    ReadDistanceSensor();

    // Check to see if same reading for long time. if so, blank the LEDs to save energy
    // Since we are using the display state rather than actual reading, we eliminate noise
    // of multiple reading. Worse case is some LEDs will be blinking if distance is boarder
    // line with threshold. Not a big deal and did not really happen in real life too much
    if (displayLedLevels != lastDisplayLevel)
    {
        // Store new reading and ensure display is not blanked
        lastDisplayLevel = displayLedLevels;
        dtChanged = millis();

        displayBlanked = false; // If changes happeing, display should not be blanked
        ourSleep = shortCycle;  // We will not sleep for long time
    }
    else  // same reading as before
    {
        if (displayBlanked) // we are already in blank mode
        {
            ourSleep = 0; // If nothing changes system should go to sleep
        }
        else
        {
			ourSleep = shortCycle;
			dtNow = millis();
			if (dtNow > dtChanged) // unless overlap, that should be true
			{
				// Substracting only meaningful if milis did not overlap
				unsigned long ts = dtNow - dtChanged;
				if (ts > milisToBlankDisplay)  // Now - lastChanged is more than 5 seconds?
				{
					displayBlanked = true;           // mark that we are in blanked state
					Serial.println("Display will be blanked");
					// The end of the loop will 'display' the LEDs as off, and on next
					// itteration of the loop we will put the system to sleep
				}
			}
            else
            {
				// If we are here it means milis overlapped
                Serial.println("Time overlapped, we are alive more than 50 days :-)");
                dtChanged = dtNow; // so what? it will take a bit longer to sleep
            }
        }
    }
    onoffOnBoard ^= true;             // Toggle the on board LED state
	SetOnBoardLED(onoffOnBoard);
    DisplayFunc();                   // Show all the LEDs representing distance
    if (ourSleep == shortCycle)
        delay(ourSleep);                 // Sleep short time
    else
        PutSystemToSleep();              // CPU into sleep mode with 2seconds watchdog
}
