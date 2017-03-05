#include <Joystick.h>
// Joystick library taken from https://github.com/MHeironimus/ArduinoJoystickLibrary

// To get multiple joysticks working in Ubuntu, one needs to first remove the usbhid module:
// sudo rmmod usbhid
// Then, we add the module back, with a quirk telling it that our joystick is in fact multiple joysticks
// For an Arduino Leonardo, the command
// sudo modprobe usbhid quirks=0x2341:0x8036:0x40
// The general command is sudo modprobe usbhid quirks=0xVID:0xPID:0x40
// and you can find your device's VID and PID by running lsusb
// The above is taken from
// http://stackoverflow.com/questions/29358179/usb-possible-to-define-multiple-distinct-hid-joysticks-on-one-interface


/* D-SUB9 pin number, wire color, function
1	White	Up
2	Blue	Down
3	Green	Left/Paddle A button/Driving controller LSB
4	Brown	Right/Paddle B button/Driving controller MSB
5	Red	Paddle B potentiometer
6	Orange	Joystick button
7	Yellow	+5V (needed for paddles)
8	Black	Ground
9	Purple	Paddle A potentiometer

Pressing a button/moving the joystick in a direction shorts the relevant pin with ground. Otherwise, the pin is floating.

Button B:
On the EPYX 500XJ (by Konix) this is pin 9
On the Wico Three Way Deluxe and Super Three way deluxe, turning the switch to F1 and pressing the base button shorts pins 5 and 9


Paddle potentiometer is rated as 1M Ohm, linear. One end is connected to the
+5V power supply (pin 7). Full right is 0 Ohm. Full left should nominally be
1 Mega-Ohm, but I have measured 760K Ohm on one paddle and 900K Ohm
on another.

There are tutorials on how to clean Atari paddles. Maybe I should do this...

My choice of colors:

1	White
2	Blue
3	Green
4	Yellow
5	Orange
6	Red
7	Light green
8	Black
9 Brown

*/

const int minAxisValue = -127; // left/up
const int maxAxisValue = 127;  // right/down
const int midAxisValue = 0;
const int clicksInFullRotation = 16; // driving controller has 4 * 4 = 16 state changes in a 360 degree turn

// digital must come before analog!
enum joyFunc {
    // digital
    fireA  = 0,
    left  = 1,
    right = 2,
    up    = 3,
    down  = 4,

    // analog
    paddleA_pot  = 5,
    paddleB_pot  = 6,

    // note the dual use
    paddleA_button = left,
    paddleB_button = right,

    driving_MSB = up,
    driving_LSB = down,

    // hybrid (fireB)
    fireB = 7,
    fireB_pullup = 7, // S1 in the schematic below

};

const int joyDigitalFuncCount = 5;
const int joyAnalogFuncCount = 2;
const int joyHybridFuncCount = 1;

const int joyTotalFuncCount = 8;

const int analogReadMaxValue = 1023;

const int analogReadTolerance = 1; // if |curr - prev| <= tolerance, treat as no change
// spec gives +- 2LSB as absolute accuracy
const int millisecondsBetweenReads = 2; // Since the 1M Ohm resistor is larger than the maximum suggested resistance going into the A2D converter, some delay is needed so that the position of one paddle does not interfere with the value read from the other one.
/*

Let us first talk about how to read the paddle position. This is done by using a voltage divider.

The voltage divider is +5V --- R1 = potentiometer (paddle) ---|--- R2 --- GND
                                                              |
                                                     tap for analog read

The nominal range of Atari paddles is 0 to 1M Ohm, linear.
So, to get the best precision, it turns out we should take R2 = 1M Ohm,
the maximum resistance of the potentiometer.
If the paddle is not connected, the voltage measured should be 0.
If the paddle is connected, the voltage range should nominally be
between +5V (R1 = 0) and +2.5v (R1 = 1M Ohm).
We take the threshold to decide if the paddle is connected to be +1.25V = 5V/4

The above schematic was a simplified introduction to paddles. In fact, to get both the paddles and fire button B working (pin 9 shorted to ground/pins 5 and 9 shorted), we use the following schematic:

+5V --- R1A  --- joystick pin 9 --- |--- R2A --- GND
     (paddle A)                     |
                                    |--- R3  --- S1 = +5V/not connected (pin joyFuncPins[?][fireB_pullup] on Arduino)
                                    |
                               analog tap A (pin joyFuncPins[?][paddleA_pot] on Arduino)


+5V --- R1B  --- joystick pin 5 --- |--- R2B --- GND
     (paddle B)                     |
                               analog tap B (pin joyFuncPins[?][paddleB_pot] on Arduino)


If the paddles are connected, both R1A and R2A are nominally between 0 and 1M Ohm. Otherwise, both R1A and R2A are infinity.

We start by checking if the paddles are connected.
* S1 is set to "not connected" (the corresponding pin is set to "input").
* If analog tap B is above the threshold of +1.25V (explained above), we know that the paddles are connected.
  + We make use of the reading just taken of analog tap B.
  + We then read analog tap A.
* Otherwise, the paddles are not connected.
  + We set S1 to +5V by setting	 the corresponding pin to "output" and "high". The value of R3 is, also, 1M Ohm.
  + We read joystick up/down/left/right, as well as button A.
  + We make another read of analog tap A.
    # If button B is off, the read should be 5/2 = 2.5 V.
    # If button B is on, by shorting pin 9 to ground, then the analog read should be 0 V.
    # If button B is on, by shorting pins 5 and 9, then the analog read should be 5/3 = 1.66 V.
  + Thus, we set the threshold to be 5 * (5/12) = 2.08 V. If we are above the threshold, button B is off. Otherwise, button B is on.

*/

/* The driving controller uses the up and down joystick pins to encode direction via a Gray code. Let up be the MSB and down be LSB.
                              v---------------
   A left rotation is         00->01->11->10-|

   There are 4 such length 4 cycles in a 360 degree turn of the controller. So, we have a resolution of 360/16 = 22.5 degrees.

   If we read the input and see that both up and down directions are on, then we have clearly connected a driving controller.
   Conversely, if we read the input and see that either the left or right directions are on, then we have clearly connected a joystick.

*/

const int analogReadThreshold = analogReadMaxValue/4;
const int fireBReadThreshold = (analogReadMaxValue * 5)/12;

const int joystickCount = 2;

int currJoyFuncVals[joystickCount][joyTotalFuncCount];
int prevJoyFuncVals[joystickCount][joyTotalFuncCount];

bool isDriving[joystickCount]; // true if a driving controller is connected
unsigned int drivingPos[joystickCount];

int minAnalogJoystickVals[joystickCount][joyAnalogFuncCount];
int maxAnalogJoystickVals[joystickCount][joyAnalogFuncCount];

int joyFuncPins[joystickCount][joyTotalFuncCount];

uint8_t fireAVal;
uint8_t fireBVal;
uint8_t leftVal;
uint8_t rightVal;
uint8_t upVal;
uint8_t downVal;

bool firstTimeFlag;

Joystick_ Joystick[joystickCount] = {
    Joystick_(0x03, JOYSTICK_TYPE_JOYSTICK, /*buttonCount*/ 2, /*hatSwitchCount*/  0, /*includeXAxis*/ true, /*includeYAxis*/ true, /*includeZAxis*/ false, /*includeRxAxis*/ true, false, false, false, false, false, false, false),
    Joystick_(0x04, JOYSTICK_TYPE_JOYSTICK, /*buttonCount*/ 2, /*hatSwitchCount*/  0, /*includeXAxis*/ true, /*includeYAxis*/ true, /*includeZAxis*/ false, /*includeRxAxis*/ true, false, false, false, false, false, false, false),
};

void setup() {

    joyFuncPins[0][fireA]  = 4;
    joyFuncPins[0][left]  = 3;
    joyFuncPins[0][right] = 2;
    joyFuncPins[0][up]    = 1;
    joyFuncPins[0][down]  = 0;
    joyFuncPins[0][paddleA_pot]  = A5;
    joyFuncPins[0][paddleB_pot]  = A4;
    joyFuncPins[0][fireB_pullup]  = 10;

    joyFuncPins[1][fireA]  = 9;
    joyFuncPins[1][left]  = 8;
    joyFuncPins[1][right] = 7;
    joyFuncPins[1][up]    = 6;
    joyFuncPins[1][down]  = 5;
    joyFuncPins[1][paddleA_pot]  = -1;  // not connected
    joyFuncPins[1][paddleB_pot]  = -1;  // not connected
    joyFuncPins[1][fireB_pullup]  = -1; // not connected

    uint8_t i, joystickIndex;

    // Set pin modes and initial analog min/max values

    for ( joystickIndex = 0; joystickIndex < joystickCount; joystickIndex++ )
    {
        // first take care of digital
        for ( i = 0; i < joyDigitalFuncCount; i++ )
        {
            pinMode( joyFuncPins[joystickIndex][i], INPUT_PULLUP );
        }

        isDriving[joystickIndex] = false;  // assume a joystick is connected, as is typically the case
        drivingPos[joystickIndex] = 0;

        // then take care of analog
        for ( i = joyDigitalFuncCount; i < joyDigitalFuncCount + joyAnalogFuncCount; i++ )
        {
            if ( joyFuncPins[joystickIndex][i] == -1 )
            {
                // It seems there is nothing that needs to be done for the pin,
                // just leave it alone: it is already in analog read mode

                continue;
            }


            // Take care of initial max/min values.
            // Nominal max should be analogReadMaxValue.
            // Nominal min should be analogReadMaxValue/2.
            // True max should be very close to analogReadMaxValue (R1 ~= 0, also for old paddles)
            // True min might be noticeably less than analogReadMaxValue/2 (my paddles have R1 max equal to 900K Ohm and 760K Ohm).
            // So, set initial max to 7/8 analogReadMaxValue,
            // and set initial min to 6/8 analogReadMaxValue.

            maxAnalogJoystickVals[joystickIndex][i - joyDigitalFuncCount] = 7 * (analogReadMaxValue/8);
            minAnalogJoystickVals[joystickIndex][i - joyDigitalFuncCount] = 6 * (analogReadMaxValue/8);

        }

        // nothing to do for hybrid (fireB)

        // keep the compiler happy
        for ( i = 0; i < joyTotalFuncCount; i++ )
        {
            prevJoyFuncVals[joystickIndex][i] = 0;
        }

        // set min and max joystick axis values
        Joystick[joystickIndex].setXAxisRange(minAxisValue, maxAxisValue);
        Joystick[joystickIndex].setYAxisRange(minAxisValue, maxAxisValue);

        // set RX range
        Joystick[joystickIndex].setRxAxisRange(0, clicksInFullRotation - 1 );

        // start the joystick
        Joystick[joystickIndex].begin(/*initAutoSendState*/ false);

    }


    firstTimeFlag = true;

    Serial.begin(9600);          //  setup serial (!!!debug!!!)

}

unsigned int regularToGray[4] = {0,1,3,2};
unsigned int *grayToRegular = regularToGray;

// return true if drivingGray implies a change of at most 1 in drivingPos, and return the increment in "increment"
bool validGrayIncrement(unsigned int drivingPos, unsigned int currentGray, int &increment )
{
    unsigned int prevGray = regularToGray[drivingPos % 4];
    unsigned int xorOfGrays = prevGray ^ currentGray;

    if ( xorOfGrays == 3 )
    {
        increment = 100; // some illegal entry
        return false;
    }

    increment = grayToRegular[currentGray] - grayToRegular[prevGray];

    if ( increment == -3 )
    {
        increment = 1;
    }
    else if (increment == 3 )
    {
        increment = -1;
    }


    return true;

}

int paddlePotRead( int joyFuncPin )
{

    if ( joyFuncPin == -1 )
    {
        return 0; // if paddles are not connected, simulate a read of 0
    }
    else
    {
        return myAnalogRead( joyFuncPin );
    }
}

// assume prevJoyFuncVals are set to previous values
// read new values into currJoyFuncVals
// digital: copy currJoyFuncVals to prevJoyFuncVals, return true if there was a change
// analog: copy currJoyFuncVals to prevJoyFuncVals if we are above the threshold of change, and return true if so
// return false otherwise
bool readJoystickVals( uint8_t joystickIndex, int *prevJoyFuncVals, int *currJoyFuncVals, int *joyFuncPins )
{
    uint8_t i;
    bool changedFlag;

    changedFlag = false;

    // First, take care of the simple digital reads
    for ( i = 0; i < joyDigitalFuncCount; i++ )
    {
        currJoyFuncVals[i] = !digitalRead( joyFuncPins[i] );
        if ( currJoyFuncVals[i] != prevJoyFuncVals[i] )
        {
            changedFlag = true;
        }
        prevJoyFuncVals[i] = currJoyFuncVals[i];

    }

    // Check if paddles are connected or not.
    // Start by setting S1 to "not connected"
    if ( joyFuncPins[fireB_pullup] != -1 )
    {
        pinMode( joyFuncPins[fireB_pullup], INPUT );
    }

    // Now, check if paddles are connected by reading digital tap B
    currJoyFuncVals[paddleB_pot] = paddlePotRead( joyFuncPins[paddleB_pot] );

    if (currJoyFuncVals[paddleB_pot] > analogReadThreshold )
    {
        // Paddles are connected

        // Read digital tap A as well
        currJoyFuncVals[paddleA_pot] = paddlePotRead( joyFuncPins[paddleA_pot] );

        // If we are above the change threshold, move current to prev

        for ( i = joyDigitalFuncCount; i < joyDigitalFuncCount + joyAnalogFuncCount; i++ )
        {
            if ( (currJoyFuncVals[i] - prevJoyFuncVals[i]) > analogReadTolerance ||
                    (prevJoyFuncVals[i] - currJoyFuncVals[i]) > analogReadTolerance )
            {
                prevJoyFuncVals[i] = currJoyFuncVals[i];

                changedFlag = true;
            }
        }
    }
    else // Joystick connected, check for fireB
    {
        // to be on the safe side, simulate a read of 0 on paddle A
        currJoyFuncVals[paddleA_pot] = 0;

        // Have we yanked out the paddles?
        if ( prevJoyFuncVals[paddleA_pot] > analogReadThreshold || prevJoyFuncVals[paddleB_pot] > analogReadThreshold )
        {
            prevJoyFuncVals[paddleA_pot] = prevJoyFuncVals[paddleB_pot] = 0;
            changedFlag = true;
        }


        if ( joyFuncPins[fireB_pullup] != -1 && joyFuncPins[paddleA_pot] != -1 )
        {
            // Start by setting S1 to +5V
            pinMode( joyFuncPins[fireB_pullup], OUTPUT );
            digitalWrite( joyFuncPins[fireB_pullup], HIGH );

            // Now, read digital tap B again, and check if we are above or bellow the threshold

            if( paddlePotRead( joyFuncPins[paddleA_pot] ) > fireBReadThreshold )
            {
                currJoyFuncVals[fireB] = 0;
            }
            else
            {
                currJoyFuncVals[fireB] = 1;
            }

            if ( currJoyFuncVals[fireB] != prevJoyFuncVals[fireB] )
            {
                changedFlag = true;
            }
        }
        else
        {
            currJoyFuncVals[fireB] = 0;
        }

        prevJoyFuncVals[fireB] = currJoyFuncVals[fireB];




    }

    /*
            Serial.print("in loop\n");
            Serial.println(i);
            Serial.println(joyFuncPins[i]);
            Serial.println(currJoyFuncVals[i]);
    */

    return changedFlag;
}

void updateAnalogMaxMin( int paddle_pot, int paddle_button, int *currJoyFuncVals, int *minAnalogJoystickVals, int *maxAnalogJoystickVals)
{
    // We only update the values if the fire button is pressed
    if ( currJoyFuncVals[paddle_button] )
    {
        if ( currJoyFuncVals[paddle_pot] < minAnalogJoystickVals[paddle_pot - joyDigitalFuncCount] )
        {
            minAnalogJoystickVals[paddle_pot - joyDigitalFuncCount] = currJoyFuncVals[paddle_pot];
        }

        if ( currJoyFuncVals[paddle_pot] > maxAnalogJoystickVals[paddle_pot - joyDigitalFuncCount] )
        {
            maxAnalogJoystickVals[paddle_pot - joyDigitalFuncCount] = currJoyFuncVals[paddle_pot];
        }

    }
}

float potTransform( int potVal )
{
    return ( (float) analogReadMaxValue ) / ( (float) potVal ) - 1.0;
}

int calculateAnalogAxisValue( int paddle_pot, int paddle_button, int *currJoyFuncVals, int *minAnalogJoystickVals, int *maxAnalogJoystickVals)
{
    int currPotVal, minPotVal, maxPotVal;
    float currTransformVal, maxTransformVal, minTransformVal;

    currPotVal = currJoyFuncVals[paddle_pot];
    minPotVal = minAnalogJoystickVals[paddle_pot - joyDigitalFuncCount];
    maxPotVal = maxAnalogJoystickVals[paddle_pot - joyDigitalFuncCount];

    if ( currPotVal < analogReadThreshold ) // paddle not connected, might as well return mid
    {
        return midAxisValue;
    }

    if ( currPotVal < minPotVal )
    {
        currPotVal = minPotVal;
    }

    if ( currPotVal > maxPotVal )
    {
        currPotVal = maxPotVal;
    }

    currTransformVal = potTransform( currPotVal );
    minTransformVal = potTransform( minPotVal );
    maxTransformVal = potTransform( maxPotVal );

    return ((currTransformVal - minTransformVal) * maxAxisValue + (maxTransformVal - currTransformVal) * minAxisValue)/(maxTransformVal - minTransformVal);
}

void writeJoystickVals( uint8_t joystickIndex, int *currJoyFuncVals, int *minAnalogJoystickVals, int *maxAnalogJoystickVals )
{
    unsigned int currentGray;
    int increment;

    // first, check for paddles
    if ( currJoyFuncVals[paddleA_pot] > analogReadThreshold || currJoyFuncVals[paddleB_pot] > analogReadThreshold )
    {
        // update the max/min values
        updateAnalogMaxMin(paddleA_pot, paddleA_button, currJoyFuncVals, minAnalogJoystickVals, maxAnalogJoystickVals);
        updateAnalogMaxMin(paddleB_pot, paddleB_button, currJoyFuncVals, minAnalogJoystickVals, maxAnalogJoystickVals);

        // check buttons
        Joystick[joystickIndex].setButton(0, currJoyFuncVals[paddleA_button]);
        Joystick[joystickIndex].setButton(1, currJoyFuncVals[paddleB_button]);

        // set axis values
        Joystick[joystickIndex].setXAxis( calculateAnalogAxisValue( paddleA_pot, paddleA_button, currJoyFuncVals, minAnalogJoystickVals, maxAnalogJoystickVals) );
        Joystick[joystickIndex].setYAxis( calculateAnalogAxisValue( paddleB_pot, paddleB_button, currJoyFuncVals, minAnalogJoystickVals, maxAnalogJoystickVals) );

    }
    else // no paddles connected, so either a joystick or a driving controller
    {
        // is it a driving controller for sure?
        if ( currJoyFuncVals[up] && currJoyFuncVals[down] )
        {
            isDriving[joystickIndex]  = true;
        }

        // is it a joystick for sure?
        if ( currJoyFuncVals[left] || currJoyFuncVals[right] )
        {
            isDriving[joystickIndex]  = false;
        }

        // fireA button
        Joystick[joystickIndex].setButton(0, currJoyFuncVals[fireA]);

        // fireB button
        Joystick[joystickIndex].setButton(1, currJoyFuncVals[fireB]);

        if ( isDriving[joystickIndex] == true )
        {
            // start by centering the joystick, for good measure
            Joystick[joystickIndex].setXAxis(midAxisValue);
            Joystick[joystickIndex].setYAxis(midAxisValue);

            currentGray = currJoyFuncVals[driving_LSB] + 2 * currJoyFuncVals[driving_MSB];

            increment = 0; // keep the compiler happy

            if ( validGrayIncrement(drivingPos[joystickIndex], currentGray, increment ) == true )
            {
                drivingPos[joystickIndex] = (drivingPos[joystickIndex] + increment) % clicksInFullRotation;

                Joystick[joystickIndex].setRxAxis( drivingPos[joystickIndex] );
            }


            // for debugging, remove!
            if ( !currJoyFuncVals[up] && !currJoyFuncVals[down] )
            {
                Joystick[joystickIndex].setYAxis(midAxisValue);
            }
            else if ( currJoyFuncVals[up] && !currJoyFuncVals[down] )
            {
                Joystick[joystickIndex].setYAxis(minAxisValue);
            }
            else if ( !currJoyFuncVals[up] && currJoyFuncVals[down] )
            {
                Joystick[joystickIndex].setYAxis(maxAxisValue);
            }
            else // currJoyFuncVals[up] && currJoyFuncVals[down], if we got here, there is a hardware problem!
            {
                Joystick[joystickIndex].setYAxis(100);
            }


        }
        else // it is a joystick
        {
            // start by nulling Rx, for good measure
            Joystick[joystickIndex].setRxAxis(0);


            // left/right
            if ( !currJoyFuncVals[left] && !currJoyFuncVals[right] )
            {
                Joystick[joystickIndex].setXAxis(midAxisValue);
            }
            else if ( currJoyFuncVals[left] && !currJoyFuncVals[right] )
            {
                Joystick[joystickIndex].setXAxis(minAxisValue);
            }
            else if ( !currJoyFuncVals[left] && currJoyFuncVals[right] )
            {
                Joystick[joystickIndex].setXAxis(maxAxisValue);
            }
            else // currJoyFuncVals[left] && currJoyFuncVals[right], we can't get here!
            {
                Joystick[joystickIndex].setXAxis(100);
            }

            // up/down
            if ( !currJoyFuncVals[up] && !currJoyFuncVals[down] )
            {
                Joystick[joystickIndex].setYAxis(midAxisValue);
            }
            else if ( currJoyFuncVals[up] && !currJoyFuncVals[down] )
            {
                Joystick[joystickIndex].setYAxis(minAxisValue);
            }
            else if ( !currJoyFuncVals[up] && currJoyFuncVals[down] )
            {
                Joystick[joystickIndex].setYAxis(maxAxisValue);
            }
            else // currJoyFuncVals[up] && currJoyFuncVals[down], if we got here, there is a hardware problem!
            {
                Joystick[joystickIndex].setYAxis(100);
            }
        }
    }

    Joystick[joystickIndex].sendState();

}

int myAnalogRead( int pin )
{
    // Need to read twice here, since we are generally changing pins.
    // Look at https://www.quora.com/Why-is-a-little-delay-needed-after-analogRead-in-Arduino
    // TODO: For setting the input pin beforehand, I should look at http://www.gammon.com.au/adc

    analogRead(pin);
    delay(millisecondsBetweenReads);
    return analogRead(pin);
}

void loop()
{
    bool stateChange;

    uint8_t joystickIndex;

    for ( joystickIndex = 0; joystickIndex < joystickCount; joystickIndex++ )
    {

        stateChange = readJoystickVals( joystickIndex, prevJoyFuncVals[joystickIndex], currJoyFuncVals[joystickIndex], joyFuncPins[joystickIndex] );

        if ( stateChange == true || firstTimeFlag == true )
        {
            writeJoystickVals( joystickIndex, currJoyFuncVals[joystickIndex], minAnalogJoystickVals[joystickIndex], maxAnalogJoystickVals[joystickIndex] );
        }
    }

    firstTimeFlag = false;
}

