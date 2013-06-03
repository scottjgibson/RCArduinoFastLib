#include <Arduino.h>
void setup();
void loop();
void calcThrottle();
void calcSteering();
void calcAux();
#line 1 "src/sketch.ino"
#include <RCArduinoFastLib.h>

 // MultiChannels
//
// rcarduino.blogspot.com
//
// A simple approach for reading three RC Channels using pin change interrupts
//
// See related posts -
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
// rcarduino.blogspot.com
//

// include the pinchangeint library - see the links in the related topics section above for details
#include <PinChangeInt.h>

// Assign your channel in pins
#define THROTTLE_IN_PIN 5
#define STEERING_IN_PIN 6
#define AUX_IN_PIN 7

// Assign your channel out pins
#define THROTTLE_OUT_PIN 8
#define STEERING_OUT_PIN 9
#define AUX_OUT_PIN 10

// Assign servo indexes
#define SERVO_THROTTLE 0
#define SERVO_STEERING 1
#define SERVO_AUX 2
#define SERVO_FRAME_SPACE 3

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2
#define AUX_FLAG 4

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;
volatile uint16_t unAuxInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint16_t unThrottleInStart;
uint16_t unSteeringInStart;
uint16_t unAuxInStart;

uint16_t unLastAuxIn = 0;
uint32_t ulVariance = 0;
uint32_t ulGetNextSampleMillis = 0;
uint16_t unMaxDifference = 0;

void setup()
{
  Serial.begin(115200);

  Serial.println("multiChannels");

  // attach servo objects, these will generate the correct
  // pulses for driving Electronic speed controllers, servos or other devices
  // designed to interface directly with RC Receivers
  CRCArduinoFastServos::attach(SERVO_THROTTLE,THROTTLE_OUT_PIN);
  CRCArduinoFastServos::attach(SERVO_STEERING,STEERING_OUT_PIN);
  CRCArduinoFastServos::attach(SERVO_AUX,AUX_OUT_PIN);
 
  // lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 3 Servos + 7 times 2000
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,7*2000);

  CRCArduinoFastServos::begin();
 
  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE);
  PCintPort::attachInterrupt(STEERING_IN_PIN, calcSteering,CHANGE);
  PCintPort::attachInterrupt(AUX_IN_PIN, calcAux,CHANGE);
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static uint16_t unAuxIn;
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
  
    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.
  
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }
  
    if(bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }
  
    if(bUpdateFlags & AUX_FLAG)
    {
      unAuxIn = unAuxInShared;
    }
   
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;
  
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }

  // do any processing from here onwards
  // only use the local values unAuxIn, unThrottleIn and unSteeringIn, the shared
  // variables unAuxInShared, unThrottleInShared, unSteeringInShared are always owned by
  // the interrupt routines and should not be used in loop

  // the following code provides simple pass through
  // this is a good initial test, the Arduino will pass through
  // receiver input as if the Arduino is not there.
  // This should be used to confirm the circuit and power
  // before attempting any custom processing in a project.

  // we are checking to see if the channel value has changed, this is indicated
  // by the flags. For the simple pass through we don't really need this check,
  // but for a more complex project where a new signal requires significant processing
  // this allows us to only calculate new values when we have new inputs, rather than
  // on every cycle.
  if(bUpdateFlags & THROTTLE_FLAG)
  {
    CRCArduinoFastServos::writeMicroseconds(SERVO_THROTTLE,unThrottleIn);
  }

  if(bUpdateFlags & STEERING_FLAG)
  {
    CRCArduinoFastServos::writeMicroseconds(SERVO_STEERING,unSteeringIn);
  }

  if(bUpdateFlags & AUX_FLAG)
  {
   CRCArduinoFastServos::writeMicroseconds(SERVO_AUX,unAuxIn);
  }

  delay(500);
  bUpdateFlags = 0;
}


// simple interrupt service routine
void calcThrottle()
{
  if(PCintPort::pinState)
  {
    unThrottleInStart = TCNT1;
  }
  else
  {
    unThrottleInShared = (TCNT1 - unThrottleInStart)>>1;
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  if(PCintPort::pinState)
  {
    unSteeringInStart = TCNT1;
  }
  else
  {
    unSteeringInShared = (TCNT1 - unSteeringInStart)>>1;

    bUpdateFlagsShared |= STEERING_FLAG;
  }
}

void calcAux()
{
  if(PCintPort::pinState)
  {
    unAuxInStart = TCNT1;
  }
  else
  {
    unAuxInShared = (TCNT1 - unAuxInStart)>>1;
    bUpdateFlagsShared |= AUX_FLAG;  }
}

