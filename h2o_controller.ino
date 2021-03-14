/*
 *                         ATtiny85
 *                      -------u-------
 *        NC - (D 5) --| 1 PB5   VCC 8 |-- +5V
 *                     |               |
 *       NTC - (D 3) --| 2 PB3   PB2 7 |-- (D 2) - Serial TX
 *                     |               | 
 *        NC - (D 4) --| 3 PB4   PB1 6 |-- (D 1) - PWM --> Fan Blue wire
 *                     |               |      
 *              Gnd ---| 4 GND   PB0 5 |-- (D 0) - NC
 *                     -----------------
 */


#define F_CPU 8000000

#include <SendOnlySoftwareSerial.h>
#include <avr/interrupt.h>
#include <String.h>
#include <avr/io.h>

// Globals
#define SERIESRESISTOR 10000
#define THERMISTORNOMINAL 10000
#define BCOEFFICIENT 3950
#define TEMPERATURENOMINAL 25.0

// Init globals
int pwmtop = 159; // Calculates to 25KHz PWM for fanpwm
double Setpoint, Input, Output;
volatile uint64_t lasttime = 0;
volatile uint64_t tickms = 0;

//PID library modified to use 64-bit millis.
#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION  1.1.1

class PID
{

  public:

  //Constants used in some of the functions below
  #define AUTOMATIC 1
  #define MANUAL  0
  #define DIRECT  0
  #define REVERSE  1
  #define P_ON_M 0
  #define P_ON_E 1

  //commonly used functions **************************************************************************
    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        double, double, double, int, int);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        double, double, double, int);     //   Setpoint.  Initial tuning parameters are also set here
  
    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default, but
                                          //   it's likely the user will want to change this depending on
                                          //   the application
  


  //available but not commonly used functions ********************************************************
    void SetTunings(double, double,       // * While most users will set the tunings once in the 
                    double);              //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(double, double,       // * overload for specifying proportional mode
                    double, int);             

  void SetControllerDirection(int);   // * Sets the Direction, or "Action" of the controller. DIRECT
                      //   means the output will increase when error is positive. REVERSE
                      //   means the opposite.  it's very unlikely that this will be needed
                      //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
                      
                      
                      
  //Display functions ****************************************************************
  double GetKp();             // These functions query the pid for interal values.
  double GetKi();             //  they were created mainly for the pid front-end,
  double GetKd();             // where it's important to know what is actually 
  int GetMode();              //  inside the PID.
  int GetDirection();           //

  private:
  void Initialize();
  
  double dispKp;        // * we'll hold on to the tuning parameters in user-entered 
  double dispKi;        //   format for display purposes
  double dispKd;        //
    
  double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

  int controllerDirection;
  int pOn;

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the 
    double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
        
  unsigned long lastTime;
  double outputSum, lastInput;

  unsigned long SampleTime;
  double outMin, outMax;
  bool inAuto, pOnE;
};
#endif

/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

//#include <PID_v1.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int POn, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    PID::SetOutputLimits(0, 255);        //default output limit corresponds to
                        //the arduino pwm limits

    SampleTime = 100;             //default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd, POn);

    lastTime = tickms-SampleTime;
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
    :PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection)
{

}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = tickms;
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double input = *myInput;
      double error = *mySetpoint - input;
      double dInput = (input - lastInput);
      outputSum+= (ki * error);

      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!pOnE) outputSum-= kp * dInput;

      if(outputSum > outMax) outputSum= outMax;
      else if(outputSum < outMin) outputSum= outMin;

      /*Add Proportional on Error, if P_ON_E is specified*/
     double output;
      if(pOnE) output = kp * error;
      else output = 0;

      /*Compute Rest of PID Output*/
      output += outputSum - kd * dInput;

      if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
      *myOutput = output;

      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
      return true;
   }
   else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd){
    SetTunings(Kp, Ki, Kd, pOn); 
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
     if(*myOutput > outMax) *myOutput = outMax;
     else if(*myOutput < outMin) *myOutput = outMin;

     if(outputSum > outMax) outputSum= outMax;
     else if(outputSum < outMin) outputSum= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *  does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
   outputSum = *myOutput;
   lastInput = *myInput;
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}

// end PID library


SendOnlySoftwareSerial Serial(PB2); // Start Serial on PB2
PID myPID(&Input, &Output, &Setpoint, 10.0, 5.0, 2.5, REVERSE);

// Functions
void fanpwm_setup(int ocr0top)
{
  /*
  Setup timer0 to generate a 25KHz PWM signal to COM0B1.
  */
  // Undo config by Arduino.
  TCCR0A  = 0;    
  TCCR0B  = 0;
  OCR0A   = 0;
  OCR0B   = 0;
  // Rebuild timer0
  TCCR0A  = (1<<COM0B1) | (1<<WGM02) | (1<<WGM00);  //Fast PWM, Phase Correct.
  TCCR0B  = (1<<CS00) | (1<<WGM02);                 //F_CPU / 1
  OCR0A   = ocr0top;                                // Set to 159 for 25khz.
  OCR0B   = 0;                                      // Duty Cycle in 0 - ocr0top.
}

void timer1_init(void){
  /*
  This timer will fire ISR(TIMER1_COMPA_vect) on match.
  */
  TCCR1 = 0;
  OCR1C = 0;
  TIMSK = 0;
  
  TCCR1 |= (1<<CTC1);   //CTC mode. Count until OCR1C, then reset
  TCCR1 |= (7<<CS10);   //F_CPU / 64
  OCR1C = 124;          //1 ms
  TIMSK |= (1<<OCIE1A); //Call ISR(TIMER1_COMPA_vect) on match
}

ISR(TIMER1_COMPA_vect){
  tickms++;
}

double readtemp(){ // returns Celcius value of NTC temp on Pin 3.
  double reading = analogRead(PB3);
  reading = (1023 / reading)  - 1;                // (1023/ADC - 1)
  reading = SERIESRESISTOR / reading;
  reading = reading / THERMISTORNOMINAL;          // (R/Ro)
  reading = log(reading);                         // ln(R/Ro)
  reading /= BCOEFFICIENT;                        // 1/B * ln(R/Ro)
  reading += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  reading = 1.0 / reading;                        // Invert
  reading -= 273.15;                              // convert absolute temp to C
  
  return reading;
}

void setup(){
  timer1_init();
  fanpwm_setup(pwmtop);  // Start PWM on pin6
  DDRB |= (1<<DDB1)      // Redirect PWM from timer1 to PB1, Pin6
       |  (1<<DDB0)
       |  (1<<DDB4)   
       |  (1<<DDB5);     
  PORTB |= (1<<PB0) | (1<<PB4)| (1<<PB5);     // pull up on PB4 and PB4
        
  // Setup pid
  Setpoint = 35;
  Input = readtemp();
  myPID.SetTunings(10.0, 4.0, 2.0);
  myPID.SetSampleTime(100);
  myPID.SetMode(AUTOMATIC);

  while (tickms < 5000){ // Run fans at 100% RPM on startup.
    OCR0B = pwmtop;
    }
  
  Serial.begin(9600);    // Start serial
  Serial.print("\r\nBootup complete!\r\n");
}

void loop(){
  Input = readtemp();
  myPID.Compute();
  OCR0B = map(Output,0,255,15,pwmtop);
  
  uint64_t now = tickms;
  if ( (now - lasttime) >= 1000 ){
    float error = Input - Setpoint;
    Serial.print(Input);   // print temp
    Serial.print(", ");
    Serial.print(error);   // print error setting
    Serial.print(", ");
    Serial.print( OCR0B ); // print pwm setting
    Serial.print("\r\n");
    lasttime = now;  
  }
}
