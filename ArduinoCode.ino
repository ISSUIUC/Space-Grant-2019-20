//Imports and instance variables below

#include <Wire.h> //Import necessary for I2C communication
#include <Servo.h> //import necessary for servos.

Servo servo1;
Servo servo2;

//VARIABLES:
//**Ones are placeholders**
float az;                         //Accelerationin z direction
bool launch = false;              //Tracks if aruduino has detected launch
bool launch_init = false;         //True whenever acceleration exceeds threshold
int launch_az_thresh = 1;         //Minimum acceleration for launch to be detected
int launch_time_thresh = 1;       //Amount of time (ms) acceleration must exceed threshold to detect launch
float launch_time = 0;            //First time acceleration above launch threshold is detected
float burn_timer = 0;             //Measures length of burrn for launch time threshold
bool free_fall = false;           //Tracks if rocket is past burnout
bool free_fall_init = false;      //True whenever negative acceleration exceeds threshold after launch
int free_fall_thresh = 1;         //Minimum negative acceleration for burnout to be detected **should be negative**
int freefall_time_thresh = 1;     //Amount of time (ms) acceleration must exceed threshold to detect burnout
float burnout_time = 0;           //First time negaative acceleration exceeding threshold is detected
float coast_timer = 0;            //Measures length of negative acceleration for free fall time threshold
float roll_rate;                  //Rate of roatation around the z axis
int roll_rate_thresh = 1;         //Minimum roll rate for roll control activation
float alt;                        //Altitude measured from altimeter
int roll_control_cutoff = 1;      //Altitude at which the rocket switches exclusively to active drag
bool apogee = false;              //Defines the end of the coast phase when true
float vel;                        //Velocity data received from the Pi
float des_alt;                    //Desired altitude of the flight, either 2345 ft or 3456 ft
float g = 9.81;                   //Acceleration due to gravity in metres/sec
float native_drag;                //Native drag force, have to see how to calculate this
float flaps_drag;                 //Additional drag force due to the deployed flaps
float m;                          //Mass of the rocket in kilograms

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  //az must be updated from sensor each time through loop
  
  //LAUNCH DETECTION LOGIC:
  if(az > launch_az_thresh && launch_init == false)     //If high acceleration is observed in z direction...
  {
    launch_time = millis();                             //...assume launch and store launch time
    launch_init = true;
  }
  
  if(az > launch_az_thresh && launch_init == true && free_fall == false)   //If the acceleration continues...
  {
    burn_timer = millis() - launch_time;                //...start measuring the lenght of the burn time
    if(burn_timer > launch_time_thresh)                 //If the acceleration lasts long enough...
    {
      launch = true;                                    //...the launch has occured
      Serial.println(String(millis()/1000) + "s: Launch detected");
    }
  }
  
  else if(az < launch_az_thresh && launch_init == true && free_fall == false)   //If the acceleration was too brief...
  {
    launch_init = false;                                //...reset the launch detection (the acceleration was just an anomaly)
  }

  //BURNOUT DETECTION LOGIC:
  if(az < free_fall_thresh && launch == true && free_fall_init == false)    //If large negative acceleration is observed after launch...
  {
    burnout_time = millis();                            //...assume burnout and store time of burnout
    free_fall_init = true;
  }

  if(az < free_fall_thresh && free_fall_init == true && free_fall == false)   //If the negative acceleration continues...
  {
     coast_timer = millis() - burnout_time;             //...start measuring the lenght of the coast time
     if(coast_timer > freefall_time_thresh)             //If the negative acceleration lasts long enough...  
     {
      free_fall = true;                                 //...burnout has occured, and the rocket is now coasting
      Serial.println(String(millis()/1000) + "s: Burnout detected");
     }
  }

  else if(az > free_fall_thresh && free_fall_init == true && free_fall == false)   //If the negative acceleration was too brief...
  {
    free_fall_init = false;                             //...reset the burnout detection (the acceleration was just an anomaly)
  }
  
  //ROLL CONTROL
  if((roll_rate > roll_rate_thresh || roll_rate < -roll_rate_thresh) && alt < roll_control_cutoff && free_fall == true && apogee == false)    //If the rocket is rolling and is in the coast phase below a certain altitude...
  {
    //...reduce the roll
  }
  //ACTIVE DRAG
  else if(free_fall == true && apogee == false)         //If the rocket is in the coast phase...
  {
    //...run active drag
    //des_vel = f(altitude)
    float des_vel = sqrt(2*(g+(native_drag/m)*(des_alt-alt)); //From v^2 - u^2 = 2*a*s equation. THe final velocity is zero, u is the desired velocity considering flap drag is applied throughout
    
    //to see if the current velocity is greater than the no-flap-deployed-desired velocity
    
    
    
  }
}
