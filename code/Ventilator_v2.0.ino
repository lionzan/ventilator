/**

Parametric ventilator code

**/

#define DEBUG false

#define _TASK_MICRO_RES
#include <TaskScheduler.h>
// Include the UI lib
#include "OLEDDisplayUi.h"
#include "SSD1306Wire.h" 

// Initialize the OLED display using Wire library
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
SSD1306Wire  display(0x3c, SDA_PIN, SCL_PIN);
OLEDDisplayUi ui ( &display );

#define DIR_PIN GPIO_NUM_12
#define STEP_PIN GPIO_NUM_13
#define TOP_STOP_PIN GPIO_NUM_27
#define BOT_STOP_PIN GPIO_NUM_14
#define LED_PIN LED_BUILTIN

#define NUMBUTTONS 4    // 3 physical buttons + 1 combination of 2
#define DEBOUNCE 5      // how many ms to debounce, 5+ ms is usually plenty

// buttons
#define BUTT_CONFIRM 0x00 // 00
#define BUTT_DOWN    0x01 // 01
#define BUTT_UP      0x02 // 10
#define BUTT_JOINT   0x03 // 11

// buttons ports
#define BUTT_GPIO_CONFIRM GPIO_NUM_18
#define BUTT_GPIO_DOWN    GPIO_NUM_19
#define BUTT_GPIO_UP      GPIO_NUM_5

// buttons status
#define ST_RELEASED   0x00 // 00
#define ST_PRESSED    0x01 // 01
#define ST_ACTIVE     0x02 // 10
#define ST_LONGPRESS  0x03 // 11
#define ST_INHIBITED  0xFF

// delays for button status
#define INHIBIT_JOINT 200     //delay to become a genuine short press, not possible to do joint anymore
#define LONG_START    500     //delay to become a long press
#define MAXINT 4294967295 // the max value of an int after which millis rolls over

// button actions: bits 8-5 button, bit 4 isStart, bits 3-1 status
#define EVENT_END_PRESS   0x01 //0001 relevant
#define EVENT_END_SHORT   0x02 //0010 relevant and same action as END_PRESS
#define EVENT_END_LONG    0x03 //0011 relevant
#define EVENT_START_SHORT 0x06 //0110
#define EVENT_START_LONG  0x07 //0111 relevant

// alternative button actions: bit 1 (SHORT=0; LONG=1) , bit 0 (0=START; 1=END)
#define ACT_SHORT       0x00 // 00
#define ACT_START_LONG  0x01 // 01
#define ACT_END_LONG    0x02 // 10

//display measure
#define SCREEN_W 128
#define SCREEN_H 64

//display states
#define OPERATE  0x00
#define SETUP    0x01

//setup states
#define SET_MODE  0x00
#define SET_VT    0x01
#define SET_PI    0x02
#define SET_RR    0x03
#define SET_IE    0x04
#define SET_PEEP  0x05
#define SET_PLAT  0x06
#define N_SET_ST  0x07

//functioning modes
#define VCV   0x00
#define PCV   0x01
#define PRVC  0x02
#define ACTIVE_MODES 0x01 // number of active modes (1=only VCV | 2=VCV and PCV | 3=all)

//define the buttons
byte buttons[] = {BUTT_GPIO_CONFIRM, BUTT_GPIO_DOWN, BUTT_GPIO_UP}; // button pins

//track if a button is just pressed, just released, or 'currently pressed'
byte pressed[NUMBUTTONS], oldPressed[NUMBUTTONS];
byte justPressed[NUMBUTTONS], justReleased[NUMBUTTONS];
byte buttStatus[NUMBUTTONS] = {ST_RELEASED,ST_RELEASED,ST_RELEASED,ST_RELEASED};
long pressedSince[NUMBUTTONS] = {0,0,0,0};

//display states and functioning modes
byte displayState = OPERATE;
byte setupState;
byte functMode = VCV;

byte activeSetupStates = 0x05;

Scheduler runner;

// Callback methods prototypes
boolean pulseOnEnable();
void pulseOnDisable();

boolean rampOnEnable();
void rampOnDisable();

// External parameters {value, min, max, step}
float tidalVolume[4] = {800.0, 250.0, 800.0, 50.0};    //tidal volume (ml)
float maxPressVCV[4] = {25.0, 15.0, 40.0, 5.0};        //max insp. pressure in VCV (cm H2O)
float respRate[4] = {20.0, 10.0, 30.0, 2.0};           //respiratory rate (1/min)
float ratioEI[4] = {1.0, 1.0, 3.0, 0.1};               //ratio exp/insp (1/1)
float PEEP[4] = {10.0, 5.0, 20.0, 5.0};                //exp. pressure max (cm H2O)
float platPress[4] = {30.0, 0.0, 35.0, 5.0};           //insp. plateau pressure (cm H2O)
float *parameters[6] = {tidalVolume, maxPressVCV, respRate, ratioEI, PEEP, platPress};

// Internal parameters
// Z - Vol mapping curve
int ZcZV[] = {0,10,20,30,40,50,60,70,80,90,100};        //mm [0..Z_MAX]
int VcZV[] = {0,80,160,240,320,400,480,560,640,720,800};//ml [0..TIDAL_MAX]
int cZVlen = 11;

float AR = 0.25;   // ramp acceleration duration as % of total ramp duration

//Mechanical parameters
#define T_PULSE 10         //microsec    pulse duration
#define DT_MIN 90         //microsec    shortest step
#define DT_MAX 1000       //microsec    longest step
#define T_MAX_V 100000     //microsec    time to max speed => acceleration = 1 / (2 * DT_MIN * T_MAX_V ) => 1/2A = DT_MIN * T_MAX_V
#define STEPS_NEMA 200     //#/360deg
#define STEPS_MULT 8       //#
#define THREAD_PITCH 10.0  //mm
#define Z_MIN   0.0        //mechanical limit Z mm
#define Z_MAX 100.0        //mechanical limit Z mm   likely to be determined individually on each machine as variable, not constant
#define Z_VMAX 100.0        //Z at Vmax               likely to be determined individually on each machine as variable, not constant
#define Z_VMIN 1.0        //Z at VMIN = V0          likely to be determined individually on each machine as variable, not constant 
#define S_MIN 0            //S at Z_MIN
#define DIR_UP   HIGH
#define DIR_DOWN LOW    
#define N_STATES 4 
#define ST_PAUSE 0
#define ST_INSPIRE 1
#define ST_RETURN  2
#define ST_TO_ZERO 3

// Internal parameters
long sMax = long ( Z_MAX * STEPS_NEMA * STEPS_MULT / THREAD_PITCH ); 
long sVmin = long ( Z_VMIN * STEPS_NEMA * STEPS_MULT / THREAD_PITCH ); //steps at bag contact (V = 0)
long sVmax = long ( Z_VMAX * STEPS_NEMA * STEPS_MULT / THREAD_PITCH );

// Internal variables
long FT = 4000000; //full cycle duration
long DS; //ramp length in steps
long DT; //ramp duration
long inv2acc = DT_MIN * T_MAX_V;  //ramp acceleration inverse 1/(2*A)
long s;  //step in ramp
long t;  //time in ramp
long dt;  //delta time step in ramp
long sF; //steps at flex point
long tF; //time at flex point
int  st = 0; //state index
int  stNext; //index of next state

struct rampDataStruct {
  long DS;
  long DT;
  boolean dir;
  int stNext;
};

  rampDataStruct rampData[N_STATES] = {   //check parameters, 10/10 gives 80mm run
    {10, FT, DIR_DOWN, ST_INSPIRE},                      //ST_PAUSE
    {(sVmax-sVmin)*50/100, 2000000, DIR_UP, ST_RETURN},  //ST_INSPIRE
    {sMax*11/10, 100000, DIR_DOWN, ST_TO_ZERO},          //ST_RETURN
    {sVmin, 100000, DIR_UP, ST_PAUSE}                    //ST_TO_ZERO
  };



// position flags
boolean atBottom;
boolean atTop;
boolean ackStop=true;
boolean clipped;   //ramp clipped by end of range switch
boolean speedLimit;
boolean dir; //ramp direction

long lastmillis;

// Tasks
Task pulse(T_PULSE, 1, NULL, &runner, false, &pulseOnEnable, &pulseOnDisable);
Task ramp(1, 1, NULL, &runner, true, &rampOnEnable, &rampOnDisable);

// singlePulse calls

boolean pulseOnEnable() {
  if (!DEBUG) {digitalWrite(STEP_PIN, HIGH);}
  return true;
}

void pulseOnDisable() {
  if (!DEBUG) {digitalWrite(STEP_PIN, LOW);}
}

// ramp calls

boolean rampOnEnable() {
  if (DEBUG) {
    Serial.print("st, s, sF, tF, t, dt, ");
    Serial.print(st);
    Serial.print(", ");
    Serial.print(s);
    Serial.print(", ");
    Serial.print(sF);
    Serial.print(", ");
    Serial.print(tF);
    Serial.print(", ");
    Serial.print(t);
    Serial.print(", ");
    Serial.print(dt);
    Serial.print(", ");
    Serial.println();
  }
  if (st != ST_PAUSE) {   //reduce remaining time for cycle completion
    rampData[ST_PAUSE].DT = rampData[ST_PAUSE].DT - dt;
    if (rampData[ST_PAUSE].DT < 0 ) {rampData[ST_PAUSE].DT = 0;}
    pulse.restartDelayed();
  } else {
    rampData[ST_PAUSE].DT = FT;
  }
  t+=dt;
  s++;
  if ( s>=DS ) {  //ramp completed
    st = stNext;
    setRamp();
  } else if ( s<=sF ) {     // before first flex 
    dt = inv2acc / t;
    if ( dt*(DS-2*s) <= (DT-2*t) ) { // speed would make ramp shorter, flex reached
      Serial.println("normal flex");
      dt = (DT-2*t) / (DS-2*s);
      sF = s;
      tF = t;
    } else if ( dt < DT_MIN ) { // too fast, must limit, set flex here (DT will be longer)
      Serial.println("speed limit");
      dt = DT_MIN;
      sF = s;
      tF = t;
      DT = 2*t + dt*(DS-2*s);   //new DT
      speedLimit = true;
    } else if ( 2*s >= DS ) { // midpoint reached
      Serial.println("midpoint reached");
      sF = s;
      tF = t;
      DT = 2*t + dt*(DS-2*s);   //new DT      
    }
  } else if ( s > (DS-sF) ) { // after second flex
    dt = inv2acc / (DT-t);
  } // no further else for flat ramp since the speed is already set

  return true;
}

void rampOnDisable() {
  if ( !ackStop ) {
    boolean oldDir = digitalRead(DIR_PIN);
    do {
      st = stNext;
      setRamp();
    } while ( dir == oldDir ); 
    digitalWrite(DIR_PIN, dir);
    ackStop = true;
  } else if (s == 0) {
    digitalWrite(DIR_PIN, dir);
  }
  ramp.setInterval(dt);
  ramp.restartDelayed();
  
}

void IRAM_ATTR bottomIRS() {
  if (digitalRead(BOT_STOP_PIN) == HIGH) {
    atBottom = true;
    if (digitalRead(DIR_PIN) == DIR_DOWN) {
      ackStop = false;
    }
  } else {
    atBottom = false;
  }
}

void IRAM_ATTR topIRS() {
  if (digitalRead(TOP_STOP_PIN) == HIGH) {
    atTop = true;
    if (digitalRead(DIR_PIN) == DIR_UP) {
      ackStop = false;
    }
  } else {
    atTop = false;
  }
}

void setRamp() {
  DS = rampData[st].DS;
  DT = rampData[st].DT;
  dir = rampData[st].dir;
  Serial.print("setRamp st, DS, DT, dir = ");
  Serial.print(st);
  Serial.print(", ");
  Serial.print(DS);
  Serial.print(", ");
  Serial.print(DT);
  Serial.print(", ");
  Serial.println(dir);
  stNext = rampData[st].stNext;
  inv2acc = DT_MIN * T_MAX_V;  //max acc
  s = 0;
  t = 0;
  sF = DS/2;
  tF = 0;
  dt = long(sqrt(1.0*inv2acc));
  speedLimit = false;
  clipped = false;
}

float VtoZ (float V) {
  //update with polynomial or spline interpolation
  float Z;
  int i = 1;
  if ( V <= VcZV[0] ) {
    Z = 1.0 * ZcZV[0];
  } else if ( V >= VcZV[cZVlen-1] ) {
    Z = 1.0 * ZcZV[cZVlen-1];
  } else {
    while ( V > VcZV[i] ) i++;
    Z = 1.0 * ZcZV[i-1] + (ZcZV[i]-ZcZV[i-1]) * (V-VcZV[i-1]) / (VcZV[i]-VcZV[i-1]);
  }
  return Z;
}

float dV_dZ (float Z) {
  //update with polynomial or spline interpolation
  return 1.0 * (VcZV[cZVlen-1]-VcZV[0])/(ZcZV[cZVlen-1]-ZcZV[0]);
}

bool checkSwitches() {
  bool event = false;
  static byte previousState[NUMBUTTONS];
  static byte currentState[NUMBUTTONS];
  static long lastTime;
  byte index;
  if (millis() < lastTime) {
    // we wrapped around, skip this iteration
    lastTime = millis();
  }
  if ((lastTime + DEBOUNCE) > millis()) {
    // not enough time has passed to debounce
    return event;
  }
  // ok we have waited DEBOUNCE milliseconds, lets reset the timer
  lastTime = millis();
  for (index = 0; index < NUMBUTTONS; index++) {
    justPressed[index] = 0;       //when we start, we clear out the "just" indicators
    justReleased[index] = 0;
    if (index == 3) {
      currentState[index] = digitalRead(buttons[1]) || digitalRead(buttons[2]); // it's an AND of LOW which is when button pressed
    } else {
      currentState[index] = digitalRead(buttons[index]);   //read the button
    }
    if (currentState[index] == previousState[index]) {
      if ((pressed[index] == LOW) && (currentState[index] == LOW)) { //Watch out here LOW has opposite meanings
        // just pressed
        justPressed[index] = 1;
        event=true;
      }
      else if ((pressed[index] == HIGH) && (currentState[index] == HIGH)) {
        justReleased[index] = 1; // just released
        event=true;
      }
      pressed[index] = !currentState[index];  //remember, digital HIGH means NOT pressed
    }
    previousState[index] = currentState[index]; //keep a running tally of the buttons
  }
  return event;
}

byte checkEvent() {
  // update needed: respond to multiple events happening in the same cycle
  byte event = 0xFF;
  if (checkSwitches()==true) {
    for (int i=BUTT_JOINT; i>=BUTT_CONFIRM; i--) {
      if ((justPressed[i]==1) && (buttStatus[i]!=ST_INHIBITED)) {
        buttStatus[i]=ST_PRESSED;
        pressedSince[i] = millis();
        if (i== BUTT_JOINT) {
          buttStatus[BUTT_UP] = ST_INHIBITED;
          buttStatus[BUTT_DOWN] = ST_INHIBITED;
          buttStatus[BUTT_JOINT] = ST_PRESSED;
        }
      } else if (justReleased[i]==1) {
        if (buttStatus[i]!=ST_INHIBITED) {
//          if (buttStatus[i]!=ST_ACTIVE) { // because ST_ACTIVE has already returned ACT_SHORT
//            event = ((byte)i << 4) + (((byte)buttStatus[i]) & 0x02); // ACT_SHORT or ACT_END_LONG
            event = ((byte)i << 4) + 2*(((byte)buttStatus[i]) == ST_LONGPRESS); // ACT_SHORT or ACT_END_LONG
//          }
          if (i==BUTT_UP) {
            buttStatus[BUTT_DOWN]=ST_RELEASED;
          } else if (i==BUTT_DOWN) {
            buttStatus[BUTT_UP]=ST_RELEASED;
          }
        }
        buttStatus[i]=ST_RELEASED;
      }
    }
  }
  // check status function of time
  for (int i=BUTT_JOINT; i>=BUTT_CONFIRM; i--) {
    if ((millis()-pressedSince[i]>LONG_START) && buttStatus[i]==ST_ACTIVE) {
      buttStatus[i] = ST_LONGPRESS;
      event = ((byte)i << 4) + ACT_START_LONG; // with new ACTION states
    } else if ((millis()-pressedSince[i]>INHIBIT_JOINT) && (buttStatus[i]==ST_PRESSED)) {
      buttStatus[i] = ST_ACTIVE;
//      event = ((byte)i << 4) + ACT_SHORT;
      switch (i) {
        case BUTT_UP :{
          buttStatus[BUTT_JOINT] = ST_INHIBITED;
          buttStatus[BUTT_DOWN] = ST_INHIBITED;
          break;
        }
        case BUTT_DOWN :{
          buttStatus[BUTT_JOINT] = ST_INHIBITED;
          buttStatus[BUTT_UP] = ST_INHIBITED;
          break;
        }
      }
    }
  }
  return event;
}

void operateFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  String modeName;
  if (functMode==VCV) {modeName="VCV";}
  else if (functMode==PCV) {modeName="PCV";}
  else if (functMode==PRVC) {modeName="PRVC";}

//rectangle position of each variable
  int x0[] = {         0,             0,  SCREEN_W/2+6,            0, SCREEN_W/2+6};
  int y0[] = {         0,  SCREEN_H/2-8,  SCREEN_H/2-8,  SCREEN_H-12,  SCREEN_H-12};
  int x1[] = {SCREEN_W/4,  SCREEN_W/2-6,  SCREEN_W/2-6, SCREEN_W/2-6, SCREEN_W/2-6};
  int y1[] = {        16,            24,            24,           10,           10};
  
  display->setFont(ArialMT_Plain_10);

  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0 , SCREEN_H-12, " accRamp "+ String(respRate[0], 0) +"/min" );
  
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(SCREEN_W/2+6 , SCREEN_H-12, " I:E 1:"+ String(ratioEI[0], 1) );
  
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0 , SCREEN_H/2+4, "ml" );
  
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(SCREEN_W/2+6 , SCREEN_H/2+4, "cmH2O" );
  
  display->setFont(ArialMT_Plain_16);

  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0 , 0, modeName );
  
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0 , SCREEN_H/2-8, "Vt" );
  
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(SCREEN_W/2+6 , SCREEN_H/2-8, "Pi" );
  
  display->setFont(ArialMT_Plain_24);

  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  if ((displayState == SETUP)) {
    display->drawString(SCREEN_W/2-6 , SCREEN_H/2-10, String(tidalVolume[0], 0) );
  } else {
    long s;
    float alfa = -3.0*PI/4.0*(1.0+2.0*DT_MIN/dt); 
//    display->drawCircle(SCREEN_W/3, SCREEN_H/4, SCREEN_H/4);
    display->drawLine(SCREEN_W/4, SCREEN_H/2, SCREEN_W/4 + SCREEN_H/3 * cos(alfa), SCREEN_H/2 - SCREEN_H/3 * sin(alfa));
//    display->drawString(SCREEN_W/2-6 , SCREEN_H/2-10, String (ZtoV (1.0*Zmax*ss/steps), 0) );
//  } else {
//    display->drawLine(SCREEN_W/3, SCREEN_H/4, SCREEN_W/3 + SCREEN_H/4 * cos(-3.0*PI/4.0), SCREEN_H/4 - SCREEN_H/4 * sin(-3.0*PI/4.0));
//    display->drawString(SCREEN_W/2-6 , SCREEN_H/2-10, "---" );    
  }
  
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  display->drawString(SCREEN_W , SCREEN_H/2-10, String(maxPressVCV[0], 0) );

  if ( displayState == SETUP ) {
    display->setColor(INVERSE);
    display->fillRect(x0[setupState],y0[setupState],x1[setupState],y1[setupState]);
    display->setColor(WHITE);  
  }

}

void operateOverlay (OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->drawCircle(SCREEN_W/3, SCREEN_H/4, SCREEN_H/4);
}

// This array keeps function pointers to all frames
FrameCallback frames[] = { operateFrame };
int frameCount = 1;

// Overlays are statically drawn on top of a frame eg. a clock
//OverlayCallback overlays[] = { operateOverlay };
//int overlaysCount = 1;


void setup () {

  Serial.begin(115200);
  Wire.begin(); //start i2c (required for connection)
  delay(2000);

  Serial.println("setup PINs");
   
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(TOP_STOP_PIN, INPUT_PULLUP);
  pinMode(BOT_STOP_PIN, INPUT_PULLUP);
  attachInterrupt(BOT_STOP_PIN, bottomIRS, CHANGE);
  attachInterrupt(TOP_STOP_PIN, topIRS, CHANGE);
  atBottom = digitalRead(BOT_STOP_PIN);
  atTop = digitalRead(TOP_STOP_PIN);
  if (atBottom || atTop) {ackStop=false;}
  
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);
  
  for (byte i=0; i< NUMBUTTONS-1; i++) {
    pinMode(buttons[i], INPUT);
    pinMode(buttons[i], INPUT_PULLUP);
  }

  Serial.println("setup Display");

  ui.setTargetFPS(10);
  ui.disableAutoTransition();
  ui.disableAllIndicators();
  // Add frames
  ui.setFrames(frames, frameCount);
  // Add overlays
//  ui.setOverlays(overlays, overlaysCount);
  // Initialising the UI will init the display too.
  ui.init();
  ui.switchToFrame(0);
  // Turn the display upside down
  display.flipScreenVertically();

  Serial.println("setup runner");
  rampData[ST_PAUSE].DT = FT;
  runner.startNow();
  runner.disableAll();
  st = ST_RETURN;
  setRamp();
  digitalWrite(DIR_PIN, dir);
  ramp.setInterval(dt);
  ramp.delay();
  ramp.restart();
}


void loop () {
  byte event;
  runner.execute();
  
  if (DEBUG && (millis()>lastmillis+200)) {
    lastmillis = millis();
//    Serial.print("s, t, pulse, ramp: ");
//    Serial.print(t);
//    Serial.print(", ");
//    Serial.print(s);
//    Serial.print(", ");
//    Serial.print(pulse.isEnabled());
//    Serial.print(", ");
//    Serial.println(ramp.isEnabled());
//    Serial.println();
  }
//  

  
  int remainingTimeBudget = ui.update();
  if ((remainingTimeBudget > 0)) {
    event = checkEvent();
    if (event!=0xFF) {
      if (displayState==OPERATE) {       //OPERATE mode
        if (event == 0x01) {                //long press CONFIRM
          displayState = SETUP;               //switch to SETUP mode
          setupState = SET_MODE;
        }
      } else if (displayState==SETUP) {  //SETUP modes
        if (event == 0x00)  {              //short press CONFIRM
          setupState = ++setupState % activeSetupStates; //switch to next SETUP state
        } else if (event == 0x01) {        //long press CONFIRM
          displayState = OPERATE;             //switch to OPERATE mode
        } else if ((event == 0x10) || (event == 0x11)) {        //DOWN press
          if (setupState ==SET_MODE) {  
            functMode = --functMode % ACTIVE_MODES;
            if (functMode != VCV) { activeSetupStates = 0x05; }
            else { activeSetupStates = 0x07; }
          } else if (parameters[setupState-1][0]>parameters[setupState-1][1]) {
            parameters[setupState-1][0] -= parameters[setupState-1][3];
          } else {  //at minimum
            // alert beep or something
          }
        } else if ((event == 0x20) || (event == 0x21)) {        //UP press
          if (setupState ==SET_MODE) {  
            functMode = ++functMode % ACTIVE_MODES;
            if (functMode != VCV) { activeSetupStates = 0x05; }
            else { activeSetupStates = 0x07; }
          } else if (parameters[setupState-1][0]<parameters[setupState-1][2]) {
            parameters[setupState-1][0] += parameters[setupState-1][3];
          } else {  //at maximum
            // alert beep or something
          }
        }
      }
      if (DEBUG) {
        Serial.print("event, displayState, setupState : ");
        Serial.print(event);
        Serial.print(", ");
        Serial.print(displayState);
        Serial.print(", ");
        Serial.println(setupState);
      }
    }
  }
}
