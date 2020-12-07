#include <Arduino.h>
/******************************************************************************
* Dinoi Follower Neuronal Network Version Arduino Mini Pro 5V
* 
* Follower Neuronal Network
* 
* Arduino NN - An artificial neural network for the Arduino
* 
* Based on Neuronal Network from robotics.hobbizine.com/arduinoann.html
******************************************************************************/

#include <math.h>

//*******************************************
// Smart Debug
//*******************************************
// Debugging einschalten, zum ausschalten auskommentieren
#define _SMARTDEBUG

char f[10];

// Debug Makros
#ifdef _SMARTDEBUG

// Hilfsfunktion für WAIT - Makro
void DebugWait(String txt)
{
  // buffer leeren
  char ch;
  while (Serial.available())
    ch = Serial.read();
  ch = 0;

  Serial.print(txt);
  Serial.println(" >press 'c' to continue...");

  // auf 'c' warten
  do
  {
    if (Serial.available() > 0)
      ch = Serial.read();
  } while (ch != 'c');
  // buffer leeren
  while (Serial.available())
    ch = Serial.read();
}

#define DEBUG_INIT(speed) Serial.begin(speed)
#define DEBUG_PRINTLN(txt) Serial.println(txt)
#define DEBUG_PRINT(txt) Serial.print(txt)
#define DEBUG_PRINTLN_VALUE(txt, val) \
  Serial.print(txt);                  \
  Serial.print(": ");                 \
  dtostrf(val, 6, 3, f);              \
  Serial.println(f)
#define DEBUG_PRINT_VALUE(txt, val) \
  Serial.print(txt);                \
  Serial.print(": ");               \
  dtostrf(val, 6, 3, f);            \
  Serial.print(f)
#define DEBUG_WAIT(txt, condition) \
  if (condition)                   \
  DebugWait(txt)
#else
#define DEBUG_INIT(speed)
#define DEBUG_PRINT(txt)
#define DEBUG_PRINTLN(txt)
#define DEBUG_PRINT_VALUE(txt, val)
#define DEBUG_PRINTLN_VALUE(txt, val)
#define DEBUG_WAIT(txt, condition)
#endif

/******************************************************************************
/* Pin Configuration
/******************************************************************************/
#define MDL 4 // Motor Direction Left
#define MSL 5 // Motor Speed Left
#define MSR 6 // Motor Speed Right
#define MDR 7 // Motor Direction Right

int IrLedVr = 3;
int IrLedVl = 2;
int IrLedHr = 10;
int IrLedHl = 9;

int IrRecVr = A3;
int IrRecVl = A2;
int IrRecHr = A0;
int IrRecHl = A1;
const int ledPin = 13; // the number of the LED pin

/* MOTOR DIFFERENCE CONFIG
/******************************************************************************/
#define DIST 65 // Define Minimum Distance

// dino
/*
#define MTRF 220 // Motor Topspeed Right Forward
#define MTRB 220 // Motor Topspeed Right Backward
#define MTLF 187 // Motor Topspeed Left  Forward
#define MTLB 189 // Motor Topspeed Left  Backward
*/

#define MTRF 230  // Motor Topspeed Right Forward
#define MTLB 220  // Motor Topspeed Left  Backward / Forw
#define MTRB 200  // Motor Topspeed Right Backward
#define MTLF 220  // Motor Topspeed Left  Forward  / Backw

#define minIR 20
#define maxIR 220

// normal
/*
#define MTRF 200  // Motor Topspeed Right Forward
#define MTLB 170  // Motor Topspeed Left  Backward / Forw
#define MTRB 200  // Motor Topspeed Right Backward
#define MTLF 170  // Motor Topspeed Left  Forward  / Backw
#define minIR 0
#define maxIR 220
*/

// Variables
int lspeed = 0; // current speed of left motor
int rspeed = 0; // current speed of right motor
int spause = 1; // wait time if a whisker was touched
bool protsta = false;
bool motsta = true;

/******************************************************************
 * Network Configuration - customized per network 
 ******************************************************************/
// Logic
// Sensor -255 to 255
// Motor  -255 to 255
// Normal forward => Sensor 150 => Motor 150
// Obstacle => Sensor 80 => 80

const int PatternCount = 6; // The number of training items or rows in the truth table
const int InputNodes = 2;   // The number of input neurons
const int OutputNodes = 2;  // The number of output neurons

const int HiddenNodes = 5;          // The number of hidden neurons
const float LearningRate = 0.3;     // The number of Learning Rate
const float Momentum = 0.9;         // Adjusts how much the results of the previous iteration affect the current iteration
const float InitialWeightMax = 0.5; // Sets the maximum starting value for weights.
float Success = 0.101;              // 0.02 0.01         // Level of minimum Success

// Obsacle Mode with return
//Success = 0.04;          // Level of minimum Success
/*
float Input[PatternCount][InputNodes] = {
    {1, 1},     // No Obstactle
    {0.8, 0.8}, // No Obstactle
    {0.4, 1},   // Obstacle on left
    {1, 0.4},   // Obstacle on right
    {0.3, 0.3}, // Obstacle left and right
    {0, 0},     // Obstacle left and right
};

const float Target[PatternCount][OutputNodes] = {
    {1, 1},      // No Obstactle
    {0.8, 0.8},  // No Obstactle
    {0.6, -0.3}, // Obstacle on left
    {-0.3, 0.6}, // Obstacle on right
    {0.1, 0.1},  // Obstacle left and right
    {0, 0},      // Obstacle left and right
};
*/

// Obsacle Mode
//Success = 0.04;          // Level of minimum Success

float Input[PatternCount][InputNodes] = {
  { 1, 1 },      // No Obstactle
  { 0.8, 0.8 },  // No Obstactle
  { 0.8, 1 },    // Obstacle on left
  { 1, 0.8 },    // Obstacle on right
  { 0.3, 0.3 },  // Obstacle left and right
  { 0, 0 },  // Obstacle left and right
};

const float Target[PatternCount][OutputNodes] = {
  { 1, 1 },      // No Obstactle
  { 0.8, 0.8 },  // No Obstactle
  { 0.8, 0.3 },      // Obstacle on left
  { 0.3, 0.8 },      // Obstacle on right
  { 0.1, 0.1 },      // Obstacle left and right
  { 0, 0 },      // Obstacle left and right
};

// OK NN
/*
float Input[PatternCount][InputNodes] = {
    {1, 1},     // No Obstactle
    {0.8, 0.8}, // No Obstactle
    {0.5, 1},   // Obstacle on left
    {1, 0.5},   // Obstacle on right
    {0.3, 0.3}, // Obstacle left and right
    {0, 0},     // Obstacle left and right
};

const float Target[PatternCount][OutputNodes] = {
    {1, 1},     // No Obstactle
    {0.7, 0.7}, // No Obstactle
    {0.4, 1},   // Obstacle on left
    {1, 0.4},   // Obstacle on right
    {0.1, 0.1}, // Obstacle left and right
    {0, 0},     // Obstacle left and right
};
*/

// Good NN Agressive
/*
float Input[PatternCount][InputNodes] = {
  { 1, 1 },      // No Obstactle
  { 0.8, 0.8 },// No Obstactle
  { 0.3, 1 },    // Obstacle on left
  { 1, 0.3 },    // Obstacle on right
  { 0.3, 0.3 },  // Obstacle left and right
};

const float Target[PatternCount][OutputNodes] = {
  { 1, 1 },      // No Obstactle
  { 0.7, 0.7 },  // No Obstactle
  { 0, 1 },    // Obstacle on left
  { 1, 0 },    // Obstacle on right
  { 0, 0 },  // Obstacle left and right
}; 
*/

// last ok
/*
float Input[PatternCount][InputNodes] = {
  { 1, 1 },      // No Obstactle
  { 0.65, 0.65 },// No Obstactle
  { 0.22, 1 },    // Obstacle on left
  { 1, 0.22 },    // Obstacle on right
  { 0.22, 0.22 },  // Obstacle left and right
  { 0.11, 0.11 },  // Obstacle left and right close
};

const float Target[PatternCount][OutputNodes] = {
  { 1, 1 },      // No Obstactle
  { 0.9, 0.9 },  // No Obstactle
  { 1, 0.2 },    // Obstacle on left
  { 0.2, 1 },    // Obstacle on right
  { 0.7, 0.7 },  // Obstacle left and right
  { 0.5, 0.5 },  // Obstacle left and right close
};
*/

/******************************************************************
 * Network Variables
 ******************************************************************/
char mode; //(t => train,c=>check, r => run, a = analyze)
int i, j, p, q, r;
int ReportEvery1000;
int RandomizedIndex[PatternCount];
long TrainingCycle;
float Rando;
float Error;
float Accum;

float Hidden[HiddenNodes];
float Output[OutputNodes];
float HiddenWeights[InputNodes + 1][HiddenNodes];
float OutputWeights[HiddenNodes + 1][OutputNodes];
float HiddenDelta[HiddenNodes];
float OutputDelta[OutputNodes];
float ChangeHiddenWeights[InputNodes + 1][HiddenNodes];
float ChangeOutputWeights[HiddenNodes + 1][OutputNodes];

/******************************************************************************
/* Setup
/******************************************************************************/
void setup()
{

  // Neuronal Setup
  /******************************************************************************/
  randomSeed(analogRead(3));
  ReportEvery1000 = 1;
  for (p = 0; p < PatternCount; p++)
  {
    RandomizedIndex[p] = p;
  }
  delay(200);
  mode = 't'; // ( t = train, a = analyze sensors )

  // Robo motors Setup
  /******************************************************************************/
  pinMode(MDL, OUTPUT);
  pinMode(MSL, OUTPUT);
  pinMode(MSR, OUTPUT);
  pinMode(MDR, OUTPUT);

  // all IR Leds / Recevier
  pinMode(ledPin, OUTPUT);
  pinMode(IrLedVr, OUTPUT);
  pinMode(IrLedVl, OUTPUT);
  pinMode(IrLedHr, OUTPUT);
  pinMode(IrLedHl, OUTPUT);
  pinMode(IrRecVr, INPUT);
  pinMode(IrRecVl, INPUT);
  pinMode(IrRecHr, INPUT);
  pinMode(IrRecHl, INPUT);

  // Serial Debug Interface
  DEBUG_INIT(57600);
  DEBUG_PRINTLN("Robot Starting with training");
}

/******************************************************************************
/* InputToOutput
/******************************************************************************/
void InputToOutput(float In1, float In2)
{
  float TestInput[] = {0, 0};
  TestInput[0] = In1;
  TestInput[1] = In2;

  /******************************************************************
    Compute hidden layer activations
  ******************************************************************/

  for (i = 0; i < HiddenNodes; i++)
  {
    Accum = HiddenWeights[InputNodes][i];
    for (j = 0; j < InputNodes; j++)
    {
      Accum += TestInput[j] * HiddenWeights[j][i];
    }
    Hidden[i] = 1.0 / (1.0 + exp(-Accum));
  }

  /******************************************************************
    Compute output layer activations and calculate errors
  ******************************************************************/

  for (i = 0; i < OutputNodes; i++)
  {
    Accum = OutputWeights[HiddenNodes][i];
    for (j = 0; j < HiddenNodes; j++)
    {
      Accum += Hidden[j] * OutputWeights[j][i];
    }
    Output[i] = 1.0 / (1.0 + exp(-Accum));
  }
  /*  DEBUG_PRINT(" Output Nodes");
  for ( i = 0 ; i < OutputNodes ; i++ ) {
    DEBUG_PRINT_VALUE(" ",Output[i]);
  }
*/
}

/******************************************************************************
 * Funktion: irRead (a,b)
 * a: Pinnummer des Ir Empfängers
 * b: Pinnummer des Ir Senders
 * c: modedeb: Space = Normalmodus returns HIGH if > DIST
 *             d = Distancemodus return Distance 0 to 100 mm
 * Retourwert: HIGH = IR-Licht detektiert 
 * d.h. gemessener Wert unterscheidet sich zum Umgebungslicht
 ******************************************************************************/
int irRead(int readPin, int triggerPin, char modedeb)
{
  boolean zustand;
  int umgebungswert = 0;
  int irwert = 0;
  float uLichtzuIr;

  for (int i = 0; i < 10; i++)
  {
    digitalWrite(triggerPin, LOW);
    delay(5);
    umgebungswert = umgebungswert + analogRead(readPin);
    digitalWrite(triggerPin, HIGH);
    delay(5);
    irwert = irwert + analogRead(readPin);
  }

  // Remove Umgebungswert
  irwert = irwert - umgebungswert;

  // detect obstacle
  if (irwert > DIST)
  {
    zustand = HIGH;
  }
  else
  {
    zustand = LOW;
  }

  // for Debug
  if (protsta && zustand || modedeb == 'd')
  {
    delay(5);
    if (modedeb == 'd')
    {
      //Serial.print(" irwert ");
      //Serial.print(irwert);
      irwert = map(irwert, minIR, maxIR, 100, 0);
      constrain(irwert, 0, 100);
      //Serial.print(" dis ");
      //Serial.print(irwert);
    };
    return irwert;
  }
  else
  {
    return zustand;
  };
}

/******************************************************************************
/* Funktion: Drive
/******************************************************************************/
void drive(int left, int right)
{
  int lstate = LOW;
  int rstate = LOW;

  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // invert left
  left = left * -1;
  // invert right
  //right = right * -1;

  if (left >= 0)
  {
    // remap the motor value to the calibrated interval
    // speed is alwas handled in the range of [-255..255]
    left = map(left, 0, 255, 0, MTLF);
  }
  else
  {
    lstate = HIGH;
    left = 255 - map(-left, 0, 255, 0, MTLB);
  }
  if (right >= 0)
  {
    right = map(right, 0, 255, 0, MTRF);
  }
  else
  {
    rstate = HIGH;
    right = 255 - map(-right, 0, 255, 0, MTRB);
  }
  analogWrite(MSL, left);
  digitalWrite(MDL, lstate);

  analogWrite(MSR, right);
  digitalWrite(MDR, rstate);
}

/******************************************************************************
/* Run NN
/******************************************************************************/
void run_nn()
{

  delay(30);

  // Get IR Values
  rspeed = irRead(IrRecVr, IrLedVr, 'd');
  lspeed = irRead(IrRecVl, IrLedVl, 'd');

  //DEBUG_PRINT_VALUE("R IR",rspeed);
  //DEBUG_PRINT_VALUE(" L IR",lspeed);

  // Convert IR to float from 0 to 1
  float f_rspeed = rspeed;
  float f_lspeed = lspeed;
  ;
  f_lspeed = f_lspeed / 100;
  f_rspeed = f_rspeed / 100;

  DEBUG_PRINT_VALUE(" R IR", f_rspeed * 100);
  DEBUG_PRINT_VALUE(" L IR", f_lspeed * 100);

  // Ask solution from NN
  InputToOutput(f_lspeed, f_rspeed); //INPUT to ANN to obtain OUTPUT

  DEBUG_PRINT_VALUE(" NN Out R Mot", Output[1] * 100);
  DEBUG_PRINTLN_VALUE(" L Mot", Output[0] * 100);

  rspeed = Output[1] * 200;
  lspeed = Output[0] * 200;

  //DEBUG_PRINT_VALUE(" - Set Speed R MOT",rspeed);
  //DEBUG_PRINTLN_VALUE(" L MOT",lspeed);

  // Drive Motors
  drive(lspeed, rspeed);
}

/******************************************************************************
/* Run NN
/******************************************************************************/
void analyze_sensors()
{

  delay(30);

  // Get IR Values
  rspeed = irRead(IrRecVr, IrLedVr, 'd');
  lspeed = irRead(IrRecVl, IrLedVl, 'd');

  DEBUG_PRINT_VALUE("R IR", rspeed);
  DEBUG_PRINT_VALUE(" L IR", lspeed);

  // IR from -100 to +200
  // rspeed = constrain(rspeed,-200,200);
  // lspeed = constrain(lspeed,-200,200);

  // Convert IR to float from 0 to 1
  float f_rspeed = rspeed;
  float f_lspeed = lspeed;
  ;
  f_lspeed = (200 + f_lspeed) / 400;
  f_rspeed = (200 + f_rspeed) / 400;

  DEBUG_PRINT_VALUE(" - IN R FL IR", f_rspeed);
  DEBUG_PRINTLN_VALUE(" L FL IR", f_lspeed);
}

/******************************************************************************
/* Check NN
/******************************************************************************/
void check_nn()
{

  float TestInput[] = {0.9, 0.9};

  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("Test Run:");

  // Test 1
  //********
  DEBUG_PRINT_VALUE("Test Input:  L IR", TestInput[0]);
  DEBUG_PRINT_VALUE(" R IR", TestInput[1]);

  InputToOutput(TestInput[0], TestInput[1]); //INPUT to ANN to obtain OUTPUT

  DEBUG_PRINT_VALUE(" Out L Mot", Output[0]);
  DEBUG_PRINTLN_VALUE(" R Mot", Output[1]);

  // Test 2
  //********
  TestInput[0] = 0.4;
  TestInput[1] = 0.9;

  DEBUG_PRINT_VALUE("Test Input:  L IR", TestInput[0]);
  DEBUG_PRINT_VALUE(" R IR", TestInput[1]);

  InputToOutput(TestInput[0], TestInput[1]); //INPUT to ANN to obtain OUTPUT

  DEBUG_PRINT_VALUE(" Out L Mot", Output[0]);
  DEBUG_PRINTLN_VALUE(" R Mot", Output[1]);

  // Test 2
  //********
  TestInput[0] = 0.7;
  TestInput[1] = 0.3;

  DEBUG_PRINT_VALUE("Test Input:  L IR", TestInput[0]);
  DEBUG_PRINT_VALUE(" R IR", TestInput[1]);

  InputToOutput(TestInput[0], TestInput[1]); //INPUT to ANN to obtain OUTPUT

  DEBUG_PRINT_VALUE(" Out L Mot", Output[0]);
  DEBUG_PRINTLN_VALUE(" R Mot", Output[1]);

  // go to run
  mode = 'r';
  DEBUG_PRINTLN(" ");
  DEBUG_PRINTLN("Go to run bot:");
}

void toTerminal()
{

  for (p = 0; p < PatternCount; p++)
  {
    DEBUG_PRINTLN(" ");
    DEBUG_PRINT_VALUE("  Training Pattern", p);
    DEBUG_PRINT("  Input: ");
    for (i = 0; i < InputNodes; i++)
    {
      DEBUG_PRINT(Input[p][i]);
      DEBUG_PRINT(" ");
    }
    DEBUG_PRINT("  Target: ");
    for (i = 0; i < OutputNodes; i++)
    {
      DEBUG_PRINT(Target[p][i]);
      DEBUG_PRINT(" ");
    }

    /******************************************************************
* Compute hidden layer activations
******************************************************************/

    for (i = 0; i < HiddenNodes; i++)
    {
      Accum = HiddenWeights[InputNodes][i];
      for (j = 0; j < InputNodes; j++)
      {
        Accum += Input[p][j] * HiddenWeights[j][i];
      }
      Hidden[i] = 1.0 / (1.0 + exp(-Accum));
    }

    /******************************************************************
* Compute output layer activations and calculate errors
******************************************************************/

    for (i = 0; i < OutputNodes; i++)
    {
      Accum = OutputWeights[HiddenNodes][i];
      for (j = 0; j < HiddenNodes; j++)
      {
        Accum += Hidden[j] * OutputWeights[j][i];
      }
      Output[i] = 1.0 / (1.0 + exp(-Accum));
    }
    DEBUG_PRINT("  Output ");
    for (i = 0; i < OutputNodes; i++)
    {
      DEBUG_PRINT(Output[i]);
      DEBUG_PRINT(" ");
    }
  }
}

/******************************************************************************
/* Train NN
/******************************************************************************/
void train_nn()
{
  /******************************************************************
* Initialize HiddenWeights and ChangeHiddenWeights 
******************************************************************/

  for (i = 0; i < HiddenNodes; i++)
  {
    for (j = 0; j <= InputNodes; j++)
    {
      ChangeHiddenWeights[j][i] = 0.0;
      Rando = float(random(100)) / 100;
      HiddenWeights[j][i] = 2.0 * (Rando - 0.5) * InitialWeightMax;
    }
  }
  /******************************************************************
* Initialize OutputWeights and ChangeOutputWeights
******************************************************************/

  for (i = 0; i < OutputNodes; i++)
  {
    for (j = 0; j <= HiddenNodes; j++)
    {
      ChangeOutputWeights[j][i] = 0.0;
      Rando = float(random(100)) / 100;
      OutputWeights[j][i] = 2.0 * (Rando - 0.5) * InitialWeightMax;
    }
  }
  DEBUG_PRINTLN("Initial/Untrained Outputs: ");
  toTerminal();
  /******************************************************************
* Begin training 
******************************************************************/

  for (TrainingCycle = 1; TrainingCycle < 2147483647; TrainingCycle++)
  {

    /******************************************************************
* Randomize order of training patterns
******************************************************************/

    for (p = 0; p < PatternCount; p++)
    {
      q = random(PatternCount);
      r = RandomizedIndex[p];
      RandomizedIndex[p] = RandomizedIndex[q];
      RandomizedIndex[q] = r;
    }
    Error = 0.0;
    /******************************************************************
* Cycle through each training pattern in the randomized order
******************************************************************/
    for (q = 0; q < PatternCount; q++)
    {
      p = RandomizedIndex[q];

      /******************************************************************
* Compute hidden layer activations
******************************************************************/

      for (i = 0; i < HiddenNodes; i++)
      {
        Accum = HiddenWeights[InputNodes][i];
        for (j = 0; j < InputNodes; j++)
        {
          Accum += Input[p][j] * HiddenWeights[j][i];
        }
        Hidden[i] = 1.0 / (1.0 + exp(-Accum));
      }

      /******************************************************************
* Compute output layer activations and calculate errors
******************************************************************/

      for (i = 0; i < OutputNodes; i++)
      {
        Accum = OutputWeights[HiddenNodes][i];
        for (j = 0; j < HiddenNodes; j++)
        {
          Accum += Hidden[j] * OutputWeights[j][i];
        }
        Output[i] = 1.0 / (1.0 + exp(-Accum));
        OutputDelta[i] = (Target[p][i] - Output[i]) * Output[i] * (1.0 - Output[i]);
        Error += 0.5 * (Target[p][i] - Output[i]) * (Target[p][i] - Output[i]);
      }

      /******************************************************************
* Backpropagate errors to hidden layer
******************************************************************/

      for (i = 0; i < HiddenNodes; i++)
      {
        Accum = 0.0;
        for (j = 0; j < OutputNodes; j++)
        {
          Accum += OutputWeights[i][j] * OutputDelta[j];
        }
        HiddenDelta[i] = Accum * Hidden[i] * (1.0 - Hidden[i]);
      }

      /******************************************************************
* Update Inner-->Hidden Weights
******************************************************************/

      for (i = 0; i < HiddenNodes; i++)
      {
        ChangeHiddenWeights[InputNodes][i] = LearningRate * HiddenDelta[i] + Momentum * ChangeHiddenWeights[InputNodes][i];
        HiddenWeights[InputNodes][i] += ChangeHiddenWeights[InputNodes][i];
        for (j = 0; j < InputNodes; j++)
        {
          ChangeHiddenWeights[j][i] = LearningRate * Input[p][j] * HiddenDelta[i] + Momentum * ChangeHiddenWeights[j][i];
          HiddenWeights[j][i] += ChangeHiddenWeights[j][i];
        }
      }

      /******************************************************************
* Update Hidden-->Output Weights
******************************************************************/

      for (i = 0; i < OutputNodes; i++)
      {
        ChangeOutputWeights[HiddenNodes][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[HiddenNodes][i];
        OutputWeights[HiddenNodes][i] += ChangeOutputWeights[HiddenNodes][i];
        for (j = 0; j < HiddenNodes; j++)
        {
          ChangeOutputWeights[j][i] = LearningRate * Hidden[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i];
          OutputWeights[j][i] += ChangeOutputWeights[j][i];
        }
      }
    }

    /******************************************************************
* Every 1000 cycles send data to terminal for display
******************************************************************/
    ReportEvery1000 = ReportEvery1000 - 1;
    if (ReportEvery1000 == 0)
    {

      // Blink Led for State Info
      digitalWrite(ledPin, HIGH);
      delay(200);
      digitalWrite(ledPin, LOW);
      delay(200);
      digitalWrite(ledPin, HIGH);
      delay(200);
      digitalWrite(ledPin, LOW);
      delay(200);

      DEBUG_PRINTLN("");
      DEBUG_PRINTLN("");
      DEBUG_PRINT_VALUE("  TrainingCycle", TrainingCycle);
      DEBUG_PRINT_VALUE("  Error Level", Error);

      toTerminal();

      if (TrainingCycle == 1)
      {
        ReportEvery1000 = 999;
      }
      else
      {
        ReportEvery1000 = 1000;
      }
    }

    /******************************************************************
* If error rate is less than pre-determined threshold then end
******************************************************************/

    if (Error < Success)
      break;
  }
  DEBUG_PRINTLN("");
  DEBUG_PRINT_VALUE("  TrainingCycle: ", TrainingCycle);
  DEBUG_PRINT_VALUE("  Error Level", Error);

  toTerminal();

  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("");
  DEBUG_PRINT("Training Set Solved!");
  DEBUG_PRINT_VALUE("TrainingCycle: ", TrainingCycle);
  DEBUG_PRINTLN_VALUE("  Error Level", Error);
  DEBUG_PRINTLN("");

  ReportEvery1000 = 1;

  // done=> set mode to run
  mode = 'c';
}

/******************************************************************************
/* Loop
/******************************************************************************/
void loop()
{

  // Train NN
  /******************************************************************************/
  if (mode == 't')
  {
    train_nn();
  }

  // Check NN
  /******************************************************************************/
  if (mode == 'c')
  {
    check_nn();
  }

  // Run NN
  /******************************************************************************/
  if (mode == 'r')
  {
    run_nn();
  }

  // Analyze Sensors
  /******************************************************************************/
  if (mode == 'a')
  {
    analyze_sensors();
  }
}
