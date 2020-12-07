#include <Arduino.h>
/******************************************************************************
// FreakyBot Follower bot formula 
/************************* *****************************************************/
/* Speed = ( Distance – 0.3 )  / 0.7
/******************************************************************************

/******************************************************************************
/* Configuration
/******************************************************************************/

/* Pin Configuration
/******************************************************************************/
#define MDL 4     // Motor Direction Left
#define MSL 5     // Motor Speed Left
#define MSR 6     // Motor Speed Right
#define MDR 7     // Motor Direction Right

int IrLedVr = 3;   
int IrLedVl = 2; 
int IrLedHr = 10; 
int IrLedHl = 9;

int IrRecVr = A3;   
int IrRecVl = A2; 
int IrRecHr = A0; 
int IrRecHl = A1;
const int ledPin =  13;      // the number of the LED pin

#define DIST 65   // Define Minimum Distance

/* MOTOR DIFFERENCE CONFIG
/******************************************************************************/
// dino
/*
#define MTRF 200  // Motor Topspeed Right Forward
#define MTRB 200  // Motor Topspeed Right Backward
#define MTLF 170  // Motor Topspeed Left  Forward
#define MTLB 180  // Motor Topspeed Left  Backward
*/

// normal
#define MTRF 200  // Motor Topspeed Right Forward
#define MTLB 200  // Motor Topspeed Left  Backward / Forw
#define MTRB 200  // Motor Topspeed Right Backward
#define MTLF 170  // Motor Topspeed Left  Forward  / Backw

// Variables
float lspeed = 0;   // current speed of left motor
float rspeed = 0;   // current speed of right motor
int   spause = 1;   // wait time if a whisker was touched
long  delay_time     = 0;        // time when pause was triggered
bool  protsta        = false;
bool  motsta         = true;

/******************************************************************************
/* Setup
/******************************************************************************/
void setup () {
  //motors
  pinMode (MDL, OUTPUT);
  pinMode (MSL, OUTPUT);
  pinMode (MSR, OUTPUT);
  pinMode (MDR, OUTPUT); 
  // all IR Leds / Recevier
  pinMode(IrLedVr, OUTPUT);   
  pinMode(IrLedVl, OUTPUT); 
  pinMode(IrLedHr, OUTPUT); 
  pinMode(IrLedHl, OUTPUT);  
  pinMode(IrRecVr, INPUT);
  pinMode(IrRecVl, INPUT); 
  pinMode(IrRecHr, INPUT);
  pinMode(IrRecHl, INPUT); 
  // Serial Interface
  Serial.begin(57600); 
  Serial.println(F("Robot Starting")); 
  delay(2000);
}


/******************************************************************************
/* Funktionen
/******************************************************************************/

/******************************************************************************
/* Funktion: Drive
/******************************************************************************/
void drive( int left, int right)
{
  int lstate = LOW;
  int rstate = LOW;
 
 left = constrain(left,-255,255);
 right = constrain(right,-255,255);
 
// invert left
left = left * -1;
// invert right
//right = right * -1;
 
  if (left >= 0)
  {
    // remap the motor value to the calibrated interval
    // speed is alwas handled in the range of [-255..255]
    left = map(left,0,255,0,MTLF);
  }
  else
  {
    lstate = HIGH;
    left = 255 - map(-left,0,255,0,MTLB);
  }
  if (right >= 0)
  {
    right = map(right,0,255,0,MTRF);
  }
  else
  {
    rstate = HIGH;
    right = 255 - map(-right,0,255,0,MTRB);
  }
  analogWrite(MSL, left);
  digitalWrite(MDL, lstate);
  analogWrite(MSR, right);
  digitalWrite(MDR, rstate);

}

/******************************************************************************
 * Funktion: irRead (a,b)
 * a: Pinnummer des Ir Empfängers
 * b: Pinnummer des Ir Senders
 * c: Modus: Space = Normalmodus returns HIGH if > DIST
 *           d = Distancemodus return Distance 0 to 100 mm
 * Retourwert: HIGH = IR-Licht detektiert 
 * d.h. gemessener Wert unterscheidet sich zum Umgebungslicht
 ******************************************************************************/
int irRead(int readPin, int triggerPin, char mode)
{
    boolean zustand;
    int umgebungswert = 0;
    int irwert =0;
    float uLichtzuIr;
    
    for (int i=0;i<10;i++) {
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
    if(irwert>DIST){
     zustand = HIGH;
    }else{
     zustand = LOW;
    }


     if(protsta && zustand || mode == 'd'){

     // for Debug 
/*     Serial.print("tr ");
     Serial.print(triggerPin);
     Serial.print(" r ");
     Serial.print(readPin);
     Serial.print(" um ");
     Serial.print(umgebungswert);
     Serial.print(" ir ");
     Serial.print(irwert);
     Serial.print(" % ");
     Serial.print(uLichtzuIr); 
*/

     if(mode == 'd'){
     //Serial.print(" irwert ");
     //Serial.print(irwert); 
     irwert = map(irwert, 0, 220, 100, 0);
     constrain(irwert,0,100);
     //Serial.print(" dis ");
     //Serial.print(irwert); 
};     
     return irwert;
	 } else {
     return zustand; 
//     Serial.print(" zu ");
//     Serial.println(zustand);
 };
  
}

// CHECKSERIAL
//*****************************************************************************
void checkserial(){
  	if ( Serial.available() > 0 ) {
		// Read the incoming byte
		char theChar = Serial.read();
			// Parse character
		switch (theChar) {
			case 'p':
         Serial.println(F("protocoll on/off"));
         delay(5);
         if (protsta){
				   protsta = false;	
				 }else{
				   protsta = true;		
	       }	
         delay(5);
         break;
			case 'm':
         delay(10);
         if (motsta){
					 motsta = false;	
				 }else{
					 motsta = true;		
				}	
        delay(5);
        break;
        default:
  				// Display error message
  				Serial.print(F("ERROR: Command not found, it was: "));
  				Serial.println(theChar);
  				Serial.println(F("p=protocoll on/off;m=motor on/off"));
          break;
       }
   }
} // void checkserial

/******************************************************************************
/* Hauptprogramm
/******************************************************************************/
void loop () {
    delay(5);
    checkserial(); 

/*  Normal Way forward 
  if ( irRead(IrRecVl,IrLedVl,' ')==HIGH && 
      irRead(IrRecVr,IrLedVr,' ')==HIGH )
    {
    // left and right whisker touched, turn both motor backwards left faster
    rspeed = -128;
    lspeed = -150;
    spause = 500;
    digitalWrite(ledPin,HIGH);    
    delay(10);
    }
  
 else if (irRead(IrRecVl,IrLedVl,' ')==HIGH)
    {
    // left whisker touched, turn right motor backwards
    rspeed = -128;
    spause = 500;
    digitalWrite(ledPin,HIGH);    
    delay(10);
    }
 else if (irRead(IrRecVr,IrLedVr,' ')==HIGH)
    {
    // right whisker touched, turn left motor backwards
    lspeed = -128;
    spause = 500;
    digitalWrite(ledPin,HIGH);    
    delay(10);
    }
*/ // Normal Way Forward End

// Follow Mode => directly drive motors with distance value
int right_ir = irRead(IrRecVr,IrLedVr,'d');
int left_ir = irRead(IrRecVl,IrLedVl,'d');

float right_ir_f = right_ir;
float left_ir_f = left_ir;
char f[10];

right_ir_f = right_ir_f / 100;
left_ir_f = left_ir_f / 100;

 Serial.print(" R IR: ");
 dtostrf(right_ir_f*100,6,3,f);
 Serial.print(f);
 
 Serial.print(" L IR: ");
 dtostrf(left_ir_f*100,6,3,f);
 Serial.print(f);

  rspeed = ( right_ir_f - 0.3 ) / 0.6;
  lspeed = ( left_ir_f - 0.3 ) / 0.6;

 /*
 Serial.print(" R IR2: ");
 Serial.print(rspeed);
 Serial.print(" L IR2: ");
 Serial.print(lspeed);
 */

 // IR from -1 to 1
 rspeed  = constrain(rspeed,-1,1);
 lspeed = constrain(lspeed,-1,1);

 Serial.print(" - R MOT: ");
 dtostrf(rspeed*100,6,3,f);
 Serial.print(f);
//Serial.print(rspeed * 100, DEC);
 Serial.print(" L MOT: ");
 dtostrf(lspeed*100,6,3,f);
 Serial.print(f);
// Serial.print(lspeed * 100, DEC);

/*         Serial.print(" l ");
         Serial.print(lspeed);
         Serial.print(" r ");
         Serial.println(rspeed);*/
//    spause = 500;
//    digitalWrite(ledPin,HIGH);    
    delay(5);

// Disable Motors if requested
if (motsta == false){
	lspeed = 0;
	rspeed = 0; };

lspeed = lspeed * 175;
rspeed = rspeed * 175;

/*
 Serial.print(" - R MOT: ");
 Serial.print(rspeed);
 Serial.print(" L MOT: ");
 Serial.println(lspeed);
*/

 drive(lspeed,rspeed);

 
 Serial.println(" ");
}
