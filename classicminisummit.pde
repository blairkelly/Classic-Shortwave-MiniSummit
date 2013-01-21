// PPM Encoder for Arduino Controller
// Blair Kelly
//based on all sorts of libraries and other work. Full list to come.

///////////
//variables
///////////
//USB communication variables.
String usbInstructionDataString = "";
boolean USBcommandExecuted = true;
String usbCommand = "";
int usbCommandVal = 0;
unsigned long waitExpiry = millis();
int waitTime = 1111;
//headtracking
boolean headtracking = false;
unsigned long htExpiry = millis();
int htExpiryDelay = 5000;
//wheel
int wheelPWMctrDefault = 1500;
int wheelPWMmaxRange = 410;
int wheelPWMminRange = 20;
int wheelLow = wheelPWMctrDefault - wheelPWMmaxRange;
int wheelHigh = wheelPWMctrDefault + wheelPWMmaxRange;
int wheelPWMmin = 1085;  //fd 1085, default
int wheelPWMmax = 1890;  //fd 1890, default
int wheelPWMctr; //set below
//throttle
int thrPWMminDefault = 974;
int thrPWMmaxDefault = 1969;
int thrPWMctr = 1473;
int thrPWMmin = thrPWMminDefault;
int thrPWMmax = thrPWMmaxDefault;
//high/low gear
int highLow_high = 1927; //high gear, 1927, default.
int highLow_low = 1012; //low gear, 1012
//differential, front
int diffFront_unlocked = 1024; //unlocked, 1024, default
int diffFront_locked = 1940; //locked, 1940
//differential, back
int diffBack_unlocked = 1005;  //unlocked, 1005, default
int diffBack_locked = 1919;    //locked, 1919;
//camera
int camxPWMctrDefault = 1519;
int camxPWMctr = camxPWMctrDefault;
int camxPWMmin = 1159;
int camxPWMmax = 1879;
boolean autoCamera = false;
float autoCameraPercent;

//button pin definitions
int pinBtnRed = 10;
int pinSwitchThreeWayOne = 4;
int pinSwitchThreeWayTwo = 7;
int pinSwitchTwoWay = 12;

//button state info
int debounceDelay = 33; //milliseconds delay till state change.
boolean btnRedState = true; //this button is on by default.
boolean btnRedLastState = true;
boolean btnRedPressed = false;
unsigned long btnRedDBtime = millis();  //red button debounce expiry
//switchstate info
//switch one, two way
boolean switchOneLastState = false;
boolean switchOneState = false;
unsigned long switchOneDBtime = millis();
//switch two, three way
boolean switchTwoOneLastState = false;
boolean switchTwoOneState = false;
unsigned long switchTwoOneDBtime = millis();  
boolean switchTwoTwoLastState = false;
boolean switchTwoTwoState = false;
unsigned long switchTwoTwoDBtime = millis();


//pot pin definitions
int AI_PIN_WHEEL = A0;    // 
int AI_PIN_THROTTLE = A1;    // 
int AI_PIN_FPV_Y_TRIM = A2;    // 
int AI_PIN_STEERRANGE = A3;    //
int AI_PIN_FPV_X_TRIM = A4;    // 
int AI_PIN_STEER_TRIM = A5;    // 

int aiWHEEL;       //analogue-in reading
int aiTHROTTLE;    //analogue-in reading
int aiFPVYTRIM;    //analogue-in reading
int aiSTEERRANGE;   //analogue-in reading 
int aiFPVXTRIM;    //analogue-in reading
int aiSTEERTRIM;   //analogue-in reading 

int Wheel_uS = wheelPWMctrDefault;     // channel 1.  Ana In Ch.0 uS var - wheel
int Throttle_uS = thrPWMctr;    // channel 2 Ana In Ch.1 uS var - throttle
int highLow_uS = highLow_high;      // channel 3, high and low gear. default is 1927 (high gear).
int diffFront_uS = diffFront_unlocked;        // channel 4, front differential
int diffBack_uS = diffBack_unlocked;        // channel 5, rear differntial.
int CameraX_uS = camxPWMctrDefault;  //channel 6, cameraX
int CameraY_uS = camxPWMctrDefault;  //channel 8, cameraY

int Fixed_uS = 10;       // PPM frame fixed LOW phase. was 300. then 30.

int outPinPPM = 9;       // digital pin 10

boolean ledState = false;

ISR(TIMER1_COMPA_vect) {
  ppmoutput(); // Jump to ppmoutput subroutine
}

void setup() {
  Serial.begin(57600) ; // communicate.

  pinMode(pinBtnRed, INPUT); 
  pinMode(pinSwitchThreeWayOne, INPUT);
  pinMode(pinSwitchThreeWayTwo, INPUT);
  pinMode(pinSwitchTwoWay, INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  pinMode(outPinPPM, OUTPUT);   // sets the digital pin as output

  // Setup timer
  TCCR1A = B00110001; // Compare register B used in mode '3'
  TCCR1B = B00010010; // WGM13 and CS11 set to 1
  TCCR1C = B00000000; // All set to 0
  TIMSK1 = B00000010; // Interrupt on compare B
  TIFR1  = B00000010; // Interrupt on compare B
  OCR1A = 22000; // 22mS PPM output refresh
  OCR1B = 1000;
}

void ppmoutput() { // PPM output sub

  // Channel 1 - Wheel
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(Wheel_uS);   

  // Channel 2 - High/Low Gear 
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  //delayMicroseconds(highLow_uS);
  delayMicroseconds(Wheel_uS);  
  
  // Channel 3 - Differential, Front
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(diffFront_uS);
  
  // Channel 4 - Throttle 
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(Throttle_uS);
  
  // Channel 5 - Differential, Back
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(diffBack_uS);   

  // Channel 6 - cameraX
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(CameraX_uS);  //unused presently
  
  //// Channel 7 - 
  //digitalWrite(outPinPPM, LOW);
  //delayMicroseconds(Fixed_uS);    // Hold
  //digitalWrite(outPinPPM, HIGH);
  //delayMicroseconds(CameraX_uS); 
  
  // Channel 8 - cameraY
  //digitalWrite(outPinPPM, LOW);
  //delayMicroseconds(Fixed_uS);    // Hold
  //digitalWrite(outPinPPM, HIGH);
  //delayMicroseconds(CameraY_uS);

  // Synchro pulse
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);  // Start Synchro pulse

}


int mapToPWM(int guideReading, int guideCentre, int guideMin, int guideMax, int deadZoneWidth, int pwmCentre, int pwmMin, int pwmMax)
{
  int thePWM; //the PWM value that will be returned.
  float floatPWM;
  float theNumerator;
  float thePercentage;

  int negDZwidth = deadZoneWidth * -1;
  int ctrThresh = guideReading - guideCentre;
  if((ctrThresh < negDZwidth) && (guideReading > guideMin)) {
    float guideRangeBelowCtr = float(guideCentre - guideMin - deadZoneWidth);
    float pwmRangeBelowCtr = float(pwmCentre - pwmMin);
    theNumerator = guideRangeBelowCtr - float(guideReading - guideMin);
    thePercentage = theNumerator / guideRangeBelowCtr;
    floatPWM = float(pwmCentre) - (pwmRangeBelowCtr * thePercentage);
    thePWM = floatPWM;
  } 
  else if ((ctrThresh > deadZoneWidth) && (guideReading < guideMax)) {
    float guideRangeAboveCtr = float(guideMax - guideCentre - deadZoneWidth);
    float pwmRangeAboveCtr = float(pwmMax - pwmCentre);
    theNumerator = guideRangeAboveCtr - float(guideMax - guideReading);
    thePercentage = theNumerator / guideRangeAboveCtr;
    floatPWM = float(pwmCentre) + (pwmRangeAboveCtr * thePercentage);
    thePWM = floatPWM;
  } 
  else if (guideReading <= guideMin) {
    thePWM = pwmMin;
  } 
  else if (guideReading >= guideMax) {
    thePWM = pwmMax;
  } 
  else {
    thePWM = pwmCentre;
  }
  //Serial.println(thePWM);
  //delay(111);
  return(thePWM);
}

int trimPotPWMadj(int potReading, int targetPWMctr, int targetPWMmin, int targetPWMmax, float leeway)
{
  //leeway is just how far you want to allow the center to be adjusted depending on the allowed ranges.
  int thePWM; //the PWM value that will be returned.
  float floatPWM;
  float theNumerator;
  float thePercentage;

  float tPWMctr = (float)targetPWMctr;
  float tPWMmin = targetPWMctr - (float(targetPWMctr - targetPWMmin) * leeway);
  float tPWMmax = targetPWMctr + (float(targetPWMmax - targetPWMctr) * leeway);

  if((potReading <= 511) && (potReading > 0)) {
    float pwmRangeBelowCtr = tPWMctr - tPWMmin;
    theNumerator = 511.0 - float(potReading);
    thePercentage = theNumerator / 511.0;
    floatPWM = tPWMctr - (pwmRangeBelowCtr * thePercentage);
    thePWM = floatPWM;
  } 
  else if ((potReading > 511) && (potReading < 1023)) {
    float pwmRangeAboveCtr = tPWMmax - tPWMctr;
    theNumerator = float(potReading) - 512.0;
    thePercentage = theNumerator / 512.0;
    floatPWM = tPWMctr + (pwmRangeAboveCtr * thePercentage);
    thePWM = floatPWM;
  } 
  else if (potReading <= 0) {
    thePWM = tPWMmin;
  } 
  else if (potReading >= 1023) {
    thePWM = tPWMmax;
  } 
  else {
    thePWM = tPWMctr;
  }
  //Serial.println(thePWM);
  //delay(111);
  return thePWM;
}

void steerRangePotPWMadj(int potReading)
{
  float thePercentage = float(potReading) / 1023.0;
  float theRange = (thePercentage * float(wheelPWMmaxRange - wheelPWMminRange)) + wheelPWMminRange;
  
  wheelLow = wheelPWMctr - theRange;
  wheelHigh = wheelPWMctr + theRange;
  
  if(wheelLow < wheelPWMmin) {
    wheelLow = wheelPWMmin;
  }
  if(wheelHigh > wheelPWMmax) {
    wheelHigh = wheelPWMmax;
  }
  
  //Serial.println(wheelPWMmin);
  //delay(222);
}

void blinkit() {
  digitalWrite(13, HIGH);
  delay(33);
  digitalWrite(13, LOW);
  delay(33);
}

void delegate(String theCommand, int theCommandVal) {
  if (theCommand.equals("-") && !autoCamera) {
      //arduino giving pan value of headset.
      CameraX_uS = theCommandVal; //equate goggles pan uS
      headtracking = true;
      htExpiry = millis() + htExpiryDelay;
  } else if (theCommand.equals("+") && !autoCamera) {
      CameraY_uS = theCommandVal;
  }
  if(!autoCamera) {
    Serial.print("K");   //tell the other arduino it may send instructions again.
  }
  waitExpiry = millis() + waitTime;
}

void serialListen()
{
  char arduinoSerialData; //FOR CONVERTING BYTE TO CHAR. here is stored information coming from the arduino.
  String currentChar = "";
  if(Serial.available() > 0) {
    arduinoSerialData = char(Serial.read());   //BYTE TO CHAR.
    currentChar = (String)arduinoSerialData; //incoming data equated to c.
    if(!currentChar.equals("1") && !currentChar.equals("2") && !currentChar.equals("3") && !currentChar.equals("4") && !currentChar.equals("5") && !currentChar.equals("6") && !currentChar.equals("7") && !currentChar.equals("8") && !currentChar.equals("9") && !currentChar.equals("0") && !currentChar.equals(".")) { 
      //the character is not a number, not a value to go along with a command,
      //so it is probably a command.
      if(!usbInstructionDataString.equals("")) {
        //usbCommandVal = Integer.parseInt(usbInstructionDataString);
        char charBuf[30];
        usbInstructionDataString.toCharArray(charBuf, 30);
        usbCommandVal = atoi(charBuf);
      }
      if((USBcommandExecuted == false) && (arduinoSerialData == 13)) {
        delegate(usbCommand, usbCommandVal);
        USBcommandExecuted = true;
        //blinkit();
      }
      if((arduinoSerialData != 13) && (arduinoSerialData != 10)) {
        usbCommand = currentChar;
      }
      usbInstructionDataString = "";
    } else {
      //in this case, we're probably receiving a command value.
      //store it
      usbInstructionDataString = usbInstructionDataString + currentChar;
      USBcommandExecuted = false;
    }
  }
  
  if ((millis() > waitExpiry) && !autoCamera) {
    //give the other arduino a boot.
    Serial.print("K");   //tell the other arduino it may send instructions again.
    waitExpiry = millis() + waitTime;
  }
  
}

void doAutoCamera()
{
  if(autoCamera) {
    //theTime = millis();
    //if(theTime > cMMtimeout) {
      int steeringRange; //depends on a few factors
      int autoCameraRange; //also depends on a few factors.
      float cameraPoint = 0.0;
      int cameraPointSet = 0;
      float percentToMove = 0.0;
      if(Wheel_uS < wheelPWMctr) {
        //turning left
        steeringRange = wheelPWMctr - wheelPWMmin;
        percentToMove = ((float)wheelPWMctr - (float)Wheel_uS) / (float)steeringRange;
        percentToMove = pow(percentToMove,2);
        autoCameraRange = autoCameraPercent * (camxPWMmax - camxPWMctr);
        cameraPoint = (float)camxPWMctr + ((float)autoCameraRange * percentToMove);
      } else if (Wheel_uS > wheelPWMctr) {
        steeringRange = wheelPWMmax - wheelPWMctr;
        percentToMove = ((float)Wheel_uS - (float)wheelPWMctr) / (float)steeringRange;
        percentToMove = pow(percentToMove,2);
        autoCameraRange = autoCameraPercent * (camxPWMctr - camxPWMmin);
        cameraPoint = (float)camxPWMctr - ((float)autoCameraRange * percentToMove);
      } else {
        //camera centred.
        cameraPoint = (float)camxPWMctr;
      }
      cameraPointSet = (int)cameraPoint; //float to int.
      //Serial.print("cameraPointSet: ");
      //Serial.println(cameraPointSet);
      //delay(111);
      CameraX_uS = cameraPointSet;
    //}
  }
}
void handleBtnRed()
{
  int reading = digitalRead(pinBtnRed);
  if(reading != btnRedLastState) {
    btnRedDBtime = millis();
  } 
  if((millis() - btnRedDBtime) > debounceDelay) {
    btnRedState = reading;
  }
  if((btnRedState == LOW) && !btnRedPressed) {
    //the state has changed... button has been pushed
    if(autoCamera) {
      Serial.println("autoCamera OFF");
      autoCamera = false;
      //digitalWrite(13, LOW);
    } else {
      Serial.println("autoCamera ON");
      autoCamera = true;
      //digitalWrite(13, HIGH);
    }
    btnRedPressed = true;
  } else if ((btnRedState == HIGH) && btnRedPressed) {
    btnRedPressed = false;
  }
  btnRedLastState = reading;
  //delay(33);
}

void handleSwitchOne()
{
 int reading = digitalRead(pinSwitchTwoWay);
 if(reading != switchOneLastState) {
    switchOneDBtime = millis();
 }
 if((millis() - switchOneDBtime) > debounceDelay) {
    switchOneState = reading;
 }
 if(switchOneState) {
   //set to low gear.
   highLow_uS = highLow_low;
   //digitalWrite(13, HIGH);
 } else {
   //set default, high gear
   highLow_uS = highLow_high;
   //digitalWrite(13, LOW);
 }
 switchOneLastState = reading;
}
void handleSwitchTwo()
{
 int reading1 = digitalRead(pinSwitchThreeWayOne);
 if(reading1 != switchTwoOneLastState) {
    switchTwoOneDBtime = millis();
 }
 if((millis() - switchTwoOneDBtime) > debounceDelay) {
    switchTwoOneState = reading1;
 }
 if(switchTwoOneState) {
   //unlock both diffs
   diffFront_uS = diffFront_unlocked;
   diffBack_uS = diffBack_unlocked;
   //digitalWrite(13, LOW);
 } else {
   //do nothing
 }
 switchTwoOneLastState = reading1;
 
 int reading2 = digitalRead(pinSwitchThreeWayTwo);
 if(reading2 != switchTwoTwoLastState) {
    switchTwoTwoDBtime = millis();
 }
 if((millis() - switchTwoTwoDBtime) > debounceDelay) {
    switchTwoTwoState = reading2;
 }
 if(switchTwoTwoState) {
   //lock both diffs
   diffFront_uS = diffFront_locked;
   diffBack_uS = diffBack_locked;
   //digitalWrite(13, HIGH);
 } else {
   //check if the other switch is off. if so, lock front diffs, unlock back diffs
   if(!switchTwoOneState) {
     diffFront_uS = diffFront_locked;
     diffBack_uS = diffBack_unlocked;
     //blinkit();
   }
 }
 switchTwoTwoLastState = reading2;
}

void loop() 
{ // Main loop
  serialListen();
  
  //read buttons
  handleBtnRed(); //current function: turns on/off autoCamera
  handleSwitchOne(); //position, front right. current function: high/low gear.
  handleSwitchTwo(); //position, front left. current function: Differentials. (Front and Back).
  
  // Read analogue ports
  aiWHEEL = analogRead(AI_PIN_WHEEL);
  aiTHROTTLE = analogRead(AI_PIN_THROTTLE);
  aiFPVYTRIM = analogRead(AI_PIN_FPV_Y_TRIM);
  aiSTEERRANGE = analogRead(AI_PIN_STEERRANGE);
  aiFPVXTRIM = analogRead(AI_PIN_FPV_X_TRIM);
  aiSTEERTRIM = analogRead(AI_PIN_STEER_TRIM);
  
  //adjust pot readings
  //trimPotPWMadj(potReading, targetPWMctr, targetPWMmin, targetPWMmax, leeway) :: leeway is range you'll allow the center point to be adjusted depending on the mins and maxs provided.
  wheelPWMctr = trimPotPWMadj(aiSTEERTRIM, wheelPWMctrDefault, wheelPWMmin, wheelPWMmax, 0.50);
  if(!headtracking && (millis() > htExpiry)) {
    camxPWMctr = trimPotPWMadj(aiFPVXTRIM, camxPWMctrDefault, camxPWMmax, camxPWMmin, 1.0);
    //Serial.println(camxPWMctr);
    //delay(100);
    headtracking = false;
  }
  //adjust max speed depending on throttle speed pot.
  steerRangePotPWMadj(aiSTEERRANGE);
  //adjust the maximum travel of the autocamera
  autoCameraPercent = (float)aiFPVYTRIM / (float)1023;

  //mapToPWM(guideReading, guideCentre, guideMin, guideMax, deadZoneWidth, pwmCentre, pwmMin, pwmMax);
  Wheel_uS = mapToPWM(aiWHEEL, 506, 392, 623, 6, wheelPWMctr, wheelLow, wheelHigh);
  if(autoCamera) {
      doAutoCamera();
  } else if (!headtracking) {
      CameraX_uS = camxPWMctr;
  }
  //chan2 is taken by a copy of wheelUS
  Throttle_uS = mapToPWM(aiTHROTTLE, 448, 355, 653, 3, thrPWMctr, thrPWMmin, thrPWMmax);
  //chan5_uS = 1500; //currently unused.
  //CameraX_uS = mapToPWM(guideReading, guideCentre, guideMin, guideMax, deadZoneWidth, pwmCentre, pwmMin, pwmMax);  //camera X
  //chan6_uS = 1500;  //currently unused
  
  //float wicky = (autoCameraPercent * 100.0) + 0.00;
  //Fixed_uS = wicky;
  //Serial.println(Throttle_uS);
  //delay(333);
}
