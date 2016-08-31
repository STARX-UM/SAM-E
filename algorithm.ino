/* Algorithm 6: 
5th algorithm option:
-EMG given preference.
-EMG uses moving average.
-Tests for bicep and tricep activation, if one is significantly more, it powers 
  that linearly with EMG signal.
-If EMG signal is below noise floor, uses a PI controller with inputs
  from two force sensors
-EMG threshold and max EMG are scaled by test phase when reset.
*/

const int AVERAGEINTERVAL = 4; //how many EMG samples to average over
const int A_MAX = 855;   //maximum of angle sensor
const int A_MIN = 250;  //minimum, should be less by ~585
const int athr = 30;  //when it starts restricting torque
static int mai = 0;  //moving average iterator
static int bs[AVERAGEINTERVAL] = {0}, ts[AVERAGEINTERVAL] = {0}; //globals for average
static bool lastDir = 0; //keeps track of direction
static float bScale = 1;
static float tScale = 1;
static int tFloating = 0;
static int bFloating = 0;
static float forceIntegral = 0;  //for PID controller
static int prevError = 0;  //for PID controller
static bool PIDmode = 0;   //1 if PID is active

//pins
const int relayPin = 2;
const int dirPin1 = 21;
const int dirPin2 = 22;
const int DACPin = A14;
const int potPin = A2;
const int tricepPin = A3;
const int bicepPin = A4;
const int forcePin1 = A0;
const int forcePin2 = A1;

int PIDcontroller(int angle)
{
  //get data
  int b = analogRead(forcePin2);
  int t = analogRead(forcePin1);
  int tthr = 500;
  int bthr = 500;
  if (t < tthr)
    t = 0;
  else //enough activation
    t -= tthr;
  if (b < bthr)
    b = 0;
  else //enough
    b -= bthr;
  //Serial.print(b); Serial.print("\t");  Serial.println(t);
  
  // PID
  int dScale;
  int pScale = 10;
  float iScale = .001;
  if (!PIDmode)
  {
    digitalWrite(13,HIGH);
    dScale = 0;
    forceIntegral = 0;  //reset derivative and integral on first time
    PIDmode = 1;
  }
  else
    dScale = 2;

  int error = b-t;
  //the thing we want to control. Outputs between -1024 and 1024.
  //Serial.println(error);
  
  if(!(angle > (A_MAX - athr)) && !(angle < (A_MIN+athr)))  //doesn't update when near edge
    forceIntegral += ((float)error); //update integral.
  else
    forceIntegral = 0;   //no input from integral when near edge

  int derivative = error - prevError;
  
  prevError = error; //update

  int torque = pScale*error + iScale*forceIntegral + dScale*derivative; //PID controller
  
  return torque;
}



void setup()
{
  Serial.begin(115200);
  pinMode(relayPin, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  digitalWrite(dirPin1, lastDir);
  digitalWrite(dirPin2, !lastDir); //initialize it
  //pinMode(DACPin, OUTPUT);
  analogWriteResolution(10);
  pinMode(13, OUTPUT); //teensy indicator LED

  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, LOW);
  digitalWrite(relayPin, LOW);
  analogWrite(DACPin, 0);

  bool doCalibration = (bool)0; //so we can test without it if we want, default scale is 1
  if (doCalibration)
  {
    /* Finds the maximum bicep value over 3 seconds of flexing. Edits the globals bScale
     * and tScale. Later the algorithm multiplies by bScale to get 0-1024 output.
     * Sets floating zero values that will be subtracted off of EMG later.
     */
    delay(2000);  //wait 2 seconds
    digitalWrite(13, HIGH);  //LED on for bicep flex
    int counter = 0;
    int value = 0;
  
    int Bmaximum = 0;
    int Tmaximum = 0;
  
    //Finding maximum Bicep value over 5 second period
    while(counter < 3000)
    {
      value = analogRead(bicepPin);
      if(value > Bmaximum) 
        Bmaximum = value;
      ++counter;
      delay(1);
    }
    bScale = 1024.0 / Bmaximum;
    
    value = 0;
    counter = 0;
    digitalWrite(13, LOW);
    delay(1000);            //Short break between Bi/Tri readings
    digitalWrite(13, HIGH);
  
    //Finding maximum Tricep value over 5 second period
    while(counter < 3000)
    {
      value = analogRead(tricepPin);
      if(value > Tmaximum)
        Tmaximum = value;
      ++counter;
      delay(1);
    }
    
    tScale = 1024.0/Tmaximum;
    
    digitalWrite(13, LOW);
    delay(1000); //wait for a second after tricep

    //calibrate no-load floating values of the emgs
    /* space starts at zero. Here the program checks whether the EMGs have outputted the
     * same value for 500 ms. If it finds this value within 2 seconds, it sets bFloating
     * and tFloating to that value. If it doesn't, it increments space. Then it looks for
     * the value to be within space of what the last one was. 
     */
     /*
    counter = 0;
    value = 0;
    int longcounter = 0;
    int space = 5;
    //bool bFinished = 0;
    //bool tFinished = 0;
    while (counter < 500)
    {
      value = analogRead(bicepPin);
      if (value <= bFloating-space || value >= bFloating+space) //if it's outside range
      {
        counter = 0;  //reset counter
        bFloating = value;  //this is the new b value we're looking for
      }
      value = analogRead(tricepPin);
      if (value >= tFloating-space && value <= tFloating+space)
      {
        counter = 0;  //reset counter
        tFloating = value;  //this is the new t value we're looking for
      }
      ++counter;
      ++longcounter;
      space = longcounter % 1000; //increases by 1 every second
      delay(1);
    } //success we did it we found a stable value  */
    counter = 0;
    double valuebic = 0;
    double valuetri = 0;
    while (counter < 1000)
    {
      valuebic += analogRead(bicepPin);
      valuetri += analogRead(tricepPin);
      delay(1);
      counter += 1;
    }
    bFloating = valuebic / 1000;
    tFloating = valuetri / 1000;
    Serial.print(bFloating);
    Serial.print("\t");
    Serial.println(tFloating);
    digitalWrite(13,HIGH);
  }  //end calibration
  digitalWrite(13, HIGH);
  delay(3000);
  digitalWrite(13, LOW);
}

void loop()
{
  /*********************************** GET DATA *****************************\
   *  reads in values froms sensors, scales from calibration,
   *  then takes average over several cycles.
   *  angle is angle, b is bicep, t is tricep, all have values 0-1024
   */
  int angle, b, t, torque, dir;
  
  //replace oldest array entries with new (scaled) values
  bs[mai] = (int)analogRead(bicepPin)*bScale;
  ts[mai] = (int)analogRead(tricepPin)*tScale;
  if (bs[mai] > bFloating) //subtract off the floating values
    bs[mai] -= bFloating;
  if (ts[mai] > tFloating)
    ts[mai] -= tFloating;

  //Serial.print(bs[mai]);
  //Serial.print("\t");
  //Serial.println(ts[mai]);
  
  if (bs[mai] > 1024) //make sure it doesn't go above 1024
    bs[mai] = 1024;
  if (ts[mai] > 1024)
    ts[mai] = 1024;
  
  angle = analogRead(potPin);  // angle is not an array

  mai +=1;  //increment moving average iterator
  if (mai == AVERAGEINTERVAL)
    mai = 0;  //don't let it go out of bounds

  b = 0; t = 0;
  for (int j = 0; j < AVERAGEINTERVAL; j++)  //take average of arrays to find b and t
  {
    b += bs[j];
    t += ts[j];
  }
  b /= AVERAGEINTERVAL; //integer divides!!
  t /= AVERAGEINTERVAL; //but it's ok since b >> AVERAGEINTERVAL
  // end of get data

  //Serial.print(b);
  //Serial.print("\t");
  //Serial.println(t); 
  //Serial.println(angle);

  /******************************** Calculate Torque ****************************\
   *  If b and t are above their threshold values, and a scale factor more than the
   *  other one, then motor torque is linear with EMG activation. If they're below,
   *  it uses the PIController(). Outputs torque between 0 and 1024 and bool dir.
   *  dir is 0 for tricep and 1 for bicep
   */
   int Bthr = 2000, Tthr = 2000; //thresholds, TODO: get this from calibration
   float sVal = 1.5;  //scale value, determines whether B and T are significantly far
   if (t > Tthr && t > sVal*b) //tricep activation
   {
      torque = t;
      dir = 0;
      Serial.println("Tricep!");
      digitalWrite(13, LOW);
   }
   else if (b > Bthr && b > sVal*t) //bicep activation
   {
      torque = b;
      dir = 1;
      Serial.println("Bicep!");
      digitalWrite(13, LOW);
   }
   else  //neither activated
   {
      //Serial.print(b);
      //Serial.print(" PID ");
      //Serial.println(t);
      //Serial.println("PID");
      
      torque = PIDcontroller(angle);
      if (torque < 0)  //this because PIcontroller returns + or - for directions
      {
        torque = -torque;
        dir = 0;
      }
      else
      {
        dir = 1;
      }
      Serial.println(torque);
   }
   /******************************** Sanity Check ****************************\
   *  Makes sure torque can't go above 1024. Scales down torque near the max 
   *  and minimum angle.
   *  If the angle is above or below certain values (A_MIN+athr and A_MAX-athr)
   *  and torque is in the direction that would push it over the limits,
   *  it scales down torque by the factor (deltaangle/athr), where deltaangle is
   *  the difference between current angle and the limit. That makes it decrease
   *  linearly until it reaches 0 at the maximum and minimum angles.
   */
   if (torque > 1024)  //you know just to be safe
      torque = 1024;
   if (torque < 0)
      torque = 0;
   
   if (angle <= (A_MIN + athr) && !dir) //if a is below lo threshold and dir is 0 (tricep)
   {
      torque = (torque * (angle-A_MIN))/athr;
      Serial.println("too far in tri dir");
   }
   if (angle >= (A_MAX - athr) && dir) //if a is above hi threshold and dir is 1 (bicep)
   {
      torque = (torque * (A_MAX-angle))/athr;
      Serial.println("too far in bi dir");
   }

   /******************************** Motor Control ****************************\
   *  Engages relay if torque is not zero. If direction is not correct, switches it.
   *  Scales output to 2.2V max.
   */
   if (torque <= 0)   //if zero torque
   {
      analogWrite(DACPin, 0);
      digitalWrite(relayPin, LOW);  
      delay(10);
      digitalWrite(dirPin1, LOW);
      digitalWrite(dirPin2, LOW);
   }
   else  //if torque not zero
   {
      if (dir != lastDir) //if we need to switch directions
      {
        Serial.print("switch to ");
        Serial.println(dir);
        digitalWrite(relayPin, LOW);
        analogWrite(DACPin, 0);
        delay(10); //wait to make sure it's stopped
        lastDir = dir;
        digitalWrite(dirPin1, dir);  //send correct dir to dir pins
        digitalWrite(dirPin2, !dir);
      }
      else
      {
          digitalWrite(dirPin1, dir);
          digitalWrite(dirPin2, !dir);
      }
      digitalWrite(relayPin, HIGH); //turn on relay
      torque = torque/3;  //scale for max 2.2V
      analogWrite(DACPin, torque);
      //Serial.print(dir);  Serial.print("\t");   Serial.println(torque);
      //delay(10);
   }
   /**********************************************************************/
}
