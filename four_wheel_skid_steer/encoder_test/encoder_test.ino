

// Encoder
const int encoder = 2;    // interrupt 0   
double pulse = 0;
double time = 0;

#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t))
// in terms of ms

void setup() {
  Serial.begin(9600); //to use the serial monitor  
  digitalWrite(encoder,1);                              
  attachInterrupt(digitalPinToInterrupt(encoder),UpdateState,CHANGE);                    
}

void loop() {

  runEvery(500)
  {
    /*
    Serial.print("pulse = ");
    Serial.print(pulse);
    Serial.print("\t");
    Serial.print("time = ");
    Serial.println(time);
    */
    Serial.print("t\n");
  }

}

// ISR 
void UpdateState()
{
  pulse=millis()-time;                               
  time=millis(); 
}

