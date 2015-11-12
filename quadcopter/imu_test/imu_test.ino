int sensorPin = A1;
int sensorValue = 0;
int prev_sensorValue = 0;
int i = 0;
int encoder_count = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorValue = analogRead(sensorPin);
  Serial.print("Sensor value: ");
  Serial.print(sensorValue);
  Serial.print("\t");
  Serial.print("i: ");
  Serial.print(i);
  Serial.print("\t");
  Serial.print("Encoder count: ");
  Serial.println(encoder_count);
  i++;
  if (i > 100 && i < 200){
    if (prev_sensorValue < 2 && sensorValue > 2){
      encoder_count++;
    }
  }
  prev_sensorValue = sensorValue;    
}
