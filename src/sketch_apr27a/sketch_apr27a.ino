int trig = 9; 
int echo = 8;
int duration;
float distance;
float meter;

void setup(){
  Serial.begin(9600);
  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  pinMode(echo, INPUT); // Corrigido de IMPUT para INPUT
  delay(6000);
  Serial.println("Distance:"); 
}

void loop(){
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  duration = pulseIn(echo, HIGH); // Usando time_pulse_us() para obter a duração do pulso

  if(duration>=38000){
    Serial.print("Out range"); // Adicionado ponto e vírgula no final da declaração
  }
  else{
    distance=duration/58;
    Serial.print(distance);
    Serial.print("cm");
    meter=distance/100;
    Serial.print("\t");
    Serial.print(meter);
    Serial.println("m");
  }
  delay(1000);
}

