
#define IN1 27
#define IN2 22
#define IN3 26
#define IN4 28
#define ENA 15
#define ENB 14
#define ENCODERRIGHT_PIN_A 0
#define ENCODERRIGHT_PIN_B 1
#define ENCODERLEFT_PIN_A 2
#define ENCODERLEFT_PIN_B 3

// Contador para os encoders
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;

// Variáveis para o encoder
byte Encoder_Left_Last;
byte Encoder_Right_Last;
int duration;
boolean Direction;

void setup() {
  Serial.begin(57600);

  // Definição dos pinos do motor e encoder
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Inicialização do encoder
  EncoderInit();
}

void loop() {
  int valor = analogRead(A0);
  if (valor >= 512) {
    // Motor 1
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    // Motor 2
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    Serial.println(" Sentido: Anti-horario");
  } else {
    // Motor 1
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    // Motor 2
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    Serial.println(" Sentido: Horario");
  }
  
  delay(100);
}

void EncoderInit() {
  pinMode(ENCODERLEFT_PIN_A, INPUT);
  pinMode(ENCODERRIGHT_PIN_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODERLEFT_PIN_A), calculatePulseLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODERRIGHT_PIN_A), calculatePulseRight, CHANGE);
}

void calculatePulseLeft() {
  // Determine the direction by comparing current and last state of encoder pins
  int state = digitalRead(ENCODERLEFT_PIN_A);
  int stateB = digitalRead(ENCODERLEFT_PIN_B);
  
  if ((Encoder_Left_Last == LOW) && (state == HIGH)) {
    // The A phase has gone from low to high
    if (stateB == LOW) {
      encoderLeftCount++; // Moving forward
    } else {
      encoderLeftCount--; // Moving backward
    }
  }
  Encoder_Left_Last = state; // Update last state
}

void calculatePulseRight() {
  // Determine the direction by comparing current and last state of encoder pins
  int state = digitalRead(ENCODERRIGHT_PIN_A);
  int stateB = digitalRead(ENCODERRIGHT_PIN_B);
  
  if ((Encoder_Right_Last == LOW) && (state == HIGH)) {
    // The A phase has gone from low to high
    if (stateB == LOW) {
      encoderRightCount++; // Moving forward
    } else {
      encoderRightCount--; // Moving backward
    }
  }
  Encoder_Right_Last = state; // Update last state
}
