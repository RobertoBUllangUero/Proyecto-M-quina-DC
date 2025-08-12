//Version 9
//compatible con matlab


//Creamos un tipo llamado union para convertir facilmente de  float a byte (Se usa sobre el mismo registro)
typedef union{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

//Declaracion de pines
const int PINPWM = 10; //Pin de salida PWM (D10)
const int buttonPin = 2;   // Pin digital 2 desginado para contar pulsos


//Variables holders para enviar datos
FLOATUNION_t speed;

volatile int pulseCount = 0; //Cuenta los pulsos (holder)
volatile unsigned long D = 0; //Variable de almacenamiento deprecated (usada para ver cuantas veces se lee el ADC por cada actualizacion de velocidad)

//Holder para recibir datos
float in;
int dutycicle = 0;
/*
Dado que el ADC es de 10 bits es nescesario
dividir en dos variables de 8 bits y concaternalos
externamente
*/
volatile uint8_t h1; //Parte "baja" del registro del ADC
volatile uint8_t h2; //Parte "alta" del regstro del ADC

FLOATUNION_t H1; //Parte "baja" del registro del ADC (guardada en tipo floatunion)
FLOATUNION_t H2; //Parte "alta" del regstro del ADC (guardada en tipo floatunion)

unsigned long currentMillis = millis();


// Variables del timer for pulse counting and resetting
unsigned long previousMillis = 0;
const unsigned long interval = 30; // 0.01 segundo

// Variables for the ADC reading timer
unsigned long adcPreviousMillis = 0;
const unsigned long adcInterval = 3; // 0.001 seconds



void setup() {
  analogWrite(PINPWM, 0);
  delay(3000); // Se realiza una espera de 3 segundos para asegurar recuperación de placa
  ADMUX = 0b01000000; // Voltaje de ref Vref externo, idealmente se debe estabilizar, resultado a la derecha, usamos el ADC0.
  ADCSRA = 0b11000011;
  Serial.begin(115200);
  DDRC &= ~_BV(PD0); // A0 como entrada analógica
  PORTC &= ~_BV(PD0); // A0 pulldown
  pinMode(buttonPin, INPUT_PULLUP); // D2 con pullup
  pinMode(PINPWM, OUTPUT); //Pin de salida PWM
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonInterrupt, FALLING);
  TCCR1B = TCCR1B & 0b11111000 | 0x01;//Timer 1 con division de timer al minimo /1
  //TCCR1B = TCCR1B & B11111000 | B00000101 ; //Ajuste frecuencia PWM aprox 31.59KHz

  // Initialize ADC reading timer
  // Set Timer2 prescaler to 64 for 0.01-second intervals
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  TCCR2B |= (1 << CS22) | (1 << CS21);
  TIMSK2 |= (1 << TOIE2);// Enable Timer2 overflow interrupt
}

void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    //Serial.print(" D: ");
    //Serial.println(D);
    //D = 0;
    speed.number = pulseCount;
    pulseCount = 0;
    previousMillis = currentMillis;
  }
  if (Serial.available() > 0){
      in = ReadFromSerial();
  }
}

void buttonInterrupt() {
  pulseCount++;
}


ISR(TIMER2_OVF_vect) {
  //Esta funcion deberia llamarse cada 0.01 segundos
  //Lectura del ADC

  ADCSRA |= _BV(ADSC);
  while (bit_is_clear(ADCSRA, ADIF)) {
  }
  h1 = ADCL;
  h2 = ADCH & 0b00000011;
  H1.number = h1;
  H2.number = h2;
  //Actualizamos el ciclo de trabajo de la salida analogica
  dutycicle = map(in,0,255,0,255);
  if (dutycicle > 100) {
  digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
  digitalWrite(LED_BUILTIN, LOW);
  }
  analogWrite(PINPWM, dutycicle);
  //Enviamos mediante serial los bits
  Serial.write("U");
  writeToSerial(H2);
  writeToSerial(H1);
  writeToSerial(speed);
  Serial.write("\n"); //Salto de linea (terminador)
  //D++;
}


//Funcion para imprimir FLOATUNION_t en el puerto serial
void writeToSerial(FLOATUNION_t data) {
  for (int i = 0; i < 4; i++) {
    Serial.write(data.bytes[i]);
  }
}

//Funcion para leer un FLOATUNION_t en el puerto serial
float ReadFromSerial() {
  FLOATUNION_t buf; // Creamos un buffer
  char receivedChar;
  int i = 0;

  while (true) {
    while (Serial.available() > 0) {
      receivedChar = Serial.read();
      if (receivedChar == '\n') {
        return buf.number;
      }
      buf.bytes[i] = receivedChar;
      i++;
      if (i >= 4) {
        break; // Evitar desbordamiento del buffer
      }
    }
  }
}

void clearSerialBuffer() {
  while (Serial.available() > 0) {
    char c = Serial.read(); // Leer y descartar los datos del puerto serie
  }
}