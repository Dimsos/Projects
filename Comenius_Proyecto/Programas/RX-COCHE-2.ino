//  PROYECTO DE GRADO - COCHE CONTROLADO A CONTROL REMOTO POR GESTOS DE GIROSCOPIO + PAN&TILT CON SENSOR ULTRASONIDOS
//
//  PROGRAMA RECEPTOR - COCHE, PAN&TILT, ULTRASONIDOS, BUZZER, ANTENA NRF24L01+MÓDULO

// ---- Incluimos las librerías del SPI, Antena, Servos (del Pan&Tilt)
#include "SPI.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "Servo.h"

// ---- Variables L298N
const int pinENA = 5;                   //Pin de velocidad, motor derecho
const int pinENB = 6;                   //Pin de velocidad, motor izquierdo
const int pinIN1 = 3;                   //Pines Motor Derecho
const int pinIN2 = 4;
const int pinIN3 = 7;                   //Pines Motor Izquierdo
const int pinIN4 = 8;

int speed = 0;                          //Inicializamos el valor de los motores como 0

// ---- Variables NRF24L01
RF24 radio(9, 10);                      //Objeto llamado "radio". Pines del SPI, correspondientes a CE y CSN

const byte address[6] = "00001";        //Matriz de bytes identificativo. Dirección que utilizarán las antenas para identificarse entre sí

float MPU[3];                           //Este valor corresponde a los datos X y Y del giroscopio que llegarán por la antena

// ---- Variables Pan&Tilt
Servo ServoX;
Servo ServoY;

// ---- Variables Ultrasonido
const int TRIG_PIN   = 33;              // Pin de Arduino del TRIGGER pin
const int ECHO_PIN   = 32;              // Pin de Arduino del ECHO pin
const int BUZZER_PIN = 30;              // Pin de Arduino para el Buzzer
const float DISTANCE1 = 20.0;           // Centimetros
const float DISTANCE2 = 10.0;

float duration_us, distance_cm;         //Inicializamos los valores de distancia y tiempo del ultrasonido.

/* Este bloque es para poder hacer los contadores o cronómetros
   para los triggers sin necesidad de usar Delay(). Hay uno para
   el Trigger del ultrasonido y otro para el Buzzer.*/
unsigned long TTrig1, TTrig2, TBuz1, TBuz2 ;
const unsigned long TTrig = 10;
const unsigned long TBuz = 10;

void setup() {
  Serial.begin(115200);                 //Inicializamos comunicación serie a 115200 baudios

  /* Inicializamos el objeto "radio". Abrimos el canal de recepción
      de datos identificado por la dirección dada anteriormente. Utilizamos el canal 0.
      Si se estan haciendo pruebas sobre mesa o una antena al lado de otra,
      radio.setPALevel nos permitirá mantener la potencia al mínimo.
      radio.startListening establece a esta antena como receptor. */
  radio.begin();
  radio.openReadingPipe(0, address);
  //radio.setPALevel(RF24_PA_MIN);
  radio.startListening();


  pinMode(pinIN1, OUTPUT); //Los pines correspondientes a los motores serán pines de salida
  pinMode(pinIN2, OUTPUT);
  pinMode(pinENA, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinENB, OUTPUT);

  /* Pines de los servos del Pan&Tilt
     el valor '90' lo utilizamos para establecer los servos en su posición media (0-180º)
     Esperamos 1.5 segundos para permitir que ocurra este proceso antes de que comienza a
     detectar los valores del giroscopio  */
  ServoX.attach(36);
  ServoY.attach(37);

  /* Pines del ultrasonido y el del buzzer
     Incluidos la inicialización del Millis() e ir midiendo el tiempo
     Dejamos un delay al final para que tenga tiempo de realizar este proceso */
  
  pinMode(TRIG_PIN, OUTPUT);   // set arduino pin to output mode
  pinMode(ECHO_PIN, INPUT);    // set arduino pin to input mode
  pinMode(BUZZER_PIN, OUTPUT); // set arduino pin to output mode
  TTrig1 = millis();
  TBuz1 = millis();

  delay(1500);
}

void loop() {
  
  
  if (radio.available()) {
    radio.read(MPU, sizeof(MPU));

    Serial.print(MPU[0]);
    Serial.print(" ");
    Serial.print(MPU[1]);
    Serial.print(" ");
    Serial.print(MPU[2]);

    Serial.print(" - - - ");
    Serial.println(speed);

    if (MPU[2] == 1.0) {
      analogWrite(pinENA, 0);
      analogWrite(pinENB, 0);
      panTilt();

    }
    else {
      Motores();
      ServoX.write(90);
      ServoY.write(90);

    }

  }
}

void Motores() {

  MPU[0] = map(MPU[0], 90, -90, -255, 255);
  MPU[1] = map(MPU[1], 90, -90, -255, 255);
  ultraBuzz();


  if (MPU[0] > 100) {           //FORWARD
    speed = MPU[0];
    analogWrite(pinENA, speed);
    analogWrite(pinENB, speed);
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
    digitalWrite(pinIN3, LOW);
    digitalWrite(pinIN4, HIGH);
  }

  if (MPU[0] < -100) {           //BACKWARDS
    speed = -1 * MPU[0];
    analogWrite(pinENA, speed);
    analogWrite(pinENB, speed);
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
    digitalWrite(pinIN3, HIGH);
    digitalWrite(pinIN4, LOW);
  }

  if (MPU[1] < -100) {
    speed = -1 * MPU[1];
    analogWrite(pinENA, speed);
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
  }

 if (MPU[1] > 100) {
    speed = MPU[1];
    analogWrite(pinENB, speed);
    digitalWrite(pinIN3, LOW);
    digitalWrite(pinIN4, HIGH);
  }

 if (MPU[0] < 100 && MPU[0] > -100 && MPU[1] < 100 && MPU[1] > -100) {
    analogWrite(pinENA, 0);
    analogWrite(pinENB, 0);
  }


  if (distance_cm < DISTANCE2) {
    if (MPU[0] > 100) {
      speed = MPU[0];
      analogWrite(pinENA, 0);
      analogWrite(pinENB, 0);
    }

    if (MPU[0] < -100) {
      speed = -1 * MPU[0];
      analogWrite(pinENA, speed);
      analogWrite(pinENB, speed);
      digitalWrite(pinIN1, HIGH);
      digitalWrite(pinIN2, LOW);
      digitalWrite(pinIN3, HIGH);
      digitalWrite(pinIN4, LOW);
    }

    if (MPU[1] < -100) {
      speed = -1 * MPU[1];
      analogWrite(pinENB, speed);
       digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      digitalWrite(pinIN3, HIGH);
      digitalWrite(pinIN4, LOW);
    }

    if (MPU[1] > 100) {
      speed = MPU[1];
      analogWrite(pinENA, speed);
      digitalWrite(pinIN1, HIGH);
      digitalWrite(pinIN2, LOW);
      digitalWrite(pinIN3, LOW);
      digitalWrite(pinIN4, LOW);
    }

    if (MPU[0] < 100 && MPU[0] > -100 && MPU[1] < 100 && MPU[1] > -100) {
      analogWrite(pinENA, 0);
      analogWrite(pinENB, 0);
    }
  }
}


void panTilt() {

  MPU[0] = map(MPU[0], 90, -90, 0, 180);
  MPU[1] = map(MPU[1], 90, -90, 0, 180);
  ultraBuzz();

  ServoX.write(MPU[1]);
  ServoY.write(MPU[0]);


}

void ultraBuzz() {
  digitalWrite(TRIG_PIN, HIGH);
  TTrig2 = millis();
  TBuz2 = millis();
  if (TTrig2 - TTrig1 >= TTrig) {
    digitalWrite(TRIG_PIN, LOW);
    TTrig1 = TTrig2;

    if (TBuz2 - TBuz1 >= TBuz) {
      // measure duration of pulse from ECHO pin
      duration_us = pulseIn(ECHO_PIN, HIGH);
      // calculate the distance
      distance_cm = 0.017 * duration_us;

      if (distance_cm < DISTANCE1 && distance_cm > DISTANCE2) {
        tone(BUZZER_PIN, 200); // turn on Piezo Buzzer
      }
      else {
        if (distance_cm < DISTANCE2)
          tone (BUZZER_PIN, 500);
        else
          noTone(BUZZER_PIN);
      }
      TBuz1 = TBuz2;
    }
  }
}
