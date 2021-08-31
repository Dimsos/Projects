//  PROYECTO DE GRADO - COCHE CONTROLADO A CONTROL REMOTO POR GESTOS DE GIROSCOPIO + PAN&TILT CON SENSOR ULTRASONIDOS
//
//  PROGRAMA TRANSMISOR - GUANTE, ANTENA NRF24L01+MÓDULO, MPU-6050, TOUCHSENSOR

// ---- Incluimos las librerías del SPI, Antena, Servos (del Pan&Tilt)
#include "SPI.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// ---- Variables NRF24L01
RF24 radio(9, 10);                      //Objeto llamado "radio". Pines del SPI, correspondientes a CE y CSN

const byte address[6] = "00001";        //Matriz de bytes identificativo. Dirección que utilizarán las antenas para identificarse entre sí

// ---- Variables MPU-6050
const int mpuAddress = 0x68;            // La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo del estado de AD0. Si no se especifica, 0x68 estará implicito

MPU6050 mpu(mpuAddress);

float MPU[3];                           //Creamos una matriz que almacenará los datos X y Y del giroscopio

int16_t ax, ay, az;                     //Declaramos las variables obtenidas del giroscopio, las variables que utilizaremos para las fórmulas del filtro complementario y para el contador millis
int16_t gx, gy, gz;
long tiempo_prev;
float dt, ang_x, ang_y, ang_x_prev, ang_y_prev, accel_ang_x, accel_ang_y;

// ---- Variables LED y Touch Sensor

const int sensorPin = 5;
const int LED = 3;

int LedState = LOW;                     //Estas variables nos permitirán controlar el estado del LED para dar indicaciones
float LedVal = 0.0;
int currentButtonState;
int lastButtonState;

void setup() {
  Serial.begin(115200);                  //Inicializamos comunicación serie a 115200 baudios

  Wire.begin();                          //Inicializamos el I2C y el MPU-6050
  mpu.initialize();

  pinMode(sensorPin, INPUT);
  pinMode(LED, OUTPUT);

  /* Inicializamos el objeto "radio". Abrimos el canal de transmisión
       de datos identificado por la dirección dada anteriormente.
       Si se estan haciendo pruebas sobre mesa o una antena al lado de otra,
       radio.setPALevel nos permitirá mantener la potencia al mínimo.
       radio.stopListening establece a esta antena como transmisor. */
  radio.begin();
  radio.openWritingPipe(address);
  //radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  delay(1500);
}

void loop() {

  lastButtonState = currentButtonState;
  currentButtonState = digitalRead(sensorPin);

  gyroMPU();
  radio.write(MPU, sizeof(MPU));

  if (lastButtonState == LOW && currentButtonState == HIGH) {
    LedState = !LedState;
    digitalWrite(LED, LedState);
    LedVal = 1.0;
    MPU[2] = LedVal;
    radio.write(MPU, sizeof(MPU));

    while (LedState == HIGH) {
      lastButtonState = currentButtonState;
      currentButtonState = digitalRead(sensorPin);
      gyroMPU();
      radio.write(MPU, sizeof(MPU));
      if (lastButtonState == LOW && currentButtonState == HIGH) {
        LedState = !LedState;
        digitalWrite(LED, LedState);
        LedVal = 0.0;
        MPU[2] = LedVal;
        radio.write(MPU, sizeof(MPU));
      }
    }
  }

}

float gyroMPU() {
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14159);
  accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14159);

  ang_x = 0.98 * (ang_x_prev + (gx / 131) * dt) + 0.02 * accel_ang_x;
  ang_y = 0.98 * (ang_y_prev + (gy / 131) * dt) + 0.02 * accel_ang_y;

  ang_x_prev = ang_x;
  ang_y_prev = ang_y;

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  Serial.print(F("Rotación en X:  "));
  Serial.print(ang_x);
  Serial.print(F("\t Rotación en Y: "));
  Serial.println(ang_y);
  Serial.print(F("\t LED: "));
  Serial.println(LedVal);

  MPU[0] = ang_x;
  MPU[1] = ang_y;

  return MPU[3];
}
