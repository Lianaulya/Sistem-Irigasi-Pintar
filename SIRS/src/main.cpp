#include <Arduino.h>
#include <PID_v1.h>

// Definisi pin
int sensorPin = A0;      // Sensor kelembapan (AO)
int relayPin = 3;        // Relay pompa

// Nilai sensor saat basah (dalam air) dan kering (di udara)
int ADC_DRY = 800;
int ADC_WET = 400;   

// Target kelembapan tanah 60% lembap
double setpoint = 60.0;  

// Variabel PID
double input = 0.0;      // Kelembapan sekarang (%)
double output = 0.0;     // Output PID (0-100)

// Parameter tuning PID
double Kp = 2.0;
double Ki = 0.5;
double Kd = 0.1;

// Kontrol relay
int RELAY_ON = LOW;      // Relay aktif LOW
int RELAY_OFF = HIGH;

// Buat objek PID controller
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Variabel untuk menyimpan nilai sensor
int sensorValue = 0;

void setup() {
  // Inisialisasi komunikasi serial pada 9600 baud rate
  Serial.begin(9600);
  
  // Inisialisasi pin relay sebagai output
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, RELAY_OFF);  // Pompa mati dulu
  
  // Setup PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);
  myPID.SetSampleTime(1000);
}

void loop() {
  // Baca nilai analog dari sensor
  sensorValue = analogRead(sensorPin);
  
  // Konversi ADC ke persen kelembapan (0-100%)
  // Semakin tinggi ADC = semakin kering
  input = map(sensorValue, ADC_DRY, ADC_WET, 0, 100);
  input = constrain(input, 0, 100);
  
  // Compute PID
  myPID.Compute();
  
  // Print ke Serial Monitor untuk monitoring
  Serial.print("ADC: ");
  Serial.print(sensorValue);
  Serial.print(" | Moisture: ");
  Serial.print(input);
  Serial.print("% | PID Out: ");
  Serial.print(output);
  Serial.print(" | Pump: ");
  
  // Kontrol pompa berdasarkan output PID
  // Kalau output > 50, pompa nyala (tanah terlalu kering)
  if (output > 50) {
    digitalWrite(relayPin, RELAY_ON);
    Serial.println("ON");
  } else {
    digitalWrite(relayPin, RELAY_OFF);
    Serial.println("OFF");
  }
  
  // Tunggu 1 detik sebelum pembacaan berikutnya
  delay(1000);
}
