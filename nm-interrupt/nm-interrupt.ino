#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO055.h"
#include "imumaths.h"

#define BNO055_SAMPLERATE_DELAY_MS (500)
#define interrupt_pin 5
bool nm_interrupt;

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  if (!bno.begin())
  {
    Serial.print("No connection.");
    while (1);
  }
  nm_interrupt = false;
  pinMode(interrupt_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interrupt_pin), nm_interrupt_callback, RISING);
  //bno.enableSlowNoMotion(5, 1, NO_MOTION);
  bno.enableAnyMotion(255,1);
  bno.enableInterruptsOnXYZ(ENABLE, ENABLE, ENABLE);
  bno.setExtCrystalUse(true);
}

void loop(void)
{
  if (!nm_interrupt) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    Serial.print("X: ");
    Serial.print(euler.x());
    Serial.print(" Y: ");
    Serial.print(euler.y());
    Serial.print(" Z: ");
    Serial.print(euler.z());
    Serial.print("\t\t");

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.println(mag, DEC);
  }
  else
  {
    bno.resetInterrupts();
    nm_interrupt = false;
    Serial.println("NM-Interrupt");
    delay(5000);
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void nm_interrupt_callback(void)
{
  nm_interrupt = true;
}
