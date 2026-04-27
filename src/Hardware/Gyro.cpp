#include "Gyro.h"

Gyro::Gyro() : biasZ(0), angle(0), absAngle(0), shouldAbsAngle(0), lastMicros(0) {

}

void Gyro::Init() {
    Wire.swap(1);       // Alternative Pins: PC2(SDA)/PC3(SCL) = Pin 16/17
    Wire.begin();       // Als I2C Master starten
    Wire.setClock(400000);
    
    // MPU6050 Aufwecken
    Wire.beginTransmission(0x68);
    Wire.write(0x6B); Wire.write(0x00);
    Wire.endTransmission();

    delay(100); // MPU6050 braucht Zeit zum Stabilisieren nach Wakeup

    // Gyro Range auf ±500°/s setzen (Register 0x1B, Wert 0x08)
    // Teiler wird dann 65.5 statt 131.0
    Wire.beginTransmission(0x68);
    Wire.write(0x1B); Wire.write(0x08);
    Wire.endTransmission();

    // DLPF auf ~42Hz setzen (Register 0x1A, Wert 0x03) — filtert Vibrationen
    Wire.beginTransmission(0x68);
    Wire.write(0x1A); Wire.write(0x03);
    Wire.endTransmission();

    delay(50);

    // Kalibrierung (Maus muss stillstehen!)
    float sum = 0;
    for(int i=0; i<500; i++) {
        Wire.beginTransmission(0x68);
        Wire.write(0x43); // Gyro X Register
        Wire.endTransmission(false);
        Wire.requestFrom(0x68, 2);
        sum += (int16_t)(Wire.read() << 8 | Wire.read());
        delay(2);
    }
    biasZ = sum / 500.0;
    angle = 0;
    absAngle = 0;
    shouldAbsAngle = 0;
    lastMicros = micros();
}

void Gyro::update() {
    uint32_t now = micros();
    float dt = (now - lastMicros) / 1000000.0;
    lastMicros = now;

    Wire.beginTransmission(0x68);
    Wire.write(0x43); // Gyro X Register
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 2);
    int16_t rawX = (Wire.read() << 8 | Wire.read());

    // 65.5 ist der Teiler für den 500°/s Bereich
    float vZ = ((float)rawX - biasZ) / 65.5;
    
    if(fabs(vZ) > 0.5) { // Rauschunterdrückung
        angle += vZ * dt;
        absAngle += vZ * dt;
    }
}