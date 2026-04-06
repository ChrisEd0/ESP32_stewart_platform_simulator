#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050_light.h>
#define RX_BUFFER_SIZE 64


TwoWire I2C_1 = TwoWire(1);   // Puerto I2C #1
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40,I2C_1);

unsigned int pos0=125; // ancho de pulso en cuentas para pocicion 0°
unsigned int pos180=520; // ancho de pulso en cuentas para la pocicion 180°
int m[6];
unsigned long timer = 0;
char rxBuffer[RX_BUFFER_SIZE];
uint8_t rxIndex = 0;

void setServo(uint8_t n_servo, int angulo) {
  int duty;
  duty=map(angulo,0,180,pos0, pos180);
  servos.setPWM(n_servo, 0, duty);  
}

void plataformInit(){
  setServo(0,90);
  setServo(1,90);
  setServo(2,90);
  setServo(3,90);
  setServo(4,90);
  setServo(5,90);
}

void plataformMove(){
  setServo(0,90-m[0]);
  setServo(1,90+m[1]);
  setServo(2,90-m[2]);
  setServo(3,90+m[3]);
  setServo(4,90-m[4]);
  setServo(5,90+m[5]);
}

void receiveInts() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '#') {
      rxBuffer[rxIndex] = '\0';

      if (parseInts(rxBuffer, m)) {
        // ✔ m[0]..m[5] correctos
      }

      rxIndex = 0;
    }
    else if (c != '\r' && rxIndex < RX_BUFFER_SIZE - 1) {
      rxBuffer[rxIndex++] = c;
    }
  }
}


// ================== PARSER ULTRA RÁPIDO ==================
bool parseInts(char *buf, int *out) {
  int idx = 0;
  int val = 0;
  bool neg = false;
  bool inNumber = false;

  for (int i = 0; buf[i] != '\0'; i++) {
    char c = buf[i];

    if (c == '\r') continue; // ignorar CR

    if (c == '-') {
      neg = true;
    }
    else if (c >= '0' && c <= '9') {
      val = val * 10 + (c - '0');
      inNumber = true;
    }
    else if (inNumber) {
      out[idx++] = neg ? -val : val;
      val = 0;
      neg = false;
      inNumber = false;

      if (idx >= 6) break;
    }
  }

  // Guardar último número
  if (inNumber && idx < 6) {
    out[idx++] = neg ? -val : val;
  }

  return (idx == 6); // frame válido
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  I2C_1.begin(25,26);//SDA=25 SCL=26
  servos.begin();  
  servos.setPWMFreq(50); //Frecuecia PWM de 60Hz o T=16,66ms
  plataformInit();
  


}

void loop() {  
  receiveInts();
  plataformMove();   
}

