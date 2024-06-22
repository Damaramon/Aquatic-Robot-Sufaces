#include <Wire.h>
#include <Adafruit_GFX.h>
#include <HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Arduino.h>
#include <Wire.h>
#include <Update.h>
#include <Adafruit_SSD1306.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <sstream>
#include <vector>
#include <fstream>
#include <MPU6050_light.h>
#include <ESP32Servo.h>

Servo escMotor1;
Servo escMotor2; 
//////////mpu6050
MPU6050 mpuu(Wire);

Adafruit_MPU6050 mpu;
int insert=0;
double tSebelum = 0;
double x_awal = 0, y_awal = 0;
double pa_x = 0, pa_y;
double hasil1, hasil2, hasil3;
double t = 0.1;
int validationA = 0, validationB = 0;

long timer = 0;
///////////

RF24 radio(4,5 ); // CE, CSN

const byte address[6] = "00001";
void MotorForward();
void MotorBackward();
void MotorLeft();
void MotorRight();
void MotorStop();

struct DataPacket {
    double x;
    double y;
};

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);

  escMotor1.attach(15);
  escMotor2.attach(2);
/////////
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  //Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  byte status = mpuu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {} // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpuu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  Serial.println("");
  delay(100);
}

void loop() {
  DataPacket packet;
 radio.startListening();
  if (radio.available()) {
    char data[32] = "";
    radio.read(&data, sizeof(data));
    Serial.println(data);

    if (strcmp(data, "F") == 0) MotorForward();
    else if (strcmp(data, "B") == 0) MotorBackward();
    else if (strcmp(data, "L") == 0) MotorLeft();
    else if (strcmp(data, "R") == 0) MotorRight();
    else if (strcmp(data, "S") == 0) MotorStop();
    Serial.println(data);

  }
  ///////MPU6050
  mpuu.update();
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
// /////////////accell

  double x_akhir, y_akhir;
  double acl_x, acl_y, acl_z, gy_x, gy_y, gy_z;
  double vx, vy, vz, sx, sy, sz;

  if (millis() - timer > 100) {
    acl_x = kalmanFilter(mpuu.getAccX(), pa_x);
    acl_y = kalmanFilter(mpuu.getAccY(), pa_y);
    // Memeriksa apakah nilai absolut sudut melebihi ambang batas
    if (acl_x < 0.1 && acl_x > -0.1) {
      acl_x = 0.0;
    }

    if (acl_y < 0.1 && acl_y > -0.1) {
      acl_y = 0.0;
    }

    acl_x *= 9.8;
    acl_y *= 9.8;

    acl_x = ValidationFilterA(acl_x);
    acl_y = ValidationFilterB(acl_y);

    vx = acl_x * t;
    vy = acl_y * t;

    sx = vx * t + 0.5 * acl_x * pow(t, 2);
    sy = vy * t + 0.5 * acl_y * pow(t, 2);

    x_akhir =  sx * cos(mpuu.getAngleZ()) - sy * sin(mpuu.getAngleZ());
    y_akhir = sy * cos(mpuu.getAngleZ()) + sx * sin(mpuu.getAngleZ());

    x_akhir =  sx;
    y_akhir = sy;

    x_awal += x_akhir;
    y_awal += y_akhir;

    Serial.print("X : ");
    Serial.print(x_awal, 8);
    Serial.print("\tY : ");
    Serial.println(y_awal, 8);
    Serial.println(F("=====================================================\n"));
    timer = millis();
  }
  insert++;
  
  if(insert == 5){
    

    radio.stopListening();
    insert=0;
    packet.x = x_awal;
    packet.y = y_awal; 
 
    radio.write(&packet, sizeof(packet));

  }
}

void MotorForward() {
  Serial.println("nasi menyebalkan");
  escMotor1.writeMicroseconds(1900);
  escMotor2.writeMicroseconds(1900);
  delay(2000);
}

void MotorBackward() {

  escMotor1.writeMicroseconds(1500);
  escMotor2.writeMicroseconds(1500); 
}

void MotorLeft() {
  escMotor1.writeMicroseconds(1900);
  escMotor2.writeMicroseconds(1600); 
}

void MotorRight() {
  escMotor1.writeMicroseconds(1600);
  escMotor2.writeMicroseconds(1900); 
}

void MotorStop() {
  escMotor1.writeMicroseconds(1500);
  escMotor2.writeMicroseconds(1500);
}

double kalmanFilter(double U, double& P_var) {
  static const double R = 5.99;
  static const double H = 1.00;
  static double Q = 15;
  static double U_hat = 0;
  static double K = 0;
  double P = P_var;
  K = P * H / (H * P * H + R);
  U_hat = U_hat + K * (U - H * U_hat);

  P = (1 - K * H) * P + Q;
  P_var = P;

  return U_hat;
}

double ValidationFilterA(double AccelA) {
  if (AccelA > 0.1 && validationA == 0) {
    validationA = 30;
  } else if (AccelA < -0.1 && validationA == 0) {
    validationA = -30;
  }

  if (validationA > 0) {
    if (AccelA < -0.1) {
      AccelA = 0;
    }
    validationA -= 1;
  } else if (validationA < 0) {
    if (AccelA > 0.1) {
      AccelA = 0;
    }
    validationA += 1;
  }
  Serial.print("AccelA: ");
  Serial.println(validationA);

  return AccelA;
}

double ValidationFilterB(double AccelA) {
  if (AccelA > 0.1 && validationB == 0) {
    validationB = 30;
  } else if (AccelA < -0.1) {
    validationB = -30;
  }
  if (validationB > 0) {
    if (AccelA < -0.1) {
      AccelA = 0;
      validationB -= 1;
    }
  }

  if (validationB < 0) {
    if (AccelA > 0.1) {
      AccelA = 0;
      validationB += 1;
    }
  }

  return AccelA;
}
