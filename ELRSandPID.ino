#include <CrsfSerial.h>
#include <Servo.h>
#include <ServoStabilizer.h>
#include <MPU6050.h>

SerialPIO Receiver(10, 11);
CrsfSerial crsf(Receiver, 200000);
Servo Aileron, Elevator, Motor, Rudder;

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;

ServoStabilizer estabilizacion;

//Pueden sacar valores de pitch y roll directamente utilizando estabilizacion.pitchValue() y estabilizacion.rollValue();

void setup() {
  
  Serial.begin(115200);
  mpu.initialize();

  Serial.println("Init ELRS");
  Receiver.begin(200000);
  Aileron.attach(29);
  Elevator.attach(28);
  Motor.attach(27);
  Rudder.attach(26);
  Aileron.writeMicroseconds(1500);
  Elevator.writeMicroseconds(1500);
  Motor.writeMicroseconds(1000);
  Rudder.writeMicroseconds(1500);

  estabilizacion.begin();
  estabilizacion.setGains(20,0.03,0.02);
  estabilizacion.setAngles(0.0,0.0);

}

void loop() {

  crsf.loop();
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
  float roll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
  
  estabilizacion.update(pitch, roll);

  if (crsf.getChannel(5)>1500){
    Elevator.writeMicroseconds(1500 - estabilizacion.outputsPitch());
    Aileron.writeMicroseconds(1500 + estabilizacion.outputsRoll());
    Serial.print(estabilizacion.outputsPitch()); Serial.println(estabilizacion.outputsRoll()); 
    Serial.println("Switch activao");
  }
  else{
    Aileron.writeMicroseconds(crsf.getChannel(1));
    Elevator.writeMicroseconds(crsf.getChannel(2));
    Motor.writeMicroseconds(crsf.getChannel(3));
    Rudder.writeMicroseconds(crsf.getChannel(4));
    Serial.println("Switch desactivao");
  }
 
}
