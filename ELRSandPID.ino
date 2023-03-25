#include <CrsfSerial.h>
#include <Servo.h>
#include <ServoStabilizer.h>

SerialPIO Receiver(10, 11);
CrsfSerial crsf(Receiver, 200000);
Servo Aileron, Elevator, Motor, Rudder;

ServoStabilizer estabilizacion;


void setup() {
  
  Serial.begin(115200);

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

  estabilizacion.initialize();
  estabilizacion.setGains(20,0.03,0.02);
  estabilizacion.setAngles(45.0,45.0);

}

void loop() {

  crsf.loop();

  if (crsf.getChannel(5)>1500){
    estabilizacion.update();
    Elevator.writeMicroseconds(1500 - estabilizacion.outputsPitch());
    Aileron.writeMicroseconds(1500 + estabilizacion.outputsRoll());
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