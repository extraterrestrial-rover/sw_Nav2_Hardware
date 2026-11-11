#include <Arduino.h>

// put function declarations here:
#define frontRatio 1
#define backRatio 1

uint8_t leftFront[] = {25,26}; // MotorDriver1 {dir1,pwm1};
uint8_t leftBack[] = {12,13};

uint8_t rightFront[] = {22,23}; //MotorDriver3 {dir1,pwm1};
uint8_t rightBack[] = {18,19};


void setMotor(uint8_t motor[]) {
    pinMode(motor[0], OUTPUT);
    pinMode(motor[1], OUTPUT); 
}
void motorCtrl(uint8_t motor[], uint8_t dir, uint8_t pwm) {
    digitalWrite(motor[0], dir); // Set direction
    analogWrite(motor[1], pwm);  // Set speed (PWM)
}

void linear(int dir, int pwm){
    motorCtrl(leftFront, dir, pwm);
    motorCtrl(leftBack, dir, pwm);

    motorCtrl(rightFront, dir, pwm);
    motorCtrl(rightBack, dir, pwm);
}

void turn(int dir, int pwm){
    motorCtrl(leftFront, !dir, pwm*frontRatio);
    motorCtrl(leftBack, !dir, pwm*backRatio);
    
    motorCtrl(rightFront, dir, pwm*frontRatio);
    motorCtrl(rightBack, !dir, pwm*backRatio);
}

void ctrl(void *pvParameters) {
    while (true) {
        if (Serial.available() > 0) {
            uint8_t command = (uint8_t)Serial.read(); // First byte is the command (direction)
            uint8_t speed = (uint8_t)Serial.read(); // Second byte is speed as an 8-bit integer
            
            Serial.print(command);
            Serial.print(" ");
            Serial.println(speed);
            // Control logic
            switch(command) {
                // Forward
                case 1:
                    linear(0, speed);
                    break;
                // Backward
                case 2:
                    linear(1, speed);
                    break;
                // Left turn
                case 3:
                    turn(0, speed);
                    break;
                // Right turn
                case 4:
                    turn(1, speed);
                    break;
                // Stop
                case 5:
                    linear(0, 0);
                    break;
            }
        }
    }
}


void setup() {
    setMotor(leftFront);
    setMotor(leftBack);
    setMotor(rightFront);
    setMotor(rightBack);
    Serial.begin(115200);

    xTaskCreatePinnedToCore(ctrl, "Control", 10000, NULL, 1, NULL, 0);
}

void loop() {
    // Main loop does nothing, tasks are running on cores
    Serial.println("drive");
    delay(1000);
}


