#include <Wire.h>
#include <Motion.h>


Motion::~Motion() {
    this->shutdownMPU();
}

void Motion::setupMPU() {
    //Starts up the MPU and configures it to measure
    Wire.begin();
    Wire.beginTransmission(this->MPU_ADDR); 
    Wire.write(0x6B); //power register
    Wire.write(0);
    Wire.endTransmission(true);
    Serial.println("MPU is ready");
}

void Motion::setAccelerationSensitivity(int range) {
    int sensitivityConfig = 0;
    switch (range) {
        case 0:
            sensitivityConfig = 0;
            sensitivityDivisionFactor = 16384;
            break;
        case 1:
            sensitivityConfig = 0x08;
            sensitivityDivisionFactor = 8192;
            break;
        case 2:
            sensitivityConfig = 0x10;
            sensitivityDivisionFactor = 4096;
            break;
        case 3:
            sensitivityConfig = 0x18;
            sensitivityDivisionFactor = 2048;
            break;
    }
    Wire.beginTransmission(this->MPU_ADDR); 
    Wire.write(0x1c); //accel config for sensitivity 
    Wire.write(sensitivityConfig);
    Wire.endTransmission(true);
    Serial.println("Accelerometer sensitivity is configured");
}

void Motion::start(int accelerationSensitivity) {
    this->setupMPU();
    this->setAccelerationSensitivity(accelerationSensitivity);
}

void Motion::updateMotionData(bool print) {
    //Starts the transmission
    Wire.beginTransmission(this->MPU_ADDR);
    Wire.write(0x3B); // Starts with accel X
    Wire.endTransmission(false); 
    Wire.requestFrom(MPU_ADDR, 14, 1); 
    
    //Reads the registers 
    this->accelerationDataX = Wire.read()<<8 | Wire.read(); 
    this->accelerationDataY = Wire.read()<<8 | Wire.read(); 
    this->accelerationDataZ = Wire.read()<<8 | Wire.read(); 
    Wire.read()<<8 | Wire.read(); //temperature, not needed
    this->gyroDataX = Wire.read()<<8 | Wire.read(); 
    this->gyroDataY = Wire.read()<<8 | Wire.read(); 
    this->gyroDataZ = Wire.read()<<8 | Wire.read();

    if (print) {
        Serial.print("Accel X:");
        Serial.println(this->accelerationDataX);
        Serial.print("Accel Y:");
        Serial.println(this->accelerationDataY);
        Serial.print("Accel Z:");
        Serial.println(this->accelerationDataZ);
        Serial.print("Gyro X:");
        Serial.println(this->gyroDataX);
        Serial.print("Gyro Y:");
        Serial.println(this->gyroDataY);
        Serial.print("Gyro Z:");
        Serial.println(this->gyroDataZ);
    }
}

void Motion::stopReadingMPU() {
    Wire.endTransmission(true);
    Serial.println("MPU reading has stopped");
}

void Motion::shutdownMPU() {
    //Need to send power off bit
    //Wire.beginTransmission(this->MPU_ADDR); 
    //Wire.write();
    //Wire.write();
    Serial.println("MPU has been shutdown");
}

void Motion::setAccelDrift(short accelXDiff, short accelYDiff, short accelZDiff) {
    this->accelXDiff = accelXDiff;
    this->accelYDiff = accelYDiff;
    this->accelZDiff = accelZDiff;
}

bool Motion::hasStablized(float duration, float interval) {
    uint32_t currentTime = millis();
    if (currentTime - this->lastStableIntervalCheck > interval * 1000 && !this->isCheckingStable) {
        this->lastStablecheck = currentTime;
        this->isCheckingStable = true;
    }
    if (this->isCheckingStable) {
        this->lastStableIntervalCheck = currentTime;

        float gravities[3];
        this->getAccelDataInGravities(gravities, false);
        gravities[2] -= 1;
        float totalGravities = abs(gravities[0]) + abs(gravities[1]) + abs(gravities[2]);
        if (totalGravities <= 0.1) {
            float currentSecondsStable = currentTime - this->lastStablecheck;

            this->secondsStable += currentSecondsStable;
            if (this->secondsStable >= duration * 1000) {
                this->isCheckingStable = false;
                this->secondsStable = 0;
                return true;
            }
        } else {
            this->secondsStable = 0;
        }
        this->lastStablecheck = currentTime;
    }

    return false;
}

short* Motion::getAccelData(short arr[3]) {
    arr[0] = this->accelerationDataX + this->accelXDiff;
    arr[1] = this->accelerationDataY + this->accelYDiff;
    arr[2] = this->accelerationDataZ + this->accelZDiff;

    return arr;
}

float* Motion::getAccelDataInGravities(float arr[3], bool print) {
    short tempArr[3];
    this->getAccelData(tempArr);
    arr[0] = (float)tempArr[0] / this->sensitivityDivisionFactor;
    arr[1] = (float)tempArr[1] / this->sensitivityDivisionFactor;
    arr[2] = (float)tempArr[2] / this->sensitivityDivisionFactor;

    if (print) {
        Serial.print("Accel X:");
        Serial.println(arr[0]);
        Serial.print("Accel Y:");
        Serial.println(arr[1]);
        Serial.print("Accel Z:");
        Serial.println(arr[2]);
    }

    return arr;
}

short* Motion::getGyroData(short arr[3]) {
    arr[0] = this->gyroDataX;
    arr[1] = this->gyroDataY;
    arr[2] = this->gyroDataZ;

    return arr;
}