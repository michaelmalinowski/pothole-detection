#ifndef Motion_H
#define Motion_H

class Motion {

    const int MPU_ADDR = 0x68;

    int sensitivityDivisionFactor = 16384; //factor to divide the raw accel data by
    //default is 2g range

    short accelXDiff, accelYDiff, accelZDiff = 0;
    short accelerationDataX, accelerationDataY, accelerationDataZ;
    short gyroDataX, gyroDataY, gyroDataZ;

    float secondsStable = 0;
    uint32_t lastStablecheck = 0;
    uint32_t lastStableIntervalCheck = 0;
    bool isCheckingStable = false;

    public:
        ~Motion();

        void start(int accelSensitivity);

        void updateMotionData(bool print=false);

        void setAccelDrift(short accelXDiff, short accelYDiff, short accelZDiff);
        
        bool hasStablized(float duration, float interval);

        float* getAccelDataInGravities(float arr[3], bool print=false);

        short* getAccelData(short arr[3]);
        
        short* getGyroData(short arr[3]);

    private:
        void setupMPU();

        void stopReadingMPU();

        void shutdownMPU();

        void setAccelerationSensitivity(int range);

        void updateLastMotion();
};

#endif