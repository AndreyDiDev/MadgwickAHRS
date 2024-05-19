/**
 ******************************************************************************
 * File Name          : 
 * Description        :
 ******************************************************************************
*/
#ifndef MAHRS_HPP_
#define MAHRS_HPP_
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Definitions ------------------------------------------------------------------*/
typedef struct{
    float algorithmGain; // beta
    float gyroRange;
    float accelRejection;
    float magRejection;
    unsigned int recoveryTriggerPeriod;
} params;

    typedef union {
        float array[3];

        struct {
            float x;
            float y;
            float z;
        } axis;

    } madVector;

    typedef union {
        float array[4];

        struct{
            float w;
            float x;
            float y;
            float z;
        }element;
    } madQuaternion;

typedef union{
    float array[3][3];

    struct{
        float xx;
        float xy;
        float xz;
        float yx;
        float yy;
        float yz;
        float zx;
        float zy;
        float zz;
    } element;

} madMatrix;

typedef struct
{
    params Parameters;
    madQuaternion quaternion;

    madVector accel;
    
    bool initialisation;

    float rampedGain;
    float rampedGainStep;

    bool angularRateRecovery;

    madVector halfAccelFeedback;
    madVector halfMagFeedback;

    bool accelIgnored;
    
    int accelRecoveryTrigger;
    int accelRecoveryTimeout;

    bool magnoIgnored;
    int magneticRecoveryTrigger;
    int magneticRecoveryTimeout;

} MahrsStruct;


typedef struct{
    float accelError;
    bool accelIgnored;
    float accelRecoveryTrigger;

    float magneticError;
    bool magnoIgnored;
    float magneticRecoveryTrigger;
} madInternalStates;

typedef struct {
    bool initialization;
    bool angularRateRecovery;
    bool accelRecovery;
    bool magneticRecovery;
} madFlags;

typedef struct {
    float filterCoefficient;
    unsigned int timeout;
    unsigned int timer;
    madVector gyroscopeOffset;
} madOffset;


    // math structs
    typedef union {
        float array[3];

        struct{
            float roll;
            float pitch;
            float yaw;
        } angle;

    } madEuler;

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif


#define VECTOR_ZERO madVector {.array = {0.0f, 0.0f, 0.0f} }
#define IDENTITY_QUATERNION ((madQuaternion){.array = {1.0f, 0.0f, 0.0f, 0.0f} })
/**
 * @brief Identity matrix.
 */
#define IDENTITY_MATRIX ((madMatrix){ .array = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}} })


#define EULER_ZERO ((madEuler){ .array = {0.0f, 0.0f, 0.0f} })

/* Macros/Enums ------------------------------------------------------------*/


/* Class ------------------------------------------------------------------*/
// class MahrsHPP
// {
// 	public:
    // function declarations 
        static inline madEuler quaternionToEuler(const madQuaternion quaternion);

        static inline float degToRads(const float degrees);

        // setup functions 
        void mahrsInitialisation(MahrsStruct *const mahrs);

        void setParams(MahrsStruct *const mahrs, const params *const parameters);
        void madReset(MahrsStruct *const mahrs);

        void offsetInitialise(madOffset *const offset, const unsigned int sampleRate);

        madVector offsetUpdate(madOffset *const offset, madVector gyro);

        // algorithm functions 

        void Update(MahrsStruct *const mahrs, const madVector gyro, const madVector accel, const madVector magno, const float deltaT);

        void updateNoMagnetometer(MahrsStruct *const mahrs, const madVector gyro, const madVector accel, const float deltaT);

        void updateExternalHeading(MahrsStruct *const mahrs, const madVector gyro, const madVector accel, const float heading, const float deltaT);

        static inline madVector Feedback(const madVector sensor, const madVector reference);

        static inline madVector HalfMagnetic(const MahrsStruct *const mahrs);

        static inline madVector HalfGravity(const MahrsStruct *const mahrs);

        madVector getLinearAcceleration(const MahrsStruct *const mahrs);

        madVector getEarthAcceleration(const MahrsStruct *const mahrs);

        void setHeading(MahrsStruct *const mahrs, const float heading);

        // math 
        static inline madVector vectorSubtract(const madVector vectorA, const madVector vectorB);

        static inline int Clamp(const int value, const int min, const int max);

        static inline float Asin(const float value);

        static inline float vectorMagnitudeSquared(const madVector vector);

        static inline float vectorMagnitude(const madVector vector);

        static inline madVector vectorCrossProduct(const madVector vectorA, const madVector vectorB);

        static inline float vectorDotProduct(const madVector vectorA, const madVector vectorB);

        static inline bool VectorIsZero(const madVector vector);

        static inline float VectorMagSquared(const madVector vector);

        static inline madVector vectorHadamardProduct(const madVector vectorA, const madVector vectorB);

        static inline float vectorSum(const madVector vector);

        static inline madVector vectorNormalise(const madVector vector);

        static inline float InverseSquareRoot(const float x);

        static inline madVector vectorMultiplyScalar(const madVector vector, const float scalar);

        static inline madVector vectorAdd(const madVector vectorA, const madVector vectorB);

        static inline madQuaternion quaternionAdd(const madQuaternion quaternionA, const madQuaternion quaternionB);

        static inline madQuaternion quaternionMultiplyVector(const madQuaternion quaternion, const madVector vector);

        static inline madQuaternion quaternionNormalise(const madQuaternion quaternion);

        static inline madQuaternion quaternionMultiply(const madQuaternion quaternionA, const madQuaternion quaternionB);

        // other helper functions
        madQuaternion getQuaternion(const MahrsStruct *const mahrs);

        void setQuaternion(MahrsStruct *const mahrs, const madQuaternion quaternion);

        static inline madVector matrixMultiplyVector(const madMatrix matrix, const madVector vector);

        // madVector getLinearAcceleration(const MahrsStruct *const mahrs);

        // madVector getEarthAcceleration(const MahrsStruct *const mahrs);

        madFlags getFlags(const MahrsStruct *const mahrs);

        static inline madMatrix quaternionToMatrix(const madQuaternion quaternion);

        

        float compassCalculateHeading(const madVector accelerometer, const madVector magnetometer);

        /**
 * @brief Gyroscope and accelerometer calibration model.
 * @param uncalibrated Uncalibrated measurement.
 * @param misalignment Misalignment matrix.
 * @param sensitivity Sensitivity.
 * @param offset Offset.
 * @return Calibrated measurement.
 */
static inline madVector calibrationInertial(const madVector uncalibrated, const madMatrix misalignment, const madVector sensitivity, const madVector offset) {
    return matrixMultiplyVector(misalignment, vectorHadamardProduct(vectorSubtract(uncalibrated, offset), sensitivity));
}


/**
 * @brief Magnetometer calibration model.
 * @param uncalibrated Uncalibrated measurement.
 * @param softIronMatrix Soft-iron matrix.
 * @param hardIronOffset Hard-iron offset.
 * @return Calibrated measurement.
 */
static inline madVector calibrationMagnetic(const madVector uncalibrated, const madMatrix softIronMatrix, const madVector hardIronOffset) {
    return matrixMultiplyVector(softIronMatrix, vectorSubtract(uncalibrated, hardIronOffset));
}


	// protected:
	    // Data
	    // AccelGyroMagnetismData* data;

	// private:

// };

#endif