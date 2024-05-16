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


    // math structs
    typedef union {
        float array[3];

        struct{
            float roll;
            float pitch;
            float yaw;
        } angle;
    } madEuler;

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

#define VECTOR_ZERO ((madVector){.array = {0.0f, 0.0f, 0.0f} })
#define IDENTITY_QUATERNION ((madQuaternion){.array = {1.0f, 0.0f, 0.0f, 0.0f} })

/* Macros/Enums ------------------------------------------------------------*/


/* Class ------------------------------------------------------------------*/
class MahrsHPP
{
	public:
    // function declarations 
        void mahrsInitialisation(MahrsStruct *const mahrs);
        void setParams(MahrsStruct *const mahrs, const params *const parameters);
        void reset(MahrsStruct *const mahrs);

        void Update(MahrsStruct *const mahrs, const madVector gyro, const madVector accel, const madVector magno, const float deltaT);

        static inline madVector Feedback(const madVector sensor, const madVector reference);

        static inline madVector HalfMagnetic(const MahrsStruct *const mahrs);

        static inline madVector HalfGravity(const MahrsStruct *const mahrs);

        static inline int Clamp(const int value, const int min, const int max);

        static inline float degToRads(const float degrees);

        static inline madVector vectorCrossProduct(const madVector vectorA, const madVector vectorB);

        static inline float vectorDotProduct(const madVector vectorA, const madVector vectorB);

        static inline bool VectorIsZero(const madVector vector);

        static inline float VectorMagSquared(const madVector vector);

        static inline madVector vectorHadamarProduct(const madVector vectorA, const madVector vectorB);

        static inline float vectorSum(const madVector vector);

        static inline madVector vectorNormalise(const madVector vector);

        static inline float InverseSquareRoot(const float x);

        static inline madVector vectorMultiplyScalar(const madVector vector, const float scalar);
	protected:
	    // Data
	    // AccelGyroMagnetismData* data;

	private:

};

#endif
