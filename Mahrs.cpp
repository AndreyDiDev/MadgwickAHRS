// Main Mahrs file to control flow of algorithm 


/* Includes/Header Files ------------------------------------------------------------------*/
#include <math.h>
#include <float.h> 
// #include "MadgwickAHRS\MarhsHPP.hpp"
#include "MarhsHPP.hpp"

/* Definitions ------------------------------------------------------------------*/

// initial gain during initialisation 
#define INITIAL_GAIN (10.0f)

// initialisation period in seconds 
#define INITIALISATION_PERIOD (3.0f)

/* Function Declarations ------------------------------------------------------------------*/

static inline madVector halfGravity(const MahrsStruct *const mahrs);

static inline madVector halfMagnetic(const MahrsStruct *const mahrs);

static inline madVector feedback(const madVector sensor, const madVector reference);

static inline int clamp(const int value, const int min, const int max);


/* Functions ------------------------------------------------------------------*/

// initialise 
void mahrsInitialisation(MahrsStruct *const mahrs){
    const params parameters = {
        .algorithmGain = 0.5f,
        .gyroRange = 0.5f,
        .accelRejection = 90.0f,
        .magRejection = 90.0f,
        .recoveryTriggerPeriod = 0,
    };

    setParams(mahrs, &parameters);
    reset(mahrs);
}

void setParams(MahrsStruct *const mahrs, const params *const parameters){
    mahrs->Parameters.algorithmGain = parameters->algorithmGain;
    mahrs->Parameters.gyroRange = parameters->gyroRange== 0.0f ? FLT_MAX : powf(0.5f * sinf(degToRads(parameters->accelRejection)), 2);
    mahrs->Parameters.accelRejection = parameters->accelRejection == 0.0f ? FLT_MAX : powf(0.5f * sinf(degToRads(parameters->accelRejection)), 2);
    mahrs->Parameters.magRejection = parameters->magRejection == 0.0f ? FLT_MAX : powf(0.5f * sinf(degToRads(parameters->magRejection)), 2);
    mahrs->Parameters.recoveryTriggerPeriod = parameters->recoveryTriggerPeriod;
    mahrs->accelRecoveryTimeout = mahrs->Parameters.recoveryTriggerPeriod;
    mahrs->magneticRecoveryTimeout = mahrs->Parameters.recoveryTriggerPeriod;

    if((parameters->algorithmGain == 0.0f) || (parameters->recoveryTriggerPeriod == 0)){
        mahrs->Parameters.accelRejection = FLT_MAX;
        mahrs->Parameters.magRejection = FLT_MAX;
    };

    if(mahrs->initialisation == false){
        mahrs->rampedGain = mahrs->Parameters.algorithmGain;
    };

    mahrs->rampedGainStep=(INITIAL_GAIN - mahrs->Parameters.algorithmGain)/INITIALISATION_PERIOD;
}

// resets the ahrs 
void reset(MahrsStruct *const mahrs){
    mahrs->quaternion = IDENTITY_QUATERNION;
    mahrs->accel = VECTOR_ZERO;
    mahrs->initialisation = true;
    mahrs->rampedGain = INITIAL_GAIN;
    mahrs->angularRateRecovery = false;
    mahrs->halfAccelFeedback = VECTOR_ZERO;
    mahrs->halfMagFeedback = VECTOR_ZERO;
    mahrs->accelIgnored = false;
    mahrs->accelRecoveryTrigger = 0;
    mahrs->accelRecoveryTimeout = mahrs->Parameters.recoveryTriggerPeriod;
    mahrs->magnoIgnored = false;
    mahrs->magneticRecoveryTrigger = 0;
    mahrs->magneticRecoveryTimeout = mahrs->Parameters.recoveryTriggerPeriod;
}

// with magno
void Update(MahrsStruct *const mahrs, const madVector gyro, const madVector accel, const madVector magno, const float deltaT) {
    #define Q mahrs->quaternion.element // check if madQ or just quaternion 

    mahrs->accel = accel;

    /* Check if the absolute values of  gyro vector (x, y, z) exceed the gyro range.
    If any of the components exceed the gyro range, it resets. */
    if((fabsf(gyro.axis.x) > mahrs->Parameters.gyroRange) 
    || (fabsf(gyro.axis.y) > mahrs->Parameters.gyroRange) 
    || (fabsf(gyro.axis.z) > mahrs->Parameters.gyroRange)){

        const madQuaternion quaternion = mahrs->quaternion;
        reset(mahrs);
        mahrs->quaternion = quaternion;
        mahrs->angularRateRecovery = true;

    }

    /* Ramp down the gain during the initialization phase of the Mahrs algorithm. */
    if(mahrs->initialisation){
        mahrs->rampedGain -= mahrs->rampedGainStep * deltaT;
        if((mahrs->rampedGain < mahrs->Parameters.algorithmGain) || (mahrs->Parameters.algorithmGain == 0.0f)){
            mahrs->rampedGain = mahrs->Parameters.algorithmGain;
            mahrs->initialisation = false;
            mahrs->angularRateRecovery = false;
        }
    }

    // calc gravity 
    const madVector halfGravityVector = HalfGravity(mahrs);

    // calculate accelerometer feedback
    madVector halfAccelFeedback = VECTOR_ZERO;
    mahrs->accelIgnored = true;
    
    if(VectorIsZero(accel) == false){
        
        // calculate accel feedback and scale by 0.5
        mahrs->halfAccelFeedback = Feedback(vectorNormalise(accel), halfGravityVector);

        // Dont ignore accel if accel error below threshold
        if(mahrs->initialisation || ((VectorMagSquared(mahrs->halfAccelFeedback)) <= mahrs->Parameters.accelRejection)){
            mahrs->accelIgnored = false;
            mahrs->accelRecoveryTrigger -= 9;
        }else{
            mahrs->accelRecoveryTrigger += 1;
        }
    }

}

static inline madVector Feedback(const madVector sensor, const madVector reference){

    if(vectorDotProduct(sensor, reference) < 0.0f){
        return vectorNormalise(vectorCrossProduct(sensor, reference));
    }

    return vectorCrossProduct(sensor, reference);
}

// update without magno


static inline madVector HalfGravity(const MahrsStruct *const mahrs){
//TO DO
}

// math --------------------------------------------------------------
static inline float degToRads(const float degrees){
    return degrees *((float) 3.14 / 180.0f);
}

static inline madVector vectorCrossProduct(const madVector vectorA, const madVector vectorB){
#define A vectorA.axis
#define B vectorB.axis

    const madVector product = {.axis = {
        .x = A.y * B.z - A.z * B.y,
        .y = A.z * B.x - A.x * B.z,
        .z = A.x * B.y - A.y * B.x,
    }};

    return product;

    #undef A
    #undef B
}

static inline float vectorDotProduct(const madVector vectorA, const madVector vectorB){
    return vectorSum(vectorHadamarProduct(vectorA, vectorB));
}

/**
 * Returns true if vector is zero
*/
static inline bool VectorIsZero(const madVector vector){
    return (vector.axis.x == 0.0f) && (vector.axis.y == 0.0f) && (vector.axis.z == 0.0f);
}

// Returns vector magnitude squared 
static inline float VectorMagSquared(const madVector vector){
    return vectorSum(vectorHadamarProduct(vector, vector));
}

static inline madVector vectorHadamarProduct(const madVector vectorA, const madVector vectorB){
    const madVector product = {.axis = {
        .x = vectorA.axis.x * vectorB.axis.x,
        .y = vectorA.axis.y * vectorB.axis.y,
        .z = vectorA.axis.z * vectorB.axis.z,
    }};
    return product;
}

static inline float vectorSum(const madVector vector){
    return (vector.axis.x) + (vector.axis.y) + (vector.axis.z);
}

static inline madVector vectorNormalise(const madVector vector){
    #ifdef NORMAL_SQRT
        const float magReciprocal = 1.0f / sqrtf(VectorMagSquared(vector));
    #else  
        const float magReciprocal = InverseSquareRoot(VectorMagSquared(vector));
    #endif
        return vectorMultiplyScalar(vector, magReciprocal);
}

#ifndef NORMAL_SQRT

static inline float InverseSquareRoot(const float x){

    typedef union {
        float f;
        int32_t i;
    } Union32;

    Union32 union32 = {.f = x};
    union32.i = 0x5F1F1412 - (union32.i >> 1);
    return union32.f * (1.69000231f - 0.714158168f * x * union32.f * union32.f);
}

#endif

/**
 * Multiplies vector by given scalar 
*/
static inline madVector vectorMultiplyScalar(const madVector vector, const float scalar){
    const madVector product = {.axis = {
        .x = vector.axis.x * scalar,
        .y = vector.axis.y * scalar,
        .z = vector.axis.z * scalar,
    }};
    return product;

}






