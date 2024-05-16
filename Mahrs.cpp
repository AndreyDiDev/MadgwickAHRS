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
    #define Q mahrs->madQuaternion.element

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
        // TO DO
    }

}

// update without magno


static inline madVector HalfGravity(const MahrsStruct *const mahrs){
//TO DO
}

// math
static inline float degToRads(const float degrees){
    return degrees *((float) 3.14 / 180.0f);
}

/**
 * Returns true if vetor is zero
*/
static inline bool VectorIsZero(const madVector vector){
    return (vector.axis.x == 0.0f) && (vector.axis.y == 0.0f) && (vector.axis.z == 0.0f);
}




