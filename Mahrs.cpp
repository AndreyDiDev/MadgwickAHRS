// Main Mahrs file to control flow of algorithm 


/* Includes/Header Files ------------------------------------------------------------------*/
#include <math.h>
#include <float.h> 
#include "MadgwickAHRS\MarhsHPP.hpp"

/* Definitions ------------------------------------------------------------------*/

// initial gain during initialisation 
#define INITIAL_GAIN (10.0f);

// initialisation period in seconds 
#define INITIALISATION_PERIOD (3.0f);

/* Function Declarations ------------------------------------------------------------------*/

static inline madVector halfGravity(const Mahrs *const mahrs);

static inline madVector halfMagnetic(const Mahrs *const mahrs);

static inline madVector feedback(const madVector sensor, const madVector reference);

static inline int clamp(const int value, const int min, const int max);


/* Functions ------------------------------------------------------------------*/

// initialise 
void mahrsInitialisation(Mahrs *const mahrs){
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

void setParams(Mahrs *const mahrs, const params *const parameters){
    mahrs->Parameters.algorithmGain = parameters->algorithmGain;
    mahrs->Parameters.gyroRange = parameters->gyroRange== 0.0f ? FLT_MAX : powf(0.5f * sinf(degToRads(parameters->accelRejection)), 2);
    mahrs->Parameters.accelRejection = parameters->accelRejection == 0.0f ? FLT_MAX : powf(0.5f * sinf(degToRads(parameters->accelRejection)), 2);
    mahrs->Parameters.magRejection = parameters->magRejection == 0.0f ? FLT_MAX : powf(0.5f * sinf(degToRads(parameters->magRejection)), 2);
    mahrs->Parameters.recoveryTriggerPeriod = parameters->recoveryTriggerPeriod;
    mahrs->accelRecoveryTimeout = mahrs->Parameters.recoveryTriggerPeriod;
    mahrs->magneticRecoveryTimeout = mahrs->parameters.recoveryTriggerPeriod;

    if(parameters->algorithmGain == 0.0f) || (parameters->recoveryTriggerPeriod == 0)){
        mahrs->Parameters.accelRejection = FLT_MAX;
        mahrs->Parameters.magneticRejection = FLT_MAX;
    }

    if(mahrs->initialisation == false){
        mahrs->rampedGain = mahrs->Parameters.algorithmGain;
    }

    mahrs->rampedGainStep = (INITIAL_GAIN - mahrs->Parameters.algorithmGain) / INITIALISATION_PERIOD;
}

void reset(Mahrs *const mahrs){

}

// with magno
void update() {

}

// update without magno



// math
static inline float degToRads(const float degrees){
    return degrees *((float) 3.14 / 180.0f);
}




