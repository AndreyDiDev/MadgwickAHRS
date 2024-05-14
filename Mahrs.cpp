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

}

void reset(Mahrs *const mahrs){

}

// with magno
void update() {

}

// update without magno




