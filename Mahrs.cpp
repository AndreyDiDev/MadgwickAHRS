// Main Mahrs file to control flow of algorithm 


/* Includes/Header Files ------------------------------------------------------------------*/
#include <math.h>
#include <float.h> 
// #include "MadgwickAHRS\MarhsHPP.hpp"
// #include "MarhsHPP.hpp"
#include "C:\Users\Andrey\Documents\AHRSRepo\MadgwickAHRS\MahrsHPP.hpp"
// #include "C:\Users\Andrey\Documents\AHRSRepo\MadgwickAHRS\MahrsHPP::

// using namespace MahrsHPP;

/* Definitions ------------------------------------------------------------------*/

// initial gain during initialisation 
#define INITIAL_GAIN (10.0f)

// initialisation period in seconds 
#define INITIALISATION_PERIOD (3.0f)

/**
 * @brief Cutoff frequency in Hz.
 */
#define CORNER_FREQUENCY (0.02f)

/**
 * @brief Timeout in seconds.
 */
#define TIMEOUT (5)

/**
 * @brief Threshold in degrees per second.
 */
#define THRESHOLD (3.0f)

/* Function Declarations ------------------------------------------------------------------*/

// static inline madVector halfGravity(const MahrsStruct *const mahrs);

// static inline madVector halfMagnetic(const MahrsStruct *const mahrs);

// static inline madVector feedback(const madVector sensor, const madVector reference);

// static inline int clamp(const int value, const int min, const int max);


/* Functions ------------------------------------------------------------------*/

// initialise 


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

void mahrsInitialisation(MahrsStruct *const mahrs){
    params parameters1 = {
        .algorithmGain = 0.5f,
        .gyroRange = 0.5f,
        .accelRejection = 90.0f,
        .magRejection = 90.0f,
        .recoveryTriggerPeriod = 0,
    };

    setParams(mahrs, &parameters1);
    madReset(mahrs);
}

// resets the ahrs 
void madReset(MahrsStruct *const mahrs){
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

// offset
void offsetInitialise(madOffset *const offset, const unsigned int sampleRate){
    offset->filterCoefficient = 2.0f * (float) M_PI * CORNER_FREQUENCY * (1.0f / (float) sampleRate);
    offset->timeout = TIMEOUT * sampleRate;
    offset->timer = 0;
    offset->gyroscopeOffset = VECTOR_ZERO;
}

madVector offsetUpdate(madOffset *const offset, madVector gyro){
        // Subtract offset from gyroscope measurement
    gyro = vectorSubtract(gyro, offset->gyroscopeOffset);

    // Reset timer if gyroscope not stationary
    if ((fabsf(gyro.axis.x) > THRESHOLD) || (fabsf(gyro.axis.y) > THRESHOLD) || (fabsf(gyro.axis.z) > THRESHOLD)) {
        offset->timer = 0;
        return gyro;
    }

    // Increment timer while gyroscope stationary
    if (offset->timer < offset->timeout) {
        offset->timer++;
        return gyro;
    }

    // Adjust offset if timer has elapsed
    offset->gyroscopeOffset = vectorAdd(offset->gyroscopeOffset, vectorMultiplyScalar(gyro, offset->filterCoefficient));

    return gyro;
}

//------------------------------------------------
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
        madReset(mahrs);
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

        // dont ignore accel during accel recovery
        if(mahrs->accelRecoveryTrigger > mahrs->accelRecoveryTimeout){
            mahrs->accelRecoveryTimeout = 0;
            mahrs->accelIgnored = false;
        } else {
            mahrs->accelRecoveryTimeout = mahrs->Parameters.recoveryTriggerPeriod;
        }

        mahrs->accelRecoveryTrigger = Clamp(mahrs->accelRecoveryTrigger, 0, mahrs->Parameters.recoveryTriggerPeriod);

        // apply accel feedback
        if(mahrs->accelIgnored == false){
            halfAccelFeedback = mahrs->halfAccelFeedback;
        }
    }

        // calculate magno feedback
        madVector halfMagneticFeedback = VECTOR_ZERO;
        mahrs->magnoIgnored = true;

        if(VectorIsZero(magno) == false){

            // calculate direction of mag field
            const madVector halfMagneticVector = HalfMagnetic(mahrs);

            // calculate magno feedback and scale by 0.5
            mahrs->halfMagFeedback = Feedback(vectorNormalise(vectorCrossProduct(halfGravityVector, magno)), halfMagneticVector);

            // dont ignore mango feedback if magnetic error below threshold
            if(mahrs->initialisation || ((VectorMagSquared(mahrs->halfMagFeedback) <= mahrs->Parameters.magRejection))){
                mahrs->magnoIgnored = false;
                mahrs->magneticRecoveryTrigger -= 9;
            } else {
                mahrs->magneticRecoveryTrigger += 1;
            }

            // dont ignore magno during magnetic recovery
            if(mahrs->magneticRecoveryTrigger > mahrs->magneticRecoveryTimeout){
                mahrs->magneticRecoveryTimeout = 0;
                mahrs->magnoIgnored = false;
            } else {
                mahrs->magneticRecoveryTimeout = mahrs->Parameters.recoveryTriggerPeriod;
            }
            mahrs->magneticRecoveryTrigger = Clamp(mahrs->magneticRecoveryTrigger, 0, mahrs->Parameters.recoveryTriggerPeriod);

            // apply magno feedback
            if (mahrs->magnoIgnored == false){
                halfMagneticFeedback = mahrs->halfMagFeedback;
            }
        }

        // convert gyro to rads per sec and scale by 0.5
        const madVector halfGyro = vectorMultiplyScalar(gyro, degToRads(0.5f));

        //apply feedback to gyro
        const madVector adjustedHalfGyro = vectorAdd(halfGyro, vectorMultiplyScalar(vectorAdd(halfAccelFeedback, halfMagneticFeedback), mahrs->rampedGain));

        // integrate rate of change of quaternion
        mahrs->quaternion = quaternionAdd(mahrs->quaternion, quaternionMultiplyVector(mahrs->quaternion, vectorMultiplyScalar(adjustedHalfGyro, deltaT)));

        // normalise quaternion
        mahrs->quaternion = quaternionNormalise(mahrs->quaternion);
    #undef Q
}

void updateNoMagnetometer(MahrsStruct *const mahrs, const madVector gyro, const madVector accel, const float deltaT){
    Update(mahrs, gyro, accel, VECTOR_ZERO, deltaT);

    // zero heading during initialisation
    if(mahrs->initialisation){
        setHeading(mahrs, 0.0f);
    }
}

void updateExternalHeading(MahrsStruct *const mahrs, const madVector gyro, const madVector accel, const float heading, const float deltaT){
    #define q mahrs->quaternion.element

    // calculate roll
    const float roll = atan2f(q.w * q.x + q.y * q.z, 0.5f -q.y * q.y - q.x * q.x);

    madVector magno = VECTOR_ZERO;

    // calculate magnetometer
    const float headingRads = degToRads(heading);
    const float sinHeadingRads = sinf(headingRads);
    magno = {.axis = {
        .x = cosf(headingRads),
        .y = -1.0f * cosf(roll) * sinHeadingRads,
        .z = sinHeadingRads * sinf(roll),
    }};

    // update algorithm
    Update(mahrs, gyro, accel, magno, deltaT);

    #undef q
}

void setHeading(MahrsStruct *const mahrs, const float heading){
    #define q mahrs->quaternion.element

    madQuaternion rotation = IDENTITY_QUATERNION;

    const float yaw = atan2f(q.w * q.z + q.x * q.y, 0.5f - q.y * q.y - q.z * q.z);
    const float halfYawMinusHeading = 0.5f * (yaw - degToRads(heading));
    rotation = {.element = {
        .w = cosf(halfYawMinusHeading),
        .x = 0.0f,
        .y = 0.0f,
        .z = -1.0f * sinf(halfYawMinusHeading),
    }};

    mahrs->quaternion = quaternionMultiply(rotation, mahrs->quaternion);
}

static inline madVector Feedback(const madVector sensor, const madVector reference){

    if(vectorDotProduct(sensor, reference) < 0.0f){ // if error is >90 degrees
        return vectorNormalise(vectorCrossProduct(sensor, reference));
    }

    return vectorCrossProduct(sensor, reference);
}

static inline madVector HalfMagnetic(const MahrsStruct *const mahrs){
    #define Q mahrs->quaternion.element

    madVector halfMagnetic = VECTOR_ZERO;

    // second col of transposed rotation matrix scaled by -0.5
    halfMagnetic = {.axis = {
        .x = -1.0f * (Q.x * Q.y + Q.w * Q.z),
        .y = 0.5f - Q.w * Q.w - Q.y * Q.y,
        .z = Q.w * Q.x - Q.y * Q.z,
    }}; 

    return halfMagnetic;

    // maybe compiler warning 
    #undef Q
}

// gives half gravity in the NED convention 
static inline madVector HalfGravity(const MahrsStruct *const mahrs){
    #define q mahrs->quaternion.element

    madVector halfGravity = VECTOR_ZERO;
    // third col scaled by -0.5
    halfGravity = {.axis ={
        .x = q.x * q.z - q.w * q.y,
        .y = q.y * q.z + q.w * q.x,
        .z = q.w * q.w - 0.5f + q.z * q.z,
    }};
    return halfGravity;

    #undef q
}

madVector getLinearAcceleration(const MahrsStruct *const mahrs){
    #define q mahrs->quaternion.element

    madVector gravityVector = VECTOR_ZERO;
    // calc gravity from the sensor's coordinate system
    gravityVector = {.axis ={
        .x = 2.0f * (q.x * q.z - q.w * q.y),
        .y = 2.0f * (q.y * q.z + q.w * q.x),
        .z = 2.0f * (q.w * q.w - 0.5f + q.z * q.z),
    }}; // third column is transposed 

    return vectorAdd(mahrs->accel, gravityVector);

    #undef q
}

// math --------------------------------------------------------------
static inline int Clamp(const int value, const int min, const int max){
    if(value < min){
        return min;
    }

    if(value > max){
        return max;
    }

    return value;
}

static inline float degToRads(const float degrees){
    return degrees *((float) M_PI / 180.0f);
}

/**
 * @brief Converts radians to degrees.
 * @param radians Radians.
 * @return Degrees.
 */
static inline float radsToDeg(const float radians) {
    return radians * (180.0f / (float) M_PI);
}

static inline float Asin(const float value) {
    if (value <= -1.0f) {
        return (float) M_PI / -2.0f;
    }
    if (value >= 1.0f) {
        return (float) M_PI / 2.0f;
    }
    return asinf(value);
}

/**
 * @brief Returns the vector magnitude.
 * @param vector Vector.
 * @return Vector magnitude.
 */
static inline float vectorMagnitude(const madVector vector) {
    return sqrtf(vectorMagnitudeSquared(vector));
}

/**
 * @brief Returns the vector magnitude squared.
 * @param vector Vector.
 * @return Vector magnitude squared.
 */
static inline float vectorMagnitudeSquared(const madVector vector) {
    return vectorSum(vectorHadamarProduct(vector, vector));
}

static inline madVector vectorAdd(const madVector vectorA, const madVector vectorB){
    #define A vectorA.axis
    #define B vectorB.axis

    madVector product = VECTOR_ZERO;

    product = {.axis={
        .x = A.x + B.x,
        .y = A.y + B.y,
        .z = A.z + B.z,
    }};
    return product;

    #undef A
    #undef B
}

static inline madVector vectorCrossProduct(const madVector vectorA, const madVector vectorB){
    #define A vectorA.axis
    #define B vectorB.axis

    madVector product = VECTOR_ZERO;

    product = {.axis = {
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
    madVector product = VECTOR_ZERO;

    product = {.axis = {
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
    float magReciprocal = 0.0f;

    #ifdef NORMAL_SQRT
        magReciprocal = 1.0f / sqrtf(VectorMagSquared(vector));
    #else  
        magReciprocal = InverseSquareRoot(VectorMagSquared(vector));
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
    madVector product = VECTOR_ZERO;

    product = {.axis = {
        .x = vector.axis.x * scalar,
        .y = vector.axis.y * scalar,
        .z = vector.axis.z * scalar,
    }};
    return product;

}

/**
 * @brief Returns vector B subtracted from vector A.
 * @param vectorA Vector A.
 * @param vectorB Vector B.
 * @return Vector B subtracted from vector A.
 */
static inline madVector vectorSubtract(const madVector vectorA, const madVector vectorB) {
    madVector product = VECTOR_ZERO;

    product = {.axis = {
            .x = vectorA.axis.x - vectorB.axis.x,
            .y = vectorA.axis.y - vectorB.axis.y,
            .z = vectorA.axis.z - vectorB.axis.z,
    }};
    return product;
}

static inline madQuaternion quaternionAdd(const madQuaternion quaternionA, const madQuaternion quaternionB){
    #define A quaternionA.element
    #define B quaternionB.element

    madQuaternion product = IDENTITY_QUATERNION;

    product = {.element ={
        .w = A.w + B.w,
        .x = A.x + B.x,
        .y = A.y + B.y,
        .z = A.z + B.z,
    }};

    return product;

    #undef A
    #undef B
}

static inline madQuaternion quaternionMultiplyVector(const madQuaternion quaternion, const madVector vector){
    #define q quaternion.element
    #define v vector.axis

    madQuaternion product = IDENTITY_QUATERNION;

    product = {.element = {
        .w = -q.x * v.x - q.y * v.y - q.z * v.z,
        .x = q.w * v.x + q.y * v.z - q.z * v.y,
        .y = q.w * v.y - q.x * v.z + q.z * v.x,
        .z = q.w * v.z + q.x * v.y - q.y * v.x,
    }};
    return product;

    #undef q
    #undef v
}

static inline madQuaternion quaternionNormalise(const madQuaternion quaternion){
    #define q quaternion.element

    #ifdef NORMAL_SQRT
        const float magReciprocal = 1.0f / sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    #else
        const float magReciprocal = InverseSquareRoot(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    #endif 

    madQuaternion product = IDENTITY_QUATERNION;

    product = {.element = {
        .w = q.w * magReciprocal,
        .x = q.x * magReciprocal,
        .y = q.y * magReciprocal,
        .z = q.z * magReciprocal,
    }};

    return product;

    #undef q
}

static inline madQuaternion quaternionMultiply(const madQuaternion quaternionA, const madQuaternion quaternionB) {
    #define A quaternionA.element
    #define B quaternionB.element

    madQuaternion product = IDENTITY_QUATERNION;

    product = {.element = {
            .w = A.w * B.w - A.x * B.x - A.y * B.y - A.z * B.z,
            .x = A.w * B.x + A.x * B.w + A.y * B.z - A.z * B.y,
            .y = A.w * B.y - A.x * B.z + A.y * B.w + A.z * B.x,
            .z = A.w * B.z + A.x * B.y - A.y * B.x + A.z * B.w,
    }};
    return product;

#undef A
#undef B
}

// helper functions
madQuaternion getQuaternion(const MahrsStruct *const mahrs) {
    return mahrs->quaternion;
}

void setQuaternion(MahrsStruct *const mahrs, const madQuaternion quaternion) {
    mahrs->quaternion = quaternion;
}

static inline madMatrix quaternionToMatrix(const madQuaternion quaternion) {
#define Q quaternion.element
    const float qwqw = Q.w * Q.w; // calculate common terms to avoid repeated operations
    const float qwqx = Q.w * Q.x;
    const float qwqy = Q.w * Q.y;
    const float qwqz = Q.w * Q.z;
    const float qxqy = Q.x * Q.y;
    const float qxqz = Q.x * Q.z;
    const float qyqz = Q.y * Q.z;

    madMatrix matrix = IDENTITY_MATRIX;

    matrix = {.element = {
            .xx = 2.0f * (qwqw - 0.5f + Q.x * Q.x),
            .xy = 2.0f * (qxqy - qwqz),
            .xz = 2.0f * (qxqz + qwqy),
            .yx = 2.0f * (qxqy + qwqz),
            .yy = 2.0f * (qwqw - 0.5f + Q.y * Q.y),
            .yz = 2.0f * (qyqz - qwqx),
            .zx = 2.0f * (qxqz - qwqy),
            .zy = 2.0f * (qyqz + qwqx),
            .zz = 2.0f * (qwqw - 0.5f + Q.z * Q.z),
    }};
    return matrix;
#undef Q
}

static inline madEuler quaternionToEuler(const madQuaternion quaternion) {
#define Q quaternion.element
    const float halfMinusQySquared = 0.5f - Q.y * Q.y; // calculate common terms to avoid repeated operations

    madEuler euler = EULER_ZERO;

    euler = {.angle = {
            .roll = radsToDeg(atan2f(Q.w * Q.x + Q.y * Q.z, halfMinusQySquared - Q.x * Q.x)),
            .pitch = radsToDeg(Asin(2.0f * (Q.w * Q.y - Q.z * Q.x))),
            .yaw = radsToDeg(atan2f(Q.w * Q.z + Q.x * Q.y, halfMinusQySquared - Q.z * Q.z)),
    }};
    return euler;
#undef Q
}

// madVector getLinearAcceleration(const MahrsStruct *const mahrs) {
// #define Q mahrs->quaternion.element

//     // Calculate gravity in the sensor coordinate frame, using NED convention 
//     const madVector gravity = {.axis = {
//             .x = 2.0f * (Q.x * Q.z - Q.w * Q.y),
//             .y = 2.0f * (Q.y * Q.z + Q.w * Q.x),
//             .z = 2.0f * (Q.w * Q.w - 0.5f + Q.z * Q.z),
//     }}; // third column of transposed rotation matrix

//     // Remove gravity from accelerometer measurement
//     return vectorAdd(mahrs->accel, gravity);

// #undef Q
// }

madVector getEarthAcceleration(const MahrsStruct *const mahrs) {
    #define Q mahrs->quaternion.element
    #define A mahrs->accel.axis

    // Calculate accelerometer measurement in the Earth coordinate frame
    const float qwqw = Q.w * Q.w;
    const float qwqx = Q.w * Q.x;
    const float qwqy = Q.w * Q.y;
    const float qwqz = Q.w * Q.z;
    const float qxqy = Q.x * Q.y;
    const float qxqz = Q.x * Q.z;
    const float qyqz = Q.y * Q.z;

    madVector accelerometerVector = VECTOR_ZERO;

    accelerometerVector = {.axis = {
            .x = 2.0f * ((qwqw - 0.5f + Q.x * Q.x) * A.x + (qxqy - qwqz) * A.y + (qxqz + qwqy) * A.z),
            .y = 2.0f * ((qxqy + qwqz) * A.x + (qwqw - 0.5f + Q.y * Q.y) * A.y + (qyqz - qwqx) * A.z),
            .z = 2.0f * ((qxqz - qwqy) * A.x + (qyqz + qwqx) * A.y + (qwqw - 0.5f + Q.z * Q.z) * A.z),
    }};

    // Remove gravity from accelerometer measurement
    accelerometerVector.axis.z += 1.0f;

    return accelerometerVector;

    #undef Q
    #undef A
}

madFlags getFlags(const MahrsStruct *const mahrs) {

    madFlags flags = {
            .initialization = mahrs->initialisation,
            .angularRateRecovery = mahrs->angularRateRecovery,
            .accelRecovery = mahrs->accelRecoveryTrigger > mahrs->accelRecoveryTimeout,
            .magneticRecovery= mahrs->magneticRecoveryTrigger > mahrs->magneticRecoveryTimeout,
    };

    return flags;
}

madInternalStates getInternalStates(const MahrsStruct *const mahrs){
    madInternalStates internalStates = {
            .accelError = radsToDeg(Asin(2.0f * vectorMagnitude(mahrs->halfAccelFeedback))),
            .accelIgnored = mahrs->accelIgnored,
            .accelRecoveryTrigger = mahrs->Parameters.recoveryTriggerPeriod == 0 ? 0.0f : (float) mahrs->accelRecoveryTrigger / (float) mahrs->Parameters.recoveryTriggerPeriod,
            .magneticError = radsToDeg(Asin(2.0f * vectorMagnitude(mahrs->halfMagFeedback))),
            .magnoIgnored = mahrs->magnoIgnored,
            .magneticRecoveryTrigger = mahrs->Parameters.recoveryTriggerPeriod == 0 ? 0.0f : (float) mahrs->magneticRecoveryTrigger / (float) mahrs->Parameters.recoveryTriggerPeriod,
    };
    return internalStates;
}

float compassCalculateHeading(const madVector accelerometer, const madVector magnetometer) {
    const madVector up = vectorMultiplyScalar(accelerometer, -1.0f);
    const madVector west = vectorNormalise(vectorCrossProduct(up, magnetometer));
    const madVector north = vectorNormalise(vectorCrossProduct(west, up));
    return radsToDeg(atan2f(west.axis.x, north.axis.x));
}







