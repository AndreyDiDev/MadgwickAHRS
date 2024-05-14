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

// where is sampling period 

/* Macros/Enums ------------------------------------------------------------*/


/* Class ------------------------------------------------------------------*/
class MahrsHPP
{
	public:
	protected:

	    void Run(void* pvParams);    // Main run code

	    // void HandleCommand(Command& cm);

	    // Data
	    // AccelGyroMagnetismData* data;

	private:

};

#endif
