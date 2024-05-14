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
