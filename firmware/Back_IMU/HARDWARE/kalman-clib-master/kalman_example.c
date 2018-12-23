/*!
* \brief This is demo code for the Kalman Filter factory includes.
*        It sets up some fake Kalman filters with measurements.
*/

#ifdef DOXYGEN

/************************************************************************/
/* Create the first filter                                              */
/************************************************************************/

// create the filter structure
#define KALMAN_NAME example
#define KALMAN_NUM_STATES 4
#define KALMAN_NUM_INPUTS 0
#include "kalman_factory_filter.h"

// create the measurement structure
#define KALMAN_MEASUREMENT_NAME gyroscope
#define KALMAN_NUM_MEASUREMENTS 3
#include "kalman_factory_measurement.h"

// create the measurement structure
#define KALMAN_MEASUREMENT_NAME accelerometer
#define KALMAN_NUM_MEASUREMENTS 3
#include "kalman_factory_measurement.h"

// clean up
#include "kalman_factory_cleanup.h"

/************************************************************************/
/* Create the second filter                                             */
/************************************************************************/

// create the filter structure
#define KALMAN_NAME roflcopter
#define KALMAN_NUM_STATES 4
#define KALMAN_NUM_INPUTS 3
#include "kalman_factory_filter.h"

// create the measurement structure
#define KALMAN_MEASUREMENT_NAME lollercoaster
#define KALMAN_NUM_MEASUREMENTS 3
#include "kalman_factory_measurement.h"

// clean up
#include "kalman_factory_cleanup.h"

/************************************************************************/
/* Initialization functions                                             */
/************************************************************************/

/*!
* An example for the Kalman Filter factories
*/
static void test_kalman_1()
{
    kalman_filter_example_init();
    kalman_filter_example_measurement_gyroscope_init();
    kalman_filter_example_measurement_accelerometer_init();

    kalman_filter_example.x.data[0] = 1;
    kalman_filter_example_measurement_gyroscope.z.data[0] = 1;
    kalman_filter_example_measurement_accelerometer.z.data[0] = 1;
}

/*!
* An example for the Kalman Filter factories
*/
static void test_kalman_2()
{
    kalman_filter_roflcopter_init();
    kalman_filter_roflcopter_measurement_lollercoaster_init();
    
    kalman_filter_roflcopter.x.data[0] = 1;
    kalman_filter_roflcopter_measurement_lollercoaster.z.data[0] = 1;
}

#endif