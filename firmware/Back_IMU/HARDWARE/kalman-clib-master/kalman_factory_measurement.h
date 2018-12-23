/*!
* \brief Initializes a named Kalman filter measurement structure.
*
* This include requires the two defines {\ref KALMAN_MEASUREMENT_NAME} and {\ref KALMAN_NUM_MEASUREMENTS}
* to be set to the base name of the Kalman Filter measurement and to the number of measured outputs,
* as well as all defines from \ref {kalman_factory_filter.h}
*
* It then will instantiate the buffers required for H, R, z and all helper matrices and vectors, 
* as well as the structure for the Kalman filter measurement and the initialization method.
*
* Suppose a Kalman filter named "direction" for which a measurement named "gyroscope" shall be crated 
* which has three measured outputs. You would start by defining the required macros
*
* \code{.c}
* #define KALMAN_MEASUREMENT_NAME gyroscope
* #define KALMAN_NUM_MEASUREMENTS 3
* \endcode
*
* After that, this file must be included
*
* \code{.c}
* #include "kalman_factory_measurement.h"
* \endcode
*
* At this point, the structure \c kalman_filter_direction_measurement_gyroscope will be created (statically) along with
* all the required buffers (i.e. \c kalman_filter_direction_measurement_gyroscope_H_buffer, etc.) and the matrices
* will be initialized and set with the correct dimensions.
*
* In addition, a parameterless static initialization function \code {kalman_filter_direction_measurement_gyroscope_init()} will
* be created which you will need to call manually in order to set up the measurement.
*
* A full example would be

* \code{.c}
* #define KALMAN_NAME example
* #define KALMAN_NUM_STATES 4
* #define KALMAN_NUM_INPUTS 0
* #include "kalman_factory_filter.h"
*
* #define KALMAN_MEASUREMENT_NAME gyroscope
* #define KALMAN_NUM_MEASUREMENTS 3
* #include "kalman_factory_measurement.h"

* #include "kalman_factory_cleanup.h"

* void test_kalman()
* {
*   kalman_filter_example_init();
*   kalman_filter_example_measurement_gyroscope_init();
*
*   kalman_filter_example.x.data[0] = 1;
*   kalman_filter_example_measurement_gyroscope.z.data[0] = 1;
* }
* \endcode
*
* In order to force creation of separate auxiliary buffers (thus preventing buffer reuse), MEASUREMENT_FORCE_NEW_BUFFERS can be defined
* prior to inclusion of this file.
*/

#ifndef MEASUREMENT_FORCE_NEW_BUFFERS
#define MEASUREMENT_FORCE_NEW_BUFFERS 0
#endif

/************************************************************************/
/* Check for inputs                                                     */
/************************************************************************/

#ifndef KALMAN_NAME
#error KALMAN_NAME needs to be defined prior to inclusion of this file. Did you include kalman_factory_filter.h?
#endif

#ifndef KALMAN_MEASUREMENT_NAME
#error KALMAN_MEASUREMENT_NAME needs to be defined prior to inclusion of this file.
#endif

#ifndef KALMAN_NUM_STATES
#error KALMAN_NUM_STATES needs to be defined prior to inclusion of this file.
#elif KALMAN_NUM_STATES <= 0
#error KALMAN_NUM_STATES must be a positive integer
#endif

#ifndef KALMAN_NUM_MEASUREMENTS
#error KALMAN_NUM_MEASUREMENTS needs to be defined prior to inclusion of this file.
#elif KALMAN_NUM_MEASUREMENTS < 0
#error KALMAN_NUM_MEASUREMENTS must be a positive integer or zero if no inputs are used
#endif

#pragma message("** Instantiating Kalman filter \"" STRINGIFY(KALMAN_NAME) "\" measurement \"" STRINGIFY(KALMAN_MEASUREMENT_NAME) "\" with " STRINGIFY(KALMAN_NUM_MEASUREMENTS) " measured outputs")

#if MEASUREMENT_FORCE_NEW_BUFFERS
#pragma message("MEASUREMENT_FORCE_NEW_BUFFERS was set. Forcing separate auxiliary buffers.")
#endif

/************************************************************************/
/* Prepare dimensions                                                   */
/************************************************************************/

// H maps from state to measurement
#define __KALMAN_H_ROWS     KALMAN_NUM_MEASUREMENTS
#define __KALMAN_H_COLS     KALMAN_NUM_STATES

// z contains the measurements
#define __KALMAN_z_ROWS     KALMAN_NUM_MEASUREMENTS
#define __KALMAN_z_COLS     1

// R models the measurement uncertainties / the process noise covariance
#define __KALMAN_R_ROWS     KALMAN_NUM_MEASUREMENTS
#define __KALMAN_R_COLS     KALMAN_NUM_MEASUREMENTS

// y contains the innovation, i.e. difference from predicted to measured values
#define __KALMAN_y_ROWS     KALMAN_NUM_MEASUREMENTS
#define __KALMAN_y_COLS     1

// S contains the innovation covariance (residual covariance)
#define __KALMAN_S_ROWS     KALMAN_NUM_MEASUREMENTS
#define __KALMAN_S_COLS     KALMAN_NUM_MEASUREMENTS

// K models the state covariance gain
#define __KALMAN_K_ROWS     KALMAN_NUM_STATES
#define __KALMAN_K_COLS     KALMAN_NUM_MEASUREMENTS

// auxiliary buffer size
#define __KALMAN_maux_ROWS      ((KALMAN_NUM_STATES > KALMAN_NUM_MEASUREMENTS) ? KALMAN_NUM_STATES : KALMAN_NUM_MEASUREMENTS)
#define __KALMAN_maux_COLS      1
#define __USE_BUFFER_AUX        ((__KALMAN_maux_ROWS * __KALMAN_maux_COLS) <= __KALMAN_aux_size)

// inverted S buffer
#define __KALMAN_Sinv_ROWS      __KALMAN_S_ROWS
#define __KALMAN_Sinv_COLS      __KALMAN_S_COLS

// temporary HxP buffer
#define __KALMAN_tempHP_ROWS    __KALMAN_H_ROWS
#define __KALMAN_tempHP_COLS    __KALMAN_H_COLS

// temporary PxH' buffer
#define __KALMAN_tempPHt_ROWS    __KALMAN_H_COLS // [sic]
#define __KALMAN_tempPHt_COLS    __KALMAN_H_ROWS // [sic]

// temporary Kx(HxP) buffer
#define __KALMAN_tempKHP_ROWS   __KALMAN_P_ROWS
#define __KALMAN_tempKHP_COLS   __KALMAN_P_COLS

/************************************************************************/
/* Name macro                                                           */
/************************************************************************/

#define KALMAN_MEASUREMENT_BASENAME_HELPER2(basename, measname)                 basename ## _measurement_ ## measname
#define KALMAN_MEASUREMENT_BASENAME_HELPER(basename, measname)                  KALMAN_MEASUREMENT_BASENAME_HELPER2(basename, measname)
#define KALMAN_MEASUREMENT_BASENAME                                             KALMAN_MEASUREMENT_BASENAME_HELPER(KALMAN_FILTER_BASENAME, KALMAN_MEASUREMENT_NAME)

#define KALMAN_MEASUREMENT_STRUCT_HELPER2(name)                                 #name
#define KALMAN_MEASUREMENT_STRUCT_HELPER(name)                                  KALMAN_MEASUREMENT_STRUCT_HELPER2(name)
#define KALMAN_MEASUREMENT_STRUCT_NAME                                          KALMAN_MEASUREMENT_STRUCT_HELPER(KALMAN_FILTER_BASENAME)

#define KALMAN_MEASUREMENT_FUNCTION_HELPER2(basename, measname, element)        basename ## _measurement_ ## measname ## _ ## element
#define KALMAN_MEASUREMENT_FUNCTION_HELPER(basename, measname, element)         KALMAN_MEASUREMENT_FUNCTION_HELPER2(basename, measname, element)
#define KALMAN_MEASUREMENT_FUNCTION_NAME(action)                                KALMAN_MEASUREMENT_FUNCTION_HELPER(KALMAN_FILTER_BASENAME, KALMAN_MEASUREMENT_NAME, action)

#define KALMAN_MEASUREMENT_BUFFER_HELPER2(basename, measname, element)          basename ## _measurement_ ## measname ## _ ## element ## _buffer
#define KALMAN_MEASUREMENT_BUFFER_HELPER(basename, measname, element)           KALMAN_MEASUREMENT_BUFFER_HELPER2(basename, measname, element)
#define KALMAN_MEASUREMENT_BUFFER_NAME(element)                                 KALMAN_MEASUREMENT_BUFFER_HELPER(KALMAN_FILTER_BASENAME, KALMAN_MEASUREMENT_NAME, element)

/************************************************************************/
/* Construct Kalman filter measurement buffers                          */
/************************************************************************/

#include "compiler.h"
#include "matrix.h"
#include "kalman.h"

#define __KALMAN_BUFFER_H   KALMAN_MEASUREMENT_BUFFER_NAME(H)
#define __KALMAN_BUFFER_R   KALMAN_MEASUREMENT_BUFFER_NAME(R)
#define __KALMAN_BUFFER_z   KALMAN_MEASUREMENT_BUFFER_NAME(z)

#define __KALMAN_BUFFER_K   KALMAN_MEASUREMENT_BUFFER_NAME(K)
#define __KALMAN_BUFFER_S   KALMAN_MEASUREMENT_BUFFER_NAME(S)
#define __KALMAN_BUFFER_y   KALMAN_MEASUREMENT_BUFFER_NAME(y)

#pragma message("Creating Kalman measurement H buffer: " STRINGIFY(__KALMAN_BUFFER_H))
static matrix_data_t __KALMAN_BUFFER_H[__KALMAN_H_ROWS * __KALMAN_H_COLS];

#pragma message("Creating Kalman measurement R buffer: " STRINGIFY(__KALMAN_BUFFER_R))
static matrix_data_t __KALMAN_BUFFER_R[__KALMAN_R_ROWS * __KALMAN_R_COLS];

#pragma message("Creating Kalman measurement z buffer: " STRINGIFY(__KALMAN_BUFFER_z))
static matrix_data_t __KALMAN_BUFFER_z[__KALMAN_z_ROWS * __KALMAN_z_COLS];

#pragma message("Creating Kalman measurement K buffer: " STRINGIFY(__KALMAN_BUFFER_K))
static matrix_data_t __KALMAN_BUFFER_K[__KALMAN_K_ROWS * __KALMAN_K_COLS];

#pragma message("Creating Kalman measurement S buffer: " STRINGIFY(__KALMAN_BUFFER_S))
static matrix_data_t __KALMAN_BUFFER_S[__KALMAN_S_ROWS * __KALMAN_S_COLS];

#pragma message("Creating Kalman measurement y buffer: " STRINGIFY(__KALMAN_BUFFER_y))
static matrix_data_t __KALMAN_BUFFER_y[__KALMAN_y_ROWS * __KALMAN_y_COLS];

/************************************************************************/
/* Construct Kalman filter measurement buffers: Temporaries             */
/************************************************************************/

// re-use filter auxiliary buffer if it is large enough
#if __USE_BUFFER_AUX && !MEASUREMENT_FORCE_NEW_BUFFERS

#define __KALMAN_BUFFER_maux     __KALMAN_BUFFER_aux
#pragma message("Re-using Kalman aux buffer for measurement aux buffer: " STRINGIFY(__KALMAN_BUFFER_maux))

#else

#define __KALMAN_BUFFER_maux     KALMAN_MEASUREMENT_BUFFER_NAME(aux)
#define __KALMAN_maux_size       (__KALMAN_maux_ROWS * __KALMAN_maux_COLS)

#pragma message("Creating Kalman measurement aux buffer: " STRINGIFY(__KALMAN_BUFFER_maux))
static matrix_data_t __KALMAN_BUFFER_maux[__KALMAN_maux_size];

#endif

// create buffer for inverted s
#define __KALMAN_Sinv_size      (__KALMAN_Sinv_ROWS * __KALMAN_Sinv_COLS)

#if __KALMAN_Sinv_size <= __KALMAN_tempPBQ_size && !MEASUREMENT_FORCE_NEW_BUFFERS

#define __KALMAN_BUFFER_Sinv  __KALMAN_BUFFER_tempPBQ
#pragma message("Re-using Kalman filter temporary P/BQ buffer for temporary S-inverted buffer: " STRINGIFY(__KALMAN_BUFFER_Sinv))

#else

#define __KALMAN_BUFFER_Sinv     KALMAN_MEASUREMENT_BUFFER_NAME(Sinv)
#pragma message("Creating Kalman measurement temporary S-inverted buffer: " STRINGIFY(__KALMAN_BUFFER_Sinv))
static matrix_data_t __KALMAN_BUFFER_Sinv[__KALMAN_Sinv_size];

#endif

// create buffer for HxP
#define __KALMAN_tempHP_size    (__KALMAN_tempHP_ROWS * __KALMAN_tempHP_COLS)

#define __KALMAN_BUFFER_tempHP  KALMAN_MEASUREMENT_BUFFER_NAME(tempHP)
#pragma message("Creating Kalman measurement temporary HxP buffer: " STRINGIFY(__KALMAN_BUFFER_tempHP))
static matrix_data_t __KALMAN_BUFFER_tempHP[__KALMAN_tempHP_size];

// create buffer for PxH'
#define __KALMAN_tempPHt_size    (__KALMAN_tempPHt_ROWS * __KALMAN_tempPHt_COLS)

#if !MEASUREMENT_FORCE_NEW_BUFFERS

#define __KALMAN_BUFFER_tempPHt  __KALMAN_BUFFER_tempHP
#pragma message("Re-using Kalman measurement temporary HxP for temporary PxH' buffer: " STRINGIFY(__KALMAN_BUFFER_tempPHt))

#else

#define __KALMAN_BUFFER_tempPHt  KALMAN_MEASUREMENT_BUFFER_NAME(tempPHt)
#pragma message("Creating Kalman measurement temporary PxH' buffer: " STRINGIFY(__KALMAN_BUFFER_tempPHt))           // TODO: reuse HxP buffer!
static matrix_data_t __KALMAN_BUFFER_tempPHt[__KALMAN_tempPHt_size];

#endif

// create Kx(HxP) buffer
#define __KALMAN_tempKHP_size    (__KALMAN_tempKHP_ROWS * __KALMAN_tempKHP_COLS)

#if (__KALMAN_tempKHP_size <= __KALMAN_tempPBQ_size) && !MEASUREMENT_FORCE_NEW_BUFFERS

#define __KALMAN_BUFFER_tempKHP     __KALMAN_BUFFER_tempPBQ
#pragma message("Re-using Kalman filter temporary P/BQ buffer for measurement temporary Kx(HxP)  buffer: " STRINGIFY(__KALMAN_BUFFER_tempKHP))

#else

#define __KALMAN_BUFFER_tempKHP  KALMAN_MEASUREMENT_BUFFER_NAME(tempKHP)
#pragma message("Creating Kalman measurement temporary Kx(HxP) buffer: " STRINGIFY(__KALMAN_BUFFER_tempKHP))
static matrix_data_t __KALMAN_BUFFER_tempKHP[__KALMAN_tempKHP_size];

#endif

/************************************************************************/
/* Construct Kalman filter measurement                                  */
/************************************************************************/

#pragma message("Creating Kalman measurement structure: " STRINGIFY(KALMAN_MEASUREMENT_BASENAME))
static kalman_measurement_t KALMAN_MEASUREMENT_BASENAME;

#pragma message ("Creating Kalman measurement initialization function: " STRINGIFY(KALMAN_MEASUREMENT_FUNCTION_NAME(init()) ))

/*!
* \brief Initializes the Kalman Filter measurement
* \return Pointer to the measurement.
*/
static kalman_measurement_t* KALMAN_MEASUREMENT_FUNCTION_NAME(init)()
{
    int i;
    for (i = 0; i < __KALMAN_z_ROWS * __KALMAN_z_COLS; ++i) { __KALMAN_BUFFER_z[i] = 0; }
    for (i = 0; i < __KALMAN_H_ROWS * __KALMAN_H_COLS; ++i) { __KALMAN_BUFFER_H[i] = 0; }
    for (i = 0; i < __KALMAN_R_ROWS * __KALMAN_R_COLS; ++i) { __KALMAN_BUFFER_R[i] = 0; }
    for (i = 0; i < __KALMAN_y_ROWS * __KALMAN_y_COLS; ++i) { __KALMAN_BUFFER_y[i] = 0; }
    for (i = 0; i < __KALMAN_S_ROWS * __KALMAN_S_COLS; ++i) { __KALMAN_BUFFER_S[i] = 0; }
    for (i = 0; i < __KALMAN_K_ROWS * __KALMAN_K_COLS; ++i) { __KALMAN_BUFFER_K[i] = 0; }

    kalman_measurement_initialize(&KALMAN_MEASUREMENT_BASENAME, KALMAN_NUM_STATES, KALMAN_NUM_MEASUREMENTS, __KALMAN_BUFFER_H, __KALMAN_BUFFER_z, __KALMAN_BUFFER_R, 
                                  __KALMAN_BUFFER_y, __KALMAN_BUFFER_S, __KALMAN_BUFFER_K,
                                  __KALMAN_BUFFER_maux, __KALMAN_BUFFER_Sinv, __KALMAN_BUFFER_tempHP, __KALMAN_BUFFER_tempPHt, __KALMAN_BUFFER_tempKHP);
    return &KALMAN_MEASUREMENT_BASENAME;
}

/************************************************************************/
/* Clean up                                                             */
/************************************************************************/

#undef KALMAN_MEASUREMENT_NAME
#undef KALMAN_NUM_MEASUREMENTS

#undef KALMAN_MEASUREMENT_BASENAME_HELPER2
#undef KALMAN_MEASUREMENT_BASENAME_HELPER
#undef KALMAN_MEASUREMENT_BASENAME

#undef KALMAN_MEASUREMENT_FUNCTION_HELPER2
#undef KALMAN_MEASUREMENT_FUNCTION_HELPER
#undef KALMAN_MEASUREMENT_FUNCTION_NAME

#undef KALMAN_MEASUREMENT_BUFFER_NAME
#undef KALMAN_MEASUREMENT_BUFFER_HELPER
#undef KALMAN_MEASUREMENT_BUFFER_HELPER2

#undef KALMAN_MEASUREMENT_STRUCT_HELPER2
#undef KALMAN_MEASUREMENT_STRUCT_HELPER

#undef __KALMAN_BUFFER_H
#undef __KALMAN_BUFFER_R
#undef __KALMAN_BUFFER_z

#undef __KALMAN_BUFFER_K
#undef __KALMAN_BUFFER_S
#undef __KALMAN_BUFFER_y

// TODO: instead of cleaning up the temporary buffers here, clean them up in kalman_factory_cleanup.h. This way, the largest buffers can be reused in other measurement definitions.

#undef __KALMAN_tempKHP_ROWS
#undef __KALMAN_tempKHP_COLS

#undef __KALMAN_tempPHt_ROWS
#undef __KALMAN_tempPHt_COLS

#undef __KALMAN_tempHP_ROWS
#undef __KALMAN_tempHP_COLS

#undef __KALMAN_maux_ROWS
#undef __KALMAN_maux_COLS
#undef __USE_BUFFER_AUX

#undef __KALMAN_tempKHP_size
#undef __KALMAN_BUFFER_tempKHP

#undef __KALMAN_BUFFER_tempHP
#undef __KALMAN_tempHP_size

#undef __KALMAN_BUFFER_tempPHt
#undef __KALMAN_tempPHt_size

#undef __KALMAN_BUFFER_Sinv
#undef __KALMAN_Sinv_size

#undef __KALMAN_BUFFER_maux
#undef __KALMAN_maux_size
