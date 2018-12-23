/*!
* \brief Initializes a named Kalman filter structure.
*
* This include requires the three defines {\ref KALMAN_NAME}, {\ref KALMAN_NUM_STATES} and
* {\ref KALMAN_NUM_INPUTS} to be set to the base name of the Kalman Filter and to the number 
* of states and inputs respectively.
*
* It then will instantiate the buffers required for A, P, x as well as B, Q and u if the number
* of inputs is greater than zero, as well as the structure for the Kalman filter and the
* initialization method.
*
* Suppose the Kalman filter shall be named "acceleration", has three states and zero inputs.
* You would start by defining the required macros
*
* \code{.c}
* #define KALMAN_NAME acceleration
* #define KALMAN_NUM_STATES 3
* #define KALMAN_NUM_INPUTS 0
* \endcode
*
* After that, this file must be included
*
* \code{.c}
* #include "kalman_factory_filter_init.h"
* \endcode
*
* At this point, the structure \c kalman_filter_acceleration will be created (statically) along with
* all the required buffers (i.e. \c kalman_filter_acceleration_A_buffer, etc.) and the matrices
* will be initialized and set with the correct dimensions.
*
* In addition, a parameterless static initialization function \c {kalman_filter_acceleration_init()} will
* be created which you will need to call manually in order to set up the filter.
*
* To clean up the defined macros (e.g. in order to be able to create another named Kalman filter),
* you will have to include kalman_factory_cleanup.h:
*
* \code{.c}
* #include "kalman_factory_cleanup.h"
* \endcode
*
* A full example would be

* \code{.c}
* #define KALMAN_NAME example
* #define KALMAN_NUM_STATES 4
* #define KALMAN_NUM_INPUTS 0

* #include "kalman_factory_filter.h"
* // NOTE that this is the point to create measurement buffers
* #include "kalman_factory_cleanup.h"

* void test_kalman()
* {
*   kalman_filter_example_init();
*   kalman_filter_example.x.data[0] = 1;
* }
* \endcode
*/

/************************************************************************/
/* Check for inputs                                                     */
/************************************************************************/

#ifndef KALMAN_NAME
#error KALMAN_NAME needs to be defined prior to inclusion of this file.
#endif

#ifndef KALMAN_NUM_STATES
#error KALMAN_NUM_STATES needs to be defined prior to inclusion of this file.
#elif KALMAN_NUM_STATES <= 0
#error KALMAN_NUM_STATES must be a positive integer
#endif

#ifndef KALMAN_NUM_INPUTS
#error KALMAN_NUM_INPUTS needs to be defined prior to inclusion of this file.
#elif KALMAN_NUM_INPUTS < 0
#error KALMAN_NUM_INPUTS must be a positive integer or zero if no inputs are used
#endif

/************************************************************************/
/* Prepare dimensions                                                   */
/************************************************************************/

#define __KALMAN_A_ROWS     KALMAN_NUM_STATES
#define __KALMAN_A_COLS     KALMAN_NUM_STATES

#define __KALMAN_P_ROWS     KALMAN_NUM_STATES
#define __KALMAN_P_COLS     KALMAN_NUM_STATES

#define __KALMAN_x_ROWS     KALMAN_NUM_STATES
#define __KALMAN_x_COLS     1

#define __KALMAN_B_ROWS     KALMAN_NUM_STATES
#define __KALMAN_B_COLS     KALMAN_NUM_INPUTS

#define __KALMAN_u_ROWS     KALMAN_NUM_INPUTS
#define __KALMAN_u_COLS     1

#define __KALMAN_Q_ROWS     KALMAN_NUM_INPUTS
#define __KALMAN_Q_COLS     KALMAN_NUM_INPUTS

#define __KALMAN_aux_ROWS     ((KALMAN_NUM_STATES > KALMAN_NUM_INPUTS) ? KALMAN_NUM_STATES : KALMAN_NUM_INPUTS)
#define __KALMAN_aux_COLS     1

#define __KALMAN_tempP_ROWS  __KALMAN_P_ROWS
#define __KALMAN_tempP_COLS  __KALMAN_P_COLS

#define __KALMAN_tempBQ_ROWS __KALMAN_B_ROWS
#define __KALMAN_tempBQ_COLS __KALMAN_B_COLS

/************************************************************************/
/* Name helper macro                                                    */
/************************************************************************/

#ifndef STRINGIFY
#define __STRING2(x) #x
#define STRINGIFY(x) __STRING2(x)
#endif

#pragma message("** Instantiating Kalman filter \"" STRINGIFY(KALMAN_NAME) "\" with " STRINGIFY(KALMAN_NUM_STATES) " states and " STRINGIFY(KALMAN_NUM_INPUTS) " inputs")

#define __CONCAT(x, y)                                  x ## y

#define KALMAN_FILTER_BASENAME_HELPER(name)             __CONCAT(kalman_filter_, name)
#define KALMAN_FILTER_BASENAME                          KALMAN_FILTER_BASENAME_HELPER(KALMAN_NAME)
#define KALMAN_BASENAME_HELPER(basename)                __CONCAT(basename, _)

/************************************************************************/
/* Name macro                                                           */
/************************************************************************/

#define KALMAN_BUFFER_HELPER2(basename, element)         basename ## element ## _buffer
#define KALMAN_BUFFER_HELPER(basename, element)         KALMAN_BUFFER_HELPER2(basename, element)
#define KALMAN_BUFFER_NAME(element)                     KALMAN_BUFFER_HELPER(KALMAN_BASENAME_HELPER(KALMAN_FILTER_BASENAME), element)

#define KALMAN_FUNCTION_HELPER2(basename, name)         basename ## name
#define KALMAN_FUNCTION_HELPER(basename, name)          KALMAN_FUNCTION_HELPER2(basename, name)
#define KALMAN_FUNCTION_NAME(name)                      KALMAN_FUNCTION_HELPER(KALMAN_BASENAME_HELPER(KALMAN_FILTER_BASENAME), name)

#define KALMAN_STRUCT_NAME                              KALMAN_FILTER_BASENAME

/************************************************************************/
/* Construct Kalman filter buffers: State                               */
/************************************************************************/

#include "compiler.h"
#include "matrix.h"
#include "kalman.h"

#define __KALMAN_BUFFER_A   KALMAN_BUFFER_NAME(A)
#define __KALMAN_BUFFER_P   KALMAN_BUFFER_NAME(P)
#define __KALMAN_BUFFER_x   KALMAN_BUFFER_NAME(x)

#pragma message("Creating Kalman filter A buffer: " STRINGIFY(__KALMAN_BUFFER_A))
static matrix_data_t __KALMAN_BUFFER_A[__KALMAN_A_ROWS * __KALMAN_A_COLS];

#pragma message("Creating Kalman filter P buffer: " STRINGIFY(__KALMAN_BUFFER_P))
static matrix_data_t __KALMAN_BUFFER_P[__KALMAN_P_ROWS * __KALMAN_P_COLS];

#pragma message("Creating Kalman filter x buffer: " STRINGIFY(__KALMAN_BUFFER_x))
static matrix_data_t __KALMAN_BUFFER_x[__KALMAN_x_ROWS * __KALMAN_x_COLS];

/************************************************************************/
/* Construct Kalman filter buffers: Inputs                              */
/************************************************************************/

#if KALMAN_NUM_INPUTS > 0

#define __KALMAN_BUFFER_B   KALMAN_BUFFER_NAME(B)
#define __KALMAN_BUFFER_Q   KALMAN_BUFFER_NAME(Q)
#define __KALMAN_BUFFER_u   KALMAN_BUFFER_NAME(u)

#pragma message("Creating Kalman filter B buffer: " STRINGIFY(__KALMAN_BUFFER_B))
static matrix_data_t __KALMAN_BUFFER_B[__KALMAN_B_ROWS * __KALMAN_B_COLS];

#pragma message("Creating Kalman filter Q buffer: " STRINGIFY(__KALMAN_BUFFER_Q))
static matrix_data_t __KALMAN_BUFFER_Q[__KALMAN_Q_ROWS * __KALMAN_Q_COLS];

#pragma message("Creating Kalman filter u buffer: " STRINGIFY(__KALMAN_BUFFER_u))
static matrix_data_t __KALMAN_BUFFER_u[__KALMAN_x_ROWS * __KALMAN_u_COLS];

#else

#pragma message("Skipping Kalman filter B buffer: (zero inputs)")
#define __KALMAN_BUFFER_B ((matrix_data_t*)0)

#pragma message("Skipping Kalman filter Q buffer: (zero inputs)")
#define __KALMAN_BUFFER_Q ((matrix_data_t*)0)

#pragma message("Skipping Kalman filter u buffer: (zero inputs)")
#define __KALMAN_BUFFER_u ((matrix_data_t*)0)

#endif

/************************************************************************/
/* Construct Kalman filter buffers: Temporaries                         */
/************************************************************************/

#define __KALMAN_BUFFER_aux     KALMAN_BUFFER_NAME(aux)
#define __KALMAN_BUFFER_tempPBQ KALMAN_BUFFER_NAME(tempPBQ)

#define __KALMAN_aux_size       (__KALMAN_aux_ROWS * __KALMAN_aux_COLS)
#define __KALMAN_tempP_size     (__KALMAN_tempP_ROWS * __KALMAN_tempP_COLS)
#define __KALMAN_tempBQ_size    (__KALMAN_tempBQ_ROWS * __KALMAN_tempBQ_COLS)

#define __KALMAN_tempPBQ_size   ((__KALMAN_tempP_size > __KALMAN_tempBQ_size) ? __KALMAN_tempP_size : __KALMAN_tempBQ_size)

#pragma message("Creating Kalman filter aux buffer: " STRINGIFY(__KALMAN_BUFFER_aux))
static matrix_data_t __KALMAN_BUFFER_aux[__KALMAN_aux_size];

#pragma message("Creating Kalman filter temporary P/BQ buffer: " STRINGIFY(__KALMAN_BUFFER_tempPBQ))
static matrix_data_t __KALMAN_BUFFER_tempPBQ[__KALMAN_tempPBQ_size];

/************************************************************************/
/* Construct Kalman filter                                              */
/************************************************************************/

#pragma message("Creating Kalman filter structure: " STRINGIFY(KALMAN_STRUCT_NAME))

/*!
* \brief The Kalman filter structure
*/
static kalman_t KALMAN_STRUCT_NAME;

#pragma message ("Creating Kalman filter initialization function: " STRINGIFY(KALMAN_FUNCTION_NAME(init()) ))

/*!
* \brief Initializes the Kalman Filter
* \return Pointer to the filter.
*/
static kalman_t* KALMAN_FUNCTION_NAME(init)()
{
    int i;
    for (i = 0; i < __KALMAN_x_ROWS * __KALMAN_x_COLS; ++i) { __KALMAN_BUFFER_x[i] = 0; }
    for (i = 0; i < __KALMAN_A_ROWS * __KALMAN_A_COLS; ++i) { __KALMAN_BUFFER_A[i] = 0; }
    for (i = 0; i < __KALMAN_P_ROWS * __KALMAN_P_COLS; ++i) { __KALMAN_BUFFER_P[i] = 0; }
    for (i = 0; i < __KALMAN_u_ROWS * __KALMAN_x_COLS; ++i) { __KALMAN_BUFFER_x[i] = 0; }

#if KALMAN_NUM_INPUTS > 0
    for (i = 0; i < __KALMAN_B_ROWS * __KALMAN_B_COLS; ++i) { __KALMAN_BUFFER_B[i] = 0; }
    for (i = 0; i < __KALMAN_Q_ROWS * __KALMAN_Q_COLS; ++i) { __KALMAN_BUFFER_Q[i] = 0; }
#endif

    kalman_filter_initialize(&KALMAN_STRUCT_NAME, KALMAN_NUM_STATES, KALMAN_NUM_INPUTS, __KALMAN_BUFFER_A, __KALMAN_BUFFER_x, 
                            __KALMAN_BUFFER_B, __KALMAN_BUFFER_u, __KALMAN_BUFFER_P, __KALMAN_BUFFER_Q,
                            __KALMAN_BUFFER_aux, __KALMAN_BUFFER_aux, __KALMAN_BUFFER_tempPBQ, __KALMAN_BUFFER_tempPBQ);
    return &KALMAN_STRUCT_NAME;
}

