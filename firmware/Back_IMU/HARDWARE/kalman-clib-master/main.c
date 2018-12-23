#include "matrix_unittests.h"
#include "kalman_example_gravity.h"

/**
* \brief Main entry point
*/
void main()
{
    matrix_unittests();
 
    kalman_gravity_demo();
    kalman_gravity_demo_lambda();
}
