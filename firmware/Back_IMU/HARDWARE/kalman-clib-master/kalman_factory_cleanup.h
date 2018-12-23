/************************************************************************/
/* Clean up                                                             */
/************************************************************************/

// remove Kalman filter defines
#undef KALMAN_NAME
#undef KALMAN_NUM_STATES
#undef KALMAN_NUM_INPUTS

// remove x macros
#undef __KALMAN_x_ROWS
#undef __KALMAN_x_COLS
#undef __KALMAN_BUFFER_x

// remove u macros
#undef __KALMAN_u_ROWS
#undef __KALMAN_u_COLS
#undef __KALMAN_BUFFER_u

// remove A macros
#undef __KALMAN_A_ROWS
#undef __KALMAN_A_COLS
#undef __KALMAN_BUFFER_A

// remove P macros
#undef __KALMAN_P_ROWS
#undef __KALMAN_P_COLS
#undef __KALMAN_BUFFER_P

// remove B macros
#undef __KALMAN_B_ROWS
#undef __KALMAN_B_COLS
#undef __KALMAN_BUFFER_B

// remove Q macros
#undef __KALMAN_Q_ROWS
#undef __KALMAN_Q_COLS
#undef __KALMAN_BUFFER_Q

// remove name helper macros
#undef KALMAN_BASENAME_HELPER
#undef KALMAN_FILTER_BASENAME
#undef KALMAN_FILTER_BASENAME_HELPER
#undef __CONCAT
#undef KALMAN_BUFFER_HELPER
#undef KALMAN_BUFFER_HELPER2
#undef KALMAN_FUNCTION_HELPER
#undef KALMAN_FUNCTION_HELPER2

// remove name macros
#undef KALMAN_STRUCT_NAME
#undef KALMAN_FUNCTION_NAME
#undef KALMAN_BUFFER_NAME

// remove auxiliaries
#undef __KALMAN_BUFFER_aux
#undef __KALMAN_BUFFER_tempPBQ
#undef __KALMAN_tempP_size
#undef __KALMAN_tempBQ_size
#undef __KALMAN_tempPBQ_size

// remove measurement defines just because we can
#undef KALMAN_MEASUREMENT_NAME
#undef KALMAN_NUM_MEASUREMENTS