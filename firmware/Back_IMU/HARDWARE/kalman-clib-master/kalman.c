#include <stdint.h>
#include <assert.h>

#include "cholesky.h"

#define EXTERN_INLINE_KALMAN INLINE
#include "kalman.h"

/*!
* \brief Initializes the Kalman Filter
* \param[in] kf The Kalman Filter structure to initialize
* \param[in] num_states The number of state variables
* \param[in] num_inputs The number of input variables
* \param[in] A The state transition matrix ({\ref num_states} x {\ref num_states})
* \param[in] x The state vector ({\ref num_states} x \c 1)
* \param[in] B The input transition matrix ({\ref num_states} x {\ref num_inputs})
* \param[in] u The input vector ({\ref num_inputs} x \c 1)
* \param[in] P The state covariance matrix ({\ref num_states} x {\ref num_states})
* \param[in] Q The input covariance matrix ({\ref num_inputs} x {\ref num_inputs})
* \param[in] aux The auxiliary buffer (length {\ref num_states} or {\ref num_inputs}, whichever is greater)
* \param[in] predictedX The temporary vector for predicted X ({\ref num_states} x \c 1)
* \param[in] temp_P The temporary matrix for P calculation ({\ref num_states} x {\ref num_states})
* \param[in] temp_BQ The temporary matrix for BQ calculation ({\ref num_states} x {\ref num_inputs})
*/
void kalman_filter_initialize(kalman_t *kf, uint_fast8_t num_states, uint_fast8_t num_inputs, matrix_data_t *A, matrix_data_t *x,
    matrix_data_t *B, matrix_data_t *u, matrix_data_t *P, matrix_data_t *Q,
    matrix_data_t *aux, matrix_data_t *predictedX, matrix_data_t *temp_P, matrix_data_t *temp_BQ)
{
    matrix_init(&kf->A, num_states, num_states, A);
    matrix_init(&kf->P, num_states, num_states, P);
    matrix_init(&kf->x, num_states, 1, x);

    matrix_init(&kf->B, num_states, num_inputs, B);
    matrix_init(&kf->Q, num_inputs, num_inputs, Q);
    matrix_init(&kf->u, num_inputs, 1, u);

    // set auxiliary vector
    kf->temporary.aux = aux;

    // set predicted x vector
    matrix_init(&kf->temporary.predicted_x, num_states, 1, predictedX);

    // set temporary P matrix
    matrix_init(&kf->temporary.P, num_states, num_states, temp_P);

    // set temporary BQ matrix
    matrix_init(&kf->temporary.BQ, num_states, num_inputs, temp_BQ);
}


/*!
* \brief Sets the measurement vector
* \param[in] kfm The Kalman Filter measurement structure to initialize
* \param[in] num_states The number of states
* \param[in] num_measurements The number of measurements
* \param[in] H The measurement transformation matrix ({\ref num_measurements} x {\ref num_states})
* \param[in] z The measurement vector ({\ref num_measurements} x \c 1)
* \param[in] R The process noise / measurement uncertainty ({\ref num_measurements} x {\ref num_measurements})
* \param[in] y The innovation ({\ref num_measurements} x \c 1)
* \param[in] S The residual covariance ({\ref num_measurements} x {\ref num_measurements})
* \param[in] K The Kalman gain ({\ref num_states} x {\ref num_measurements})
* \param[in] aux The auxiliary buffer (length {\ref num_states} or {\ref num_measurements}, whichever is greater)
* \param[in] S_inv The temporary matrix for the inverted residual covariance  ({\ref num_measurements} x {\ref num_measurements})
* \param[in] temp_HP The temporary matrix for HxP ({\ref num_measurements} x {\ref num_states})
* \param[in] temp_PHt The temporary matrix for PxH' ({\ref num_states} x {\ref num_measurements})
* \param[in] temp_KHP The temporary matrix for KxHxP ({\ref num_states} x {\ref num_states})
*/
void kalman_measurement_initialize(kalman_measurement_t *kfm, uint_fast8_t num_states, uint_fast8_t num_measurements, matrix_data_t *H, matrix_data_t *z, matrix_data_t *R,
    matrix_data_t *y, matrix_data_t *S, matrix_data_t *K,
    matrix_data_t *aux, matrix_data_t *S_inv, matrix_data_t *temp_HP, matrix_data_t *temp_PHt, matrix_data_t *temp_KHP)
{
    matrix_init(&kfm->H, num_measurements, num_states, H);
    matrix_init(&kfm->R, num_measurements, num_measurements, R);
    matrix_init(&kfm->z, num_measurements, 1, z);

    matrix_init(&kfm->K, num_states, num_measurements, K);
    matrix_init(&kfm->S, num_measurements, num_measurements, S);
    matrix_init(&kfm->y, num_measurements, 1, y);

    // set auxiliary vector
    kfm->temporary.aux = aux;

    // set inverted S matrix
    matrix_init(&kfm->temporary.S_inv, num_measurements, num_measurements, S_inv);

    // set temporary HxP matrix
    matrix_init(&kfm->temporary.HP, num_measurements, num_states, temp_HP);

    // set temporary PxH' matrix
    matrix_init(&kfm->temporary.PHt, num_states, num_measurements, temp_PHt);

    // set temporary KxHxP matrix
    matrix_init(&kfm->temporary.KHP, num_states, num_states, temp_KHP);
}

/*!
* \brief Performs the time update / prediction step of only the state vector
* \param[in] kf The Kalman Filter structure to predict with.
*/
void kalman_predict_x(register kalman_t *const kf)
{
    // matrices and vectors
    const matrix_t *RESTRICT const A = &kf->A;
    matrix_t *RESTRICT const x = &kf->x;

    // temporaries
    matrix_t *RESTRICT const xpredicted = &kf->temporary.predicted_x;

    /************************************************************************/
    /* Predict next state using system dynamics                             */
    /* x = A*x                                                              */
    /************************************************************************/

    // x = A*x
    matrix_mult_rowvector(A, x, xpredicted);
    matrix_copy(xpredicted, x);
}

/*!
* \brief Performs the time update / prediction step of only the state covariance matrix
* \param[in] kf The Kalman Filter structure to predict with.
*/
void kalman_predict_Q(register kalman_t *const kf)
{
    // matrices and vectors
    const matrix_t *RESTRICT const A = &kf->A;
    const matrix_t *RESTRICT const B = &kf->B;
    matrix_t *RESTRICT const P = &kf->P;

    // temporaries
    matrix_data_t *RESTRICT const aux = kf->temporary.aux;
    matrix_t *RESTRICT const P_temp = &kf->temporary.P;
    matrix_t *RESTRICT const BQ_temp = &kf->temporary.BQ;

    /************************************************************************/
    /* Predict next covariance using system dynamics and input              */
    /* P = A*P*A' + B*Q*B'                                                  */
    /************************************************************************/

    // P = A*P*A'
    matrix_mult(A, P, P_temp, aux);                 // temp = A*P
    matrix_mult_transb(P_temp, A, P);               // P = temp*A'

    // P = P + B*Q*B'
    if (kf->B.rows > 0)
    {
        matrix_mult(B, &kf->Q, BQ_temp, aux);       // temp = B*Q
        matrix_multadd_transb(BQ_temp, B, P);       // P += temp*B'
    }
}

/*!
* \brief Performs the time update / prediction step of only the state covariance matrix
* \param[in] kf The Kalman Filter structure to predict with.
*/
void kalman_predict_Q_tuned(register kalman_t *const kf, matrix_data_t lambda)
{
    // matrices and vectors
    const matrix_t *RESTRICT const A = &kf->A;
    const matrix_t *RESTRICT const B = &kf->B;
    matrix_t *RESTRICT const P = &kf->P;

    // temporaries
    matrix_data_t *RESTRICT const aux = kf->temporary.aux;
    matrix_t *RESTRICT const P_temp = &kf->temporary.P;
    matrix_t *RESTRICT const BQ_temp = &kf->temporary.BQ;

    /************************************************************************/
    /* Predict next covariance using system dynamics and input              */
    /* P = A*P*A' * 1/lambda^2 + B*Q*B'                                     */
    /************************************************************************/

    // lambda = 1/lambda^2
    lambda = (matrix_data_t)1.0 / (lambda * lambda); // TODO: This should be precalculated, e.g. using kalman_set_lambda(...);

    // P = A*P*A'
    matrix_mult(A, P, P_temp, aux);                 // temp = A*P
    matrix_multscale_transb(P_temp, A, lambda, P);   // P = temp*A' * 1/(lambda^2)

    // P = P + B*Q*B'
    if (kf->B.rows > 0)
    {
        matrix_mult(B, &kf->Q, BQ_temp, aux);       // temp = B*Q
        matrix_multadd_transb(BQ_temp, B, P);        // P += temp*B'
    }
}

/*!
* \brief Performs the measurement update step.
* \param[in] kf The Kalman Filter structure to correct.
*/
void kalman_correct(kalman_t *kf, kalman_measurement_t *kfm)
{
    matrix_t *RESTRICT const P = &kf->P;
    const matrix_t *RESTRICT const H = &kfm->H;
    matrix_t *RESTRICT const K = &kfm->K;
    matrix_t *RESTRICT const S = &kfm->S;
    matrix_t *RESTRICT const y = &kfm->y;
    matrix_t *RESTRICT const x = &kf->x;

    // temporaries
    matrix_data_t *RESTRICT const aux = kfm->temporary.aux;
    matrix_t *RESTRICT const Sinv = &kfm->temporary.S_inv;
    matrix_t *RESTRICT const temp_HP = &kfm->temporary.HP;
    matrix_t *RESTRICT const temp_KHP = &kfm->temporary.KHP;
    matrix_t *RESTRICT const temp_PHt = &kfm->temporary.PHt;

    /************************************************************************/
    /* Calculate innovation and residual covariance                         */
    /* y = z - H*x                                                          */
    /* S = H*P*H' + R                                                       */
    /************************************************************************/

    // y = z - H*x
    matrix_mult_rowvector(H, x, y);
    matrix_sub_inplace_b(&kfm->z, y);

    // S = H*P*H' + R
    matrix_mult(H, P, temp_HP, aux);            // temp = H*P
    matrix_mult_transb(temp_HP, H, S);          // S = temp*H'
    matrix_add_inplace(S, &kfm->R);             // S += R 

    /************************************************************************/
    /* Calculate Kalman gain                                                */
    /* K = P*H' * S^-1                                                      */
    /************************************************************************/

    // K = P*H' * S^-1
    cholesky_decompose_lower(S);
    matrix_invert_lower(S, Sinv);               // Sinv = S^-1
    // NOTE that to allow aliasing of Sinv and temp_PHt, a copy must be performed here
    matrix_mult_transb(P, H, temp_PHt);         // temp = P*H'
    matrix_mult(temp_PHt, Sinv, K, aux);        // K = temp*Sinv

    /************************************************************************/
    /* Correct state prediction                                             */
    /* x = x + K*y                                                          */
    /************************************************************************/

    // x = x + K*y 
    matrix_multadd_rowvector(K, y, x);

    /************************************************************************/
    /* Correct state covariances                                            */
    /* P = (I-K*H) * P                                                      */
    /*   = P - K*(H*P)                                                      */
    /************************************************************************/

    // P = P - K*(H*P)
    matrix_mult(H, P, temp_HP, aux);            // temp_HP = H*P
    matrix_mult(K, temp_HP, temp_KHP, aux);     // temp_KHP = K*temp_HP
    matrix_sub(P, temp_KHP, P);                 // P -= temp_KHP 
}