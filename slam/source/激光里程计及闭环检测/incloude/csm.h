//
// Created by ycai on 2020/11/4.
//

#ifndef CSM_CSM_H
#define CSM_CSM_H

#include <math.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>

#include "laser_data.h"
#include "egsl.h"

#include <Eigen/Dense>

class CSM
{
public:
    CSM();
    ~CSM();

    void sm_icp(struct sm_params* params, struct sm_result* res, double* num_corres_ratio);
    /** Fills the fields: *up_bigger, *up_smaller, *down_bigger, *down_smaller.*/
    void ld_create_jump_tables(LDP);
    int icp_loop(struct sm_params*params, const double*q0, double*x_new,
                 double*total_error, int*valid, int*iterations, double* num_corres_ratio);
    void find_correspondences_tricks(struct sm_params*params);

    void kill_outliers_double(struct sm_params*params);

    void kill_outliers_trim(struct sm_params*params,  double*total_error);

    int compute_next_estimate(struct sm_params*params,
                                   const double x_old[3], double x_new[3]);

    int gpc_solve(int K, const struct gpc_corr*c,
                       const double*x0, const double *cov_x0,
                       double *x_out);


    double gpc_total_error(const struct gpc_corr*co, int n, const double*x);

    double gpc_error(const struct gpc_corr*co, const double*x);

    int termination_criterion(struct sm_params*params, const double*delta);

    void compute_covariance_exact(
            LDP laser_ref, LDP laser_sens, const gsl_vector*x,
            egsl::val *cov0_x, egsl::val *dx_dy1, egsl::val *dx_dy2);

    egsl::val compute_C_k(egsl::val p_j1, egsl::val p_j2);

    egsl::val dC_drho(egsl::val p1, egsl::val p2);

    egsl::val ld_fisher0(LDP ld);//Censi's solution

    egsl::val ld_fisher(LDP ld);//University of California at Berkeley's solution

    //static inline智能在类的头文件中实现
    INLINE double local_distance_squared_d(const double* a, const double* b)
    {
        //distance_counter++;
        double x = a[0] - b[0];
        double y = a[1] - b[1];
        return x*x + y*y;
    }

    /** This is very close (but *less* than) to sin(x), for
    x in (0, PI/2). It's a 5degree taylor expansion. */
    INLINE double mysin(double x) {
        const double a = -1.0/6.0;
        const double b = +1.0/120.0;
        double x2 = x*x;
        return x * (.99 + x2 * ( a + b * x2));
    }

    


private:
    egsl* egsl_;
    
};


#endif //CSM_CSM_H
