//
// Created by ycai on 2020/11/23.
//

#ifndef CSM_EGSL_HPP
#define CSM_EGSL_HPP

#define MAX_VALS 1024
#define MAX_CONTEXTS 1024

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_poly.h>

#include <math.h>
#include <assert.h>
#include <string.h>
#include "math_utils.h"

class egsl
{
public:
    struct egsl_val
    {
        gsl_matrix * gslm;
        int cid : 16;
        int index : 16;
    };
    typedef struct egsl_val val;

    struct egsl_variable
    {
        gsl_matrix* gsl_m;
    };

    struct egsl_context
    {
        char name[256];
        int nallocated;
        int nvars;
        struct egsl_variable vars[MAX_VALS];
    };

    egsl();
    ~egsl() {};

    /*Core functions*/
    /* Push a new context. */
    void egsl_push();
    void egsl_push_named(const char* name);
    /* Pops a context */
    void egsl_pop();
    void egsl_pop_named(const char* name);
    void egsl_free(void);

    val assemble_val(int cid,int index,gsl_matrix* m);
    int its_context(val v);
    int its_var_index(val v);
    void check_valid_val(val v);

    double* egsl_atmp(val v,size_t i,size_t j);
    val egsl_alloc(size_t rows,size_t columns);
    val egsl_alloc_in_context(int cid,size_t rows,size_t cols);
    gsl_matrix * egsl_gslm(val v);
    /** Creates a copy of v in the previous context.*/
    val egsl_promote(val v);

    /** Operations among values */
    val egsl_scale(double, val);
    val egsl_sum3(val, val, val);
    val egsl_mult(val, val);
    val egsl_transpose(val);
    val egsl_inverse(val);
    val egsl_sub(val,val);
    val egsl_sum(val v1,val v2);
    val egsl_compose_col(val v1, val v2);
    val egsl_compose_row(val v1, val v2);
    void egsl_add_to(val v1, val v2);
    void egsl_add_to_col(val v1, size_t j, val v2);
    val egsl_copy_val(val v1);

    double egsl_norm(val);

    void egsl_symm_eig(val v, double* eigenvalues, val* eigenvectors);

    double egsl_atv(val, size_t i);
    double egsl_atm(val, size_t i, size_t j);

    /* File: egsl_conversions.c
     Conversions */

    val egsl_vFa(size_t rows, const double*);
    val egsl_vFda(size_t rows, size_t columns, const double*);

    /** Copies a VECTOR value into array */
    void egsl_v2a(val, double*);
    /** Copies a MATRIX value into array (row1 .. rown) */
    void egsl_v2da(val, double*);
    /** Copies a vector value into a gsl_vector */
    void egsl_v2vec(val, gsl_vector*);

    val egsl_vFgslv(const gsl_vector*);
    val egsl_vFgslm(const gsl_matrix*);

    gsl_matrix* egsl_v2gslm(val);

    /*/ File: egsl_misc.c
    Miscellaneous useful matrixes. */
    val egsl_zeros(size_t rows, size_t columns);
    val egsl_ones(size_t rows, size_t columns);
    val egsl_vers(double theta);
    val egsl_rot(double theta);

    /* Misc */
    void egsl_print(const char*str, val);
    /** Prints eigenvalues and eigenvectors of a symmetric matrix */
    void egsl_print_spectrum(const char*s, val v);
    void egsl_print_stats(void);

    /** Private implementations things */
    void egsl_expect_size(val v, size_t rows, size_t cols);
    void egsl_error(void);

    /* GSL stuff */
    const char* gsl_friendly_pose(gsl_vector*v);
    gsl_vector * vector_from_array(unsigned int n, double *x);
    void vector_to_array(const gsl_vector*v, double*);
    void copy_from_array(gsl_vector*v, double*);

    void oplus(const gsl_vector*x1,const gsl_vector*x2, gsl_vector*res);
    void ominus(const gsl_vector*x, gsl_vector*res);
    void pose_diff(const gsl_vector*pose2,const gsl_vector*pose1,gsl_vector*res);

    void transform(const gsl_vector* point2d, const gsl_vector* pose, gsl_vector*result2d);
    void gsl_vector_set_nan(gsl_vector*v);

    double distance(const gsl_vector* a,const gsl_vector* b);
    double distance_squared(const gsl_vector* a,const gsl_vector* b);

    /** Returns norm of 2D point p */
    double norm(const gsl_vector*p);
    const char* egsl_friendly_pose(val pose);
    const char* egsl_friendly_cov(val cov);

    /*gpc solve gsl*/
    void m_trans(const gsl_matrix*A, gsl_matrix*A_t);
    void m_mult(const gsl_matrix*A, const gsl_matrix*B, gsl_matrix*AB);
    void m_add(const gsl_matrix*A, const gsl_matrix*B, gsl_matrix*ApB);
    void m_add_to(const gsl_matrix*A, gsl_matrix*B);
    void m_scale(double m, gsl_matrix*A);
    double m_dot(const gsl_matrix*A,const gsl_matrix*B);
    void m_inv(const gsl_matrix*A, gsl_matrix*invA);
    double m_det(const gsl_matrix*A);

    /* Returns the real part of the roots in roots */
    int poly_real_roots(unsigned int n, const double*a, double *roots);

    int poly_greatest_real_root(unsigned int n, const double*a, double *root);

    void m_display(const char*str, gsl_matrix*m);



private:
    int cid;
    int max_cid;
    //static struct egsl_context egsl_contexts[MAX_CONTEXTS];
    int egsl_first_time;
    int egsl_total_allocations;
    int egsl_cache_hits;

};

#endif //CSM_EGSL_HPP
