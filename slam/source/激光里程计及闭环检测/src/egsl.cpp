//
// Created by ycai on 2020/11/23.
#include "egsl.h"

//

static struct egsl::egsl_context egsl_contexts[MAX_CONTEXTS];

egsl::egsl()
{
    cid = 0;
    max_cid = 0;
    egsl_first_time = 1;
    egsl_total_allocations = 0;
    egsl_cache_hits = 0;

}

void egsl::egsl_error(void)
{
    egsl_print_stats();
    assert(0);
}

egsl::val egsl::assemble_val(int cid, int index, gsl_matrix*m) {
    val v;
    v.cid=cid;
    v.index=index;
    v.gslm = m;
    return v;
}

int egsl::its_context(val v)
{
    return v.cid;
}

int egsl::its_var_index(val v)
{
    return v.index;
}

void egsl::check_valid_val(val v)
{
    int context = its_context(v);
    if(context>cid) {
        fprintf(stderr, "Val is from invalid context (%d>%d)\n",context,cid);
        egsl_error();
    }
    int var_index = its_var_index(v);
    if(var_index >= egsl_contexts[context].nvars) {
        fprintf(stderr, "Val is invalid  (%d>%d)\n",var_index,
                egsl_contexts[context].nvars);
        egsl_error();
    }
}

gsl_matrix* egsl::egsl_gslm(val v)
{
    check_valid_val(v);
    return v.gslm;
}

void egsl::egsl_push() { egsl_push_named("unnamed context"); }
void egsl::egsl_pop() { egsl_pop_named("unnamed context"); }

void egsl::egsl_push_named(const char*name)
{
    if(egsl_first_time)
    {
        int c;
        for(c=0;c<MAX_CONTEXTS;c++) {
            egsl_contexts[c].nallocated = 0;
            egsl_contexts[c].nvars = 0;
            sprintf(egsl_contexts[c].name, "not yet used");
        }
        egsl_first_time  = 0;
    }
    cid++;
    if(cid >= MAX_CONTEXTS)
    {
        fprintf(stderr, "egsl: maximum number of contexts reached \n");
        egsl_print_stats();
        assert(0);
    }

    if(max_cid < cid) max_cid = cid;

    if(name != 0)
        sprintf(egsl_contexts[cid].name, "%s", name);
    else
        sprintf(egsl_contexts[cid].name, "Unnamed context");
}

void egsl::egsl_pop_named(const char*name)
{
    assert(cid>=0);/*, "No egsl_push before?"); */
    if(name != 0) {
        if(strcmp(name, egsl_contexts[cid].name)) {
            fprintf(stderr, "egsl: context mismatch. You want to pop '%s', you are still at '%s'\n",
                    name, egsl_contexts[cid].name);
            egsl_print_stats();
            assert(0);
        }
    }

    egsl_contexts[cid].nvars = 0;
    sprintf(egsl_contexts[cid].name, "Popped");
    cid--;
}

void egsl::egsl_print_stats()
{
    fprintf(stderr, "egsl: total allocations: %d   cache hits: %d\n",
            egsl_total_allocations, egsl_cache_hits);
/*	printf("egsl: sizeof(val) = %d\n",(int)sizeof(val)); */
    int c; for(c=0;c<=max_cid&&c<MAX_CONTEXTS;c++) {
        /*	printf("egsl: context #%d\n ",c); */
/*	 	if(0==egsl_contexts[c].nallocated) break; */
        fprintf(stderr, "egsl: context #%d allocations: %d active: %d name: '%s' \n",
                c,	egsl_contexts[c].nallocated, egsl_contexts[c].nvars, egsl_contexts[c].name);
    }
}

egsl::val egsl::egsl_alloc(size_t rows, size_t columns)
{
    struct egsl_context*c = egsl_contexts+cid;

    if(c->nvars>=MAX_VALS) {
        fprintf(stderr,"Limit reached, in context %d, nvars is %d\n",cid,c->nvars);
        egsl_error();
    }
    int index = c->nvars;
    if(index<c->nallocated) {
        gsl_matrix*m = c->vars[index].gsl_m;
        if(m->size1 == rows && m->size2 == columns) {
            egsl_cache_hits++;
            c->nvars++;
            return assemble_val(cid,index,c->vars[index].gsl_m);
        } else {
            gsl_matrix_free(m);
            egsl_total_allocations++;
            c->vars[index].gsl_m = gsl_matrix_alloc((size_t)rows,(size_t)columns);
            c->nvars++;
            return assemble_val(cid,index,c->vars[index].gsl_m);
        }
    } else {
        egsl_total_allocations++;
        c->vars[index].gsl_m = gsl_matrix_alloc((size_t)rows,(size_t)columns);
        c->nvars++;
        c->nallocated++;
        return assemble_val(cid,index,c->vars[index].gsl_m);
    }
}

egsl::val egsl::egsl_alloc_in_context(int context, size_t rows, size_t columns)
{
    struct egsl_context*c = egsl_contexts+context;

    if(c->nvars>=MAX_VALS) {
        fprintf(stderr,"Limit reached, in context %d, nvars is %d\n",context,c->nvars);
        egsl_error();
    }
    int index = c->nvars;
    if(index<c->nallocated) {
        gsl_matrix*m = c->vars[index].gsl_m;
        if(m->size1 == rows && m->size2 == columns) {
            egsl_cache_hits++;
            c->nvars++;
            return assemble_val(context,index,c->vars[index].gsl_m);
        } else {
            gsl_matrix_free(m);
            egsl_total_allocations++;
            c->vars[index].gsl_m = gsl_matrix_alloc((size_t)rows,(size_t)columns);
            c->nvars++;
            return assemble_val(context,index,c->vars[index].gsl_m);
        }
    } else {
        egsl_total_allocations++;
        c->vars[index].gsl_m = gsl_matrix_alloc((size_t)rows,(size_t)columns);
        c->nvars++;
        c->nallocated++;
        return assemble_val(context,index,c->vars[index].gsl_m);
    }
}

/** Creates a copy of v in the previous context. */
egsl::val egsl::egsl_promote(val v)
{
    if(cid==0) {
        egsl_error();
    }

    gsl_matrix * m = egsl_gslm(v);
    val v2 = egsl_alloc_in_context(cid-1, m->size1, m->size2);
    gsl_matrix * m2 = egsl_gslm(v2);
    gsl_matrix_memcpy(m2, m);
    return v2;
}

void egsl::egsl_expect_size(val v, size_t rows, size_t cols)
{
    gsl_matrix * m = egsl_gslm(v);

    int bad = (rows && (m->size1!=rows)) || (cols && (m->size2!=cols));
    if(bad) {
        fprintf(stderr, "Matrix size is %d,%d while I expect %d,%d",
                (int)m->size1,(int)m->size2,(int)rows,(int)cols);

        egsl_error();
    }
}

void egsl::egsl_print(const char*str, val v)
{
    gsl_matrix * m = egsl_gslm(v);
    size_t i,j;
    int context = its_context(v);
    int var_index = its_var_index(v);
    fprintf(stderr, "%s =  (%d x %d)  context=%d index=%d\n",
            str,(int)m->size1,(int)m->size2,  context, var_index);

    for(i=0;i<m->size1;i++) {
        if(i==0)
            fprintf(stderr, " [ ");
        else
            fprintf(stderr, "   ");

        for(j=0;j<m->size2;j++)
            fprintf(stderr, "%f ", gsl_matrix_get(m,i,j));


        if(i==m->size1-1)
            fprintf(stderr, "] \n");
        else
            fprintf(stderr, "; \n");
    }
}

double* egsl::egsl_atmp(val v, size_t i, size_t j)
{
    gsl_matrix * m = egsl_gslm(v);
    return gsl_matrix_ptr(m,(size_t)i,(size_t)j);
}

double egsl::egsl_norm(val v1)
{
    egsl_expect_size(v1, 0, 1);
    double n=0;
    size_t i;
    gsl_matrix * m = egsl_gslm(v1);
    for(i=0;i<m->size1;i++) {
        double v = gsl_matrix_get(m,i,0);
        n += v * v;
    }
    return sqrt(n);
}

double egsl::egsl_atv(val v1,  size_t i)
{
    return *egsl_atmp(v1, i, 0);
}

double egsl::egsl_atm(val v1, size_t i, size_t j)
{
    return *egsl_atmp(v1, i, j);
}

void egsl::egsl_free(void)
{
    int c;
    for(c=0;c<=max_cid;c++) {
        for(int i=egsl_contexts[c].nvars; i<egsl_contexts[c].nallocated; i++){
            gsl_matrix_free(egsl_contexts[c].vars[i].gsl_m);
        }
        egsl_contexts[c].nallocated = egsl_contexts[c].nvars;
    }
}

/*egsl_conversions.c*/
egsl::val egsl::egsl_vFda(size_t rows, size_t cols, const double *a)
{
    val v = egsl_alloc(rows, cols);

    size_t i; size_t j;
    for(i=0;i<rows;i++)
        for(j=0;j<cols;j++) {
            *egsl_atmp(v,i,j) = a[j+i*cols];
        }
    return v;
}

egsl::val egsl::egsl_vFa(size_t rows, const double*a)
{
    val v = egsl_alloc(rows,1);
    size_t i;
    for(i=0;i<rows;i++)
        *egsl_atmp(v,i,0) =  a[i];
    return v;
}

void egsl::egsl_v2a(val v, double*vec)
{
    gsl_matrix *m = egsl_gslm(v);
    size_t i;
    for(i=0; i < m->size1; i++)
        vec[i] = gsl_matrix_get(m,i,0);
}

void egsl::egsl_v2da(val v, double*a)
{
    gsl_matrix *m = egsl_gslm(v);
    size_t i,j;
    for(i=0;i<m->size1;i++)
        for(j=0;j<m->size2;j++)
            a[j*m->size1 +i] = gsl_matrix_get(m,i,j);
}

void egsl::egsl_v2vec(val v, gsl_vector*vec)
{
    size_t i;
    for(i=0;i<vec->size;i++)
        gsl_vector_set(vec,i, *egsl_atmp(v,i,0));
}


egsl::val egsl::egsl_vFgslv(const gsl_vector*vec)
{
    val v = egsl_alloc(vec->size,1);
    size_t i;
    for(i=0;i<vec->size;i++)
        *egsl_atmp(v,i,0) = gsl_vector_get(vec,i);
    return v;
}

egsl::val egsl::egsl_vFgslm(const gsl_matrix*m)
{
    val v = egsl_alloc(m->size1,m->size2);
    gsl_matrix * m2 = egsl_gslm(v);
    gsl_matrix_memcpy(m2,m);
    return v;
}

gsl_matrix* egsl::egsl_v2gslm(val v)
{
    gsl_matrix * m = egsl_gslm(v);
    //gsl_matrix * m2 = gsl_matrix_alloc(m->size1,m->size2);
    //gsl_matrix_memcpy(m2,m);
    //return m2;
    val v1 = egsl_alloc(m->size1,m->size2);
    gsl_matrix_memcpy(v1.gslm,m);
    return v1.gslm;

    
    
}

/*egsl_misc.c*/
egsl::val egsl::egsl_rot(double theta) {
    double R[2*2] = {
            cos(theta), -sin(theta),
            sin(theta),  cos(theta)
    };
    return egsl_vFda(2,2,R);
}

egsl::val egsl::egsl_zeros(size_t rows, size_t columns)
{
    val v = egsl_alloc(rows,columns);
    gsl_matrix * m = egsl_gslm(v);
    gsl_matrix_set_all(m,0.0);
    return v;
}

egsl::val egsl::egsl_ones(size_t rows, size_t columns)
{
    val v = egsl_alloc(rows,columns);
    gsl_matrix * m = egsl_gslm(v);
    gsl_matrix_set_all(m,1.0);
    return v;
}

egsl::val egsl::egsl_vers(double theta)
{
    double v[2] = { cos(theta), sin(theta)};
    return egsl_vFa(2,v);
}

void egsl::egsl_print_spectrum(const char*s, val v)
{
    gsl_matrix *m = egsl_gslm(v);
    /* expect same size */
    size_t n = m->size1;
    double eval[n]; val evec[n];
    egsl_symm_eig(v, eval, evec);
    size_t i,j;
    for(j=0;j<n;j++) {
        fprintf(stderr, "%s | eval[%d] = %+5.5f evec[%d]= ",
                s, (int)j, eval[j],(int)j);
        for(i=0;i<n;i++)
            fprintf(stderr, "%+4.4f ", egsl_atv(evec[j],i));
        fprintf(stderr, " sqrt(eval[%d])=%5.5f  \n", (int)j, sqrt(eval[j]));
    }
}

/*egsl_ops.c*/
egsl::val egsl::egsl_sub(val v1,val v2)
{
    return egsl_sum(v1, egsl_scale(-1.0,v2));
}

egsl::val egsl::egsl_compose_col(val v1, val v2)
{
    gsl_matrix *m1 = egsl_gslm(v1);
    gsl_matrix *m2 = egsl_gslm(v2);
    egsl_expect_size(v2, 0, m1->size2);
    val v3 = egsl_alloc(m1->size1+m2->size1,m1->size2);
    gsl_matrix *m3 = egsl_gslm(v3);
    size_t i,j;
    for(j=0;j<m1->size2;j++) {
        for(i=0;i<m1->size1;i++)
            gsl_matrix_set(m3, i, j, gsl_matrix_get(m1,i,j));

        for(i=0;i<m2->size1;i++)
            gsl_matrix_set(m3, m1->size1+i, j, gsl_matrix_get(m2,i,j));
    }
    return v3;
}

egsl::val egsl::egsl_compose_row(val v1, val v2)
{
    gsl_matrix *m1 = egsl_gslm(v1);
    gsl_matrix *m2 = egsl_gslm(v2);
    egsl_expect_size(v2, m1->size1, 0);
    val v3 = egsl_alloc(m1->size1, m1->size2 + m2->size2);
    gsl_matrix *m3 = egsl_gslm(v3);
    size_t i,j;
    for(i=0;i<m1->size1;i++) {
        for(j=0;j<m1->size2;j++)
            gsl_matrix_set(m3, i, j, gsl_matrix_get(m1,i,j));

        for(j=0;j<m2->size2;j++)
            gsl_matrix_set(m3, i, m1->size2+j, gsl_matrix_get(m2,i,j));
    }
    return v3;
}

void egsl::egsl_add_to(val v1, val v2)
{
    gsl_matrix * m1 = egsl_gslm(v1);
    gsl_matrix * m2 = egsl_gslm(v2);
    gsl_matrix_add(m1,m2);
}

void egsl::egsl_add_to_col(val v1, size_t j, val v2)
{
/*	egsl_print("m1",v1);
	egsl_print("m2",v2); */
    gsl_matrix * m1 = egsl_gslm(v1);
    gsl_matrix * m2 = egsl_gslm(v2);

/*	printf("m1 size = %d,%d j = %d\n",m1->size1,m1->size2,j); */
    egsl_expect_size(v2, m1->size1, 1);
    size_t i;
    for(i=0;i<m1->size1;i++) {
        *gsl_matrix_ptr(m1, i, j) += gsl_matrix_get(m2,i,0);
    }
}


egsl::val egsl::egsl_copy_val(val v1)
{
    gsl_matrix * m1 = egsl_gslm(v1);
    val v2 = egsl_alloc(m1->size1,m1->size2);
    gsl_matrix * m2 = egsl_gslm(v2);
    gsl_matrix_memcpy(m2,m1);
    return v2;
}

egsl::val egsl::egsl_scale(double s, val v1)
{
    val v2 = egsl_copy_val(v1);
    gsl_matrix * m2 = egsl_gslm(v2);
    gsl_matrix_scale(m2, s);
    return v2;
}

egsl::val egsl::egsl_sum(val v1, val v2)
{
    gsl_matrix * m1 = egsl_gslm(v1);
    gsl_matrix * m2 = egsl_gslm(v2);
    val v3 = egsl_alloc(m1->size1,m1->size2);
    gsl_matrix * m3 = egsl_gslm(v3);
    gsl_matrix_memcpy(m3,m1);
    gsl_matrix_add(m3,m2);
    return v3;
}

egsl::val egsl::egsl_sum3(val v1, val v2, val v3)
{
    return egsl_sum(v1, egsl_sum(v2,v3));
}

egsl::val egsl::egsl_mult(val v1, val v2)
{
    gsl_matrix * a = egsl_gslm(v1);
    gsl_matrix * b = egsl_gslm(v2);
    val v = egsl_alloc(a->size1,b->size2);
    gsl_matrix * ab = egsl_gslm(v);
    gsl_blas_dgemm(CblasNoTrans,CblasNoTrans,1.0,a,b,0.0,ab);
    return v;
}

egsl::val egsl::egsl_transpose(val v1)
{
    gsl_matrix * m1 = egsl_gslm(v1);
    val v2 = egsl_alloc(m1->size2,m1->size1);
    gsl_matrix * m2 = egsl_gslm(v2);
    gsl_matrix_transpose_memcpy(m2,m1);
    return v2;
}

egsl::val egsl::egsl_inverse(val v1)
{
    gsl_matrix*A = egsl_gslm(v1);
    val v2 = egsl_alloc(A->size1,A->size1);
    gsl_matrix*invA = egsl_gslm(v2);
    size_t n = A->size1;
    gsl_matrix * m = gsl_matrix_alloc(n,n);
    gsl_matrix_memcpy(m,A);
    gsl_permutation * perm = gsl_permutation_alloc (n);
    /* Make LU decomposition of matrix m */
    int s;
    gsl_linalg_LU_decomp (m, perm, &s);
    /* Invert the matrix m */
    gsl_linalg_LU_invert (m, perm, invA);
    gsl_permutation_free(perm);
    gsl_matrix_free(m);
    return v2;
}

void egsl::egsl_symm_eig(val v, double* eigenvalues, val* eigenvectors)
{
    gsl_matrix *m = egsl_gslm(v);
    size_t N = m->size1;
    /* Check for v to be square */

    gsl_matrix *A = gsl_matrix_alloc(N,N);
    gsl_matrix_memcpy(A, m);

    gsl_vector *eval = gsl_vector_alloc(N);
    gsl_matrix *evec = gsl_matrix_alloc(N,N);

    gsl_eigen_symmv_workspace * ws = gsl_eigen_symmv_alloc(N);
    gsl_eigen_symmv(A, eval, evec, ws);
    gsl_eigen_symmv_free(ws);


    gsl_eigen_symmv_sort(eval, evec, GSL_EIGEN_SORT_VAL_DESC);

    size_t j;
    for(j=0;j<N;j++) {
        eigenvalues[j] = gsl_vector_get(eval, j);
        eigenvectors[j] = egsl_alloc(N,1);
        size_t i;
        for(i=0;i<N;i++)
            *egsl_atmp(eigenvectors[j],i,0) = gsl_matrix_get(evec,i,j);
    }


    gsl_vector_free(eval);
    gsl_matrix_free(evec);
    gsl_matrix_free(A);
}

void egsl::transform(const gsl_vector* p, const gsl_vector* x, gsl_vector*res) {
    double theta = gvg(x,2);
    double c = cos(theta); double s = sin(theta);
    gsl_vector_set(res, 0, c * gvg(p,0) -s*gvg(p,1) + gvg(x,0));
    gsl_vector_set(res, 1, s * gvg(p,0) +c*gvg(p,1) + gvg(x,1));
}

void egsl::gsl_vector_set_nan(gsl_vector*v) {
    gvs(v,0,GSL_NAN);
    gvs(v,1,GSL_NAN);
}

double egsl::norm(const gsl_vector*a){
    double x = gvg(a,0);
    double y = gvg(a,1);
    return sqrt(x*x+y*y);
}

gsl_vector* egsl::vector_from_array(unsigned int n, double *x) {
    gsl_vector * v = gsl_vector_alloc(n);
    unsigned int i;
    for(i=0;i<n;i++)
        gvs(v,i,x[i]);

    return v;
}

void egsl::copy_from_array(gsl_vector*v, double*x) {
    size_t i;
    for(i=0;i<v->size;i++)
        gsl_vector_set(v,i, x[i]);
}

void egsl::vector_to_array(const gsl_vector*v, double*x){
    size_t i;
    for(i=0;i<v->size;i++)
        x[i] = gvg(v,i);
}

void egsl::oplus(const gsl_vector*x1,const gsl_vector*x2, gsl_vector*res) {
    double c = cos(gvg(x1,2));
    double s = sin(gvg(x1,2));
    gvs(res,0,  gvg(x1,0)+c*gvg(x2,0)-s*gvg(x2,1));
    gvs(res,1,  gvg(x1,1)+s*gvg(x2,0)+c*gvg(x2,1));
    gvs(res,2,  gvg(x1,2)+gvg(x2,2));
}

void egsl::ominus(const gsl_vector*x, gsl_vector*res) {
    double c = cos(gvg(x,2));
    double s = sin(gvg(x,2));
    gvs(res,0,  -c*gvg(x,0)-s*gvg(x,1));
    gvs(res,1,   s*gvg(x,0)-c*gvg(x,1));
    gvs(res,2,  -gvg(x,2));
}

void egsl::pose_diff(const gsl_vector*pose2,const gsl_vector*pose1,gsl_vector*res) {
    gsl_vector* temp = gsl_vector_alloc(3);
    ominus(pose1, temp);
    oplus(temp, pose2, res);
    gsl_vector_free(temp);
}

const char* egsl::gsl_friendly_pose(gsl_vector*v) {
    return friendly_pose(v->data);
}

static char egsl_tmp_buf[1024];//TODO:可以将该静态放到头文件中，作为私有变量，应该更加科学！
const char* egsl::egsl_friendly_pose(val v) {    
    sprintf(egsl_tmp_buf, "(%4.2f mm, %4.2f mm, %4.4f deg)",
            1000*egsl_atv(v,0),
            1000*egsl_atv(v,1),
            rad2deg(egsl_atv(v,2)));
    return egsl_tmp_buf;
}

const char* egsl::egsl_friendly_cov(val cov) {   
    double limit_x  = 2 * sqrt(egsl_atm(cov, 0, 0));
    double limit_y  = 2 * sqrt(egsl_atm(cov, 1, 1));
    double limit_th = 2 * sqrt(egsl_atm(cov, 2, 2));

    sprintf(egsl_tmp_buf, "(+- %4.2f mm,+- %4.2f mm,+- %4.4f deg)",
            1000*limit_x,
            1000*limit_y,
            rad2deg(limit_th));
    return egsl_tmp_buf;
}


void egsl::m_trans(const gsl_matrix*A, gsl_matrix*A_t){
    gsl_matrix_transpose_memcpy(A_t,A);
}

void egsl::m_mult(const gsl_matrix*A, const gsl_matrix*B, gsl_matrix*AB){
    gsl_blas_dgemm(CblasNoTrans,CblasNoTrans,1.0,A,B,0.0,AB);
}

void egsl::m_add_to(const gsl_matrix*A, gsl_matrix*B){
    gsl_matrix_add(B, A);
}

void egsl::m_scale(double m, gsl_matrix*A){
    gsl_matrix_scale(A,m);
}

void egsl::m_add (const gsl_matrix*A, const gsl_matrix*B, gsl_matrix*ApB){
    gsl_matrix_memcpy(ApB,A);
    gsl_matrix_add(ApB,B);
}

void egsl::m_inv(const gsl_matrix*A, gsl_matrix*invA) {
    unsigned int n = A->size1;
    gsl_matrix * m = gsl_matrix_alloc(n,n);
    gsl_matrix_memcpy(m,A);
    gsl_permutation * perm = gsl_permutation_alloc (n);
    /* Make LU decomposition of matrix m */
    int s;
    gsl_linalg_LU_decomp (m, perm, &s);
    /* Invert the matrix m */
    gsl_linalg_LU_invert (m, perm, invA);
    gsl_permutation_free(perm);
    gsl_matrix_free(m);
}

double egsl::m_det(const gsl_matrix*A) {
    unsigned int n = A->size1;
    gsl_matrix * m = gsl_matrix_alloc(n,n);
    gsl_matrix_memcpy(m,A);
    gsl_permutation * perm = gsl_permutation_alloc (n);
    int sign;
    gsl_linalg_LU_decomp (m, perm, &sign);
    double det = gsl_linalg_LU_det(m, sign);

    gsl_permutation_free(perm);
    gsl_matrix_free(m);
    return det;
}

double egsl::m_dot(const gsl_matrix*A,const gsl_matrix*B) {
    double sum = 0;
    unsigned int j;
    for(j=0;j<A->size2;j++)
        sum += gmg(A,0,j)*gmg(B,j,0);
    return sum;
}

int egsl::poly_real_roots(unsigned int n, const double*a, double *roots) {
    double z[(n-1)*2];
    gsl_poly_complex_workspace * w  = gsl_poly_complex_workspace_alloc(n);
    if(GSL_SUCCESS != gsl_poly_complex_solve (a, n, w, z)) {
        return 0;
    }
    gsl_poly_complex_workspace_free (w);

    unsigned int i=0;
    for(i=0;i<n-1;i++) {
        roots[i] = z[2*i];
    }
    return 1;
}


int egsl::poly_greatest_real_root(unsigned int n, const double*a, double *root) {
    double z[(n-1)*2];
    gsl_poly_complex_workspace * w  = gsl_poly_complex_workspace_alloc(n);
    if(GSL_SUCCESS != gsl_poly_complex_solve (a, n, w, z)) {
        return 0;
    }
    gsl_poly_complex_workspace_free (w);
    if(TRACE_ALGO) {
        printf("Solving the equation\n a = [");
        for(unsigned int i=0;i<n;i++) {
            printf("%lf ", a[i]);
        }
        printf("]\n");
    }

    unsigned int i;
    double lambda = 0; int assigned = 0;
    for (i = 0; i < n-1; i++) {
        if(TRACE_ALGO) {
            fprintf (stderr, "root z%d = %+.18f + %+.18f i \n", i, z[2*i], z[2*i+1]);
        }
/*		 XXX ==0 is bad */
        if( z[2*i+1]==0 ) /* real root */
            if(!assigned || (z[2*i]>lambda)) {
                assigned = 1;
                lambda = z[2*i];
            }
    }
    if(TRACE_ALGO)
        fprintf (stderr, "lambda = %+.18f \n", lambda);
    if(!assigned) {
        fprintf(stderr, "poly_greatest_real_root: Could not find real root for polynomial.\n");
        fprintf(stderr, "polynomial coefficients : ");
        for (i = 0; i < n; i++)
            fprintf(stderr, " %lf ", a[i]);
        fprintf(stderr, "\nRoots:\n");

        for (i = 0; i < n-1; i++)
            fprintf (stderr, "root z%d = %+.18f + %+.18f i \n", i, z[2*i], z[2*i+1]);

        return 0;
    }

    *root = lambda;
    return 1;
}

void egsl::m_display(const char*str, gsl_matrix*m) {
    printf("%s= \n", str);
    unsigned int i,j;
    for(i=0;i<m->size1;i++) {
        printf("   ");
        for(j=0;j<m->size2;j++)
            printf("%e ", gmg(m,i,j));
        printf("\n");
    }
}
