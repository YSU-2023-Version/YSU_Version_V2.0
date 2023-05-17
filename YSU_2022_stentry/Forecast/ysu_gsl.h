/*
@auther：糊盒员
本部分参考gsl官方文档中中gsl_multifit_nlinear_winit的使用。https://www.gnu.org/software/gsl/doc/html/nls.html
尝试过使用c++风格包装接口，但是当算子为成员变量时会产生内存报错，故放弃。
本部分使用C风格，后续可重写。

*/

#ifndef GSL_H
#define GSL_H


#include"Main/headfiles.h"
//#include <gsl/gsl_rng.h>

//#include <gsl/gsl_randist.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlinear.h>

using namespace std;

//回归计算历史值数量
//#define N      7    /* number of data points to fit */
// #define TMAX   (3.0)  /* time variable in [0,TMAX] */

struct data {
  size_t n;
  double * t;
  double * y;
};


struct ysugsl
{
public:
    const gsl_multifit_nlinear_type *T = gsl_multifit_nlinear_trust;
    const size_t n = 7;//N
    const size_t p = 3;//

    gsl_multifit_nlinear_workspace *w;//workspace
    gsl_multifit_nlinear_fdf fdf;
    gsl_multifit_nlinear_parameters fdf_params;
    gsl_vector *f;
    gsl_matrix *J;//Jacobian行列式
    gsl_matrix *covar ;//协方差
    gsl_vector_view x;
    gsl_vector_view wts;//权重

};

void gslInit(ysugsl *g);

int expb_f (const gsl_vector * x, void *data, gsl_vector * f);//mubiao hanshu


int expb_df (const gsl_vector * x, void *data, gsl_matrix * J);


void callback(const size_t iter, void *params,
         const gsl_multifit_nlinear_workspace *w);


double gsl_compute (ysugsl *g,data d,double aim_time,double *weights);


#endif
