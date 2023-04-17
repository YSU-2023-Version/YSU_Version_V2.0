#include "ysu_gsl.h"
//回归计算精度要求，当两次计算结果相差小于此值则计算结束
const double xtol = 1e-6;
const double gtol = 1e-6;
const double ftol = 0.0;
//回归计算初始值，修改后无过多影响，3表示目标函数中有3个待定系数
double x_init[3] = { 0, 0, 0 }; /* starting values */

//目标函数计算函数，可根据需要修改目标函数，如需修改请务必及时修改其他配套变量
int expb_f (const gsl_vector * x, void *data, gsl_vector * f)//mubiao hanshu
{
  size_t n = ((struct data *)data)->n;
  double *t = ((struct data *)data)->t;
  double *y = ((struct data *)data)->y;

  double A = gsl_vector_get (x, 0);
  double lambda = gsl_vector_get (x, 1);
  double b = gsl_vector_get (x, 2);
//目标函数表达式，可根据需要修改
    for (size_t i = 0; i < n; i++)
      { gsl_vector_set (f, i, A * exp (-lambda * t[i]) + b- y[i]);}

  return GSL_SUCCESS;
}

//jacobian行列式计算函数
int expb_df (const gsl_vector * x, void *data, gsl_matrix * J)
{
  size_t n = ((struct data *)data)->n;
  double *t = ((struct data *)data)->t;

  double A = gsl_vector_get (x, 0);
  double lambda = gsl_vector_get (x, 1);

  for (size_t i = 0; i < n; i++)
    {
      /* Jacobian matrix J(i,j) = dfi / dxj, */
      /* where fi = (Yi - yi)/sigma[i],      */
      /*       Yi = A * exp(-lambda * t_i) + b  */
      /* and the xj are the parameters (A,lambda,b) */
      double e = exp(-lambda * t[i]);
      gsl_matrix_set (J, i, 0, e);
      gsl_matrix_set (J, i, 1, -t[i] * A * e);
      gsl_matrix_set (J, i, 2, 1.0);
    }
  return GSL_SUCCESS;
}

//回调函数，可根据需要查看回归计算过程变量
void callback(const size_t iter, void *params,
         const gsl_multifit_nlinear_workspace *w)
{
//  gsl_vector *f = gsl_multifit_nlinear_residual(w);
//  gsl_vector *x = gsl_multifit_nlinear_position(w);
//  double rcond;
//  /* compute reciprocal condition number of J(x) */
//  gsl_multifit_nlinear_rcond(&rcond, w);

//  fprintf(stderr, "iter %2zu: A = %.4f, lambda = %.4f, b = %.4f, cond(J) = %8.4f, |f(x)| = %.4f\n",
//          iter,
//          gsl_vector_get(x, 0),
//          gsl_vector_get(x, 1),
//          gsl_vector_get(x, 2),
//          1.0 / rcond,
//          gsl_blas_dnrm2(f));
}

void gslInit(ysugsl *g)
{
    /* define the function to be minimized */
    g->fdf.f = expb_f;
    g->fdf.df = expb_df;   /* set to NULL for finite-difference Jacobian */
    g->fdf.fvv = NULL;     /* not using geodesic acceleration */
    g->fdf.n = g->n;
    g->fdf.p = g->p;
}


double gsl_compute (ysugsl *g,data d,double aim_time,double *weights)
{
//变量初始化
   g->fdf_params=gsl_multifit_nlinear_default_parameters();
   g->covar= gsl_matrix_alloc (g->p, g->p);

   g->x= gsl_vector_view_array (x_init, g->p);
   g->wts  = gsl_vector_view_array(weights, g->n);
   double chisq, chisq0;
   int status, info;//biao zhi wei
//   size_t i;

   g->fdf.params = &d;


//回归计算
  /* allocate workspace with default parameters */
  g->w = gsl_multifit_nlinear_alloc (g->T, &g->fdf_params, g->n, g->p);

  /* initialize solver with starting point and weights */
  gsl_multifit_nlinear_winit (&g->x.vector, &g->wts.vector, &g->fdf, g->w);

  /* compute initial cost function */
  g->f = gsl_multifit_nlinear_residual(g->w);
  gsl_blas_ddot(g->f, g->f, &chisq0);

  /* solve the system with a maximum of 100 iterations */
  status = gsl_multifit_nlinear_driver(20, xtol, gtol, ftol,callback, NULL, &info, g->w);

  /* compute covariance of best fit parameters */
  g->J = gsl_multifit_nlinear_jac(g->w);
  gsl_multifit_nlinear_covar (g->J, 0.0, g->covar);

  /* compute final cost */
  gsl_blas_ddot(g->f, g->f, &chisq);


  return gsl_vector_get(g->w->x,0)* exp (-1*gsl_vector_get(g->w->x,1) * aim_time) + gsl_vector_get(g->w->x,2);
}
