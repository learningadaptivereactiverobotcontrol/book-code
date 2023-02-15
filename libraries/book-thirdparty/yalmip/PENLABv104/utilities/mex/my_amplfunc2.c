/* AMPL Interface to Matlab as a mex file
 * 
 * Adjusted version, my 'triple' interface
 * version: 0.45
 * last update: 18/03/07
 *
 * Note: As a start amplfunc.c & spamfunc.c written by Ampl were used
 * and incorporated into interface. (Currently, MODE 1 Dense and Sparse
 * mode should be the same as amplfunc.c and spamfunc.c, respectively.)
 * Interface was expanded by new 3 modes more convinient and/or similar
 * to Pennlp/optimization code.
 * The original licence is included below.
 *   
 * List of Changes:
 * ----------------
 * corrected: my_permalloc(), BOUNDS in hessian
 * added new raw interface (as MODE 2)
 * corrected: += g_i*g_i' in Hessian computation; gigrd_tmp array for gi_grad()
 * added bequal_m2 for MODE 2 for possibility to adjust order of multipliers from Pennlp
 * added possibility not to read [x_guess,v_guess] while initialization in MODE 2/3/4
 * added conditions if there are no [equal/general] constraints not to alloc memory block
 *   of length zero (sometimes it could cause problems since Malloc by Ampl alloc at least 
 *   something to return nonzero pointer but this could have been confusing sometimes for
 *   the code)
 *   Now it should be strictly controlled - if there are no constraints, neither allocation
 *   nor mecpy nor whatever is invoked.
 * corrected the sign of the objective function, OBJSIGN & objtype[] handling, should be fixed
 *   in case that there is no objective function in the file
 *
 * TO DO List:
 * -----------
 * transform v_guess (initial guess of L. multipliers) that it fits to
 *   transformed constraints
 * include sparse computation of hessian matrix in MODE 3
 * update usage()
 * expand info mode (information about opened problem when usage() is called)
 * write proper documentation
 */

/*************Original*Licence***********************************
Copyright (C) 1997-1998, 2000 Lucent Technologies
All Rights Reserved

Permission to use, copy, modify, and distribute this software and
its documentation for any purpose and without fee is hereby
granted, provided that the above copyright notice appear in all
copies and that both that the copyright notice and this
permission notice and warranty disclaimer appear in supporting
documentation, and that the name of Lucent or any of its entities
not be used in advertising or publicity pertaining to
distribution of the software without specific, written prior
permission.

LUCENT DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS.
IN NO EVENT SHALL LUCENT OR ANY OF ITS ENTITIES BE LIABLE FOR ANY
SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER
IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF
THIS SOFTWARE.
****************************************************************/

/*
 *  Raw interface - MODE 2
 *  every problem is transformed to
 *    min   f(x) 
 *    s.t.  g_i(x)<=0 
 *          h_i(x)=0
 *  even box constraints are included as g_i(x), equalities in
 *  box constraints are strictly forbidden (could be simplified)!
 *
 *  global variables 'uploaded' to Matlab
 *    MODE ... current mode, e.i., 2
 *    DENSE .. density mode (1..dense problem, 0..sparse problem)
 *    N ...... number of variables
 *    N_BOUNDS ... number of box constraints as tranformed as 
 *       inequalities g_i(x)
 *    N_INEQUAL ... total number of inequalities g_i(x) (inc. box!!!)
 *    N_EQUAL ..... total number of equality constraints h_i(x)
 *    N_CONSTR .... total number of all constraints (=N_INEQUAL+N_EQUAL) 
 *    BEQUAL_M2[] . dim N_CONSTR, flags of equalities among constraints without
 *       changing order of constraints, e.i., same as BEQUALITY array generated
 *       in Pennlp or in MODE 3. In MODE 2 there are all inequality constraints
 *       followed by equality ones (e.i., internal BEQUALITY[] array in the case
 *       would be 0,0,0...,0,1,1,...,1) in constrast to BEQUALITY[] in 
 *       Pennlp/MODE 3 where all constraints are mixed together. BEQUAL_M2[] array
 *       serves to be able to transform Pennlp/MODE 3 multipliers to multipliers
 *       in MODE 2 (Note: relative order among all equality constraints is
 *       preserved, the same with inequalities)
 *    N_LNLCONSTR . total number og eq. & ineq. constraints (without box), 
 *        e.i., N_LNLCONSTR = N_CONSTR-N_BOUNDS = N_EQUAL+N_INEQUAL-N_BOUNDS
 *        not necessary, just for compatibility
 *  internal important global
 *    NPERM ...... (dim N_CONSTR) permutation of constraints how they
 *        are transformed
 *      first N_BOUNDS (0..N_BOUNDS-1) are transformed boxed constraints,
 *        i or -(i+1) aims to real number of variable constraint
 *      N_BOUNDS..N_CONSTR-1 are the rest inequality & equality constraints
 *        i or -(i+1) aims to the rest of linear/nonlinear constraints
 *      numbers i-th constraints are 0-based
 *  usage
 *    [x_guess,v_guess]=fce('name.nl',2[,dense_flag]) initiate problem, allocate
 *        memory and return initial guess of starting point and lagrangian multipliers
 *        (output is optional, possible to call just fce(...) without output)
 *    [f,g,h] = fce(x,0)  function values of f and all constraints
 *    [grad f, grad g, grad h] = fce(x,1)  gradients, column-wise
 *    H=fce(v)  hessian H=nabla^2 f + sum v_i nabla^2 g_i + sum v_(i-..) nabla^2 h_i
 *        'v' could be of size N_CONSTR, but first N_BOUNDS elements should
 *        correspond to box constraints and therefore they are ignored in hessian,
 *        or it could have dimension N_LNLCONSTR and then it is expected to
 *        correspond to lagrangian multipliers of all 'true' constraints
 *
 */

/*
 *  Initialization:  ?? = func(stub_nl_name, mode, dense/sparse)
 *  dense/sparse is optional, default is used (see macro bellow)
 *  returns [x,v] initial guess for modes 2/3/4
 *  computation:
 *    fce(x,0)
 *    fce(x,1)
 *    fce(v)
 *  MODE 4 ignores all inequalities (equal only)
 *     f,equal / grad f, Jac A of equal / hes lagr with eq. only
 *  MODE 3 - Pennlp mode: aug.lagr.F,equal / grad F, Jac A of equal / hes augm. lagr.
 *     penalty and multipliers are needed --> global arrays
 * writing results [not tested!]
 *
 *  
 */


/* Raw interface - MODE 1  (aka amplfunc.c/spamfunc.c)
   Sample mex function (in MATLAB 5.x format) for getting functions,
   gradients, and dense Hessians from an AMPL .nl file.  Start with

	[x,bl,bu,v,cl,cu] = amplfunc('stub')

   or, for complementarity problems,

	[x,bl,bu,v,cl,cu,cv] = amplfunc('stub')

   to read in a problem (discarding the previous problem, if any).
   The return values are:

	x = primal initial guess
	v = dual initial guess
	bl, bu = lower and upper bounds on x
	cl, cu = lower and upper bounds on c (the constraint bodies).
	cv variables complementing constraints:  if cv(i) > 0, then
		constraint i complements x(cv(i)); otherwise
		constraint i is an ordinary constraint.

   Then

	[f,c] = amplfunc(x,0)

   gives the function (f) and constraint bodies (c) at x;

	[g,Jac] = amplfunc(x,1)

   gives the gradient g of f, the Jacobian matrix J of c, and

	W = amplfunc(v)

   gives the Hessian W of the Lagrangian function L = f + v*c
   (at the last x at which amplfunc(x,1) was called).

   After finding optimal values for x and v,

	amplfunc('solution message',x,v)

   to write a stub.sol file.

   Note: The interface doesn't recognise min/max in the formulation in the problem,
   it is expected to be minimisation always.
*/

#include "mex.h"
#undef printf
#include "asl_pfgh.h"

#ifdef _WIN32
/* Omit sw "signal" catching and x86 precision adjustment. */
#define ASL_NO_FP_INIT
#include "fpinit.c"
#endif /* _WIN32 */

/* Penalty function (MODE 3 only) */
extern double phi2(double);
extern double D_phi2(double);
extern double D2_phi2(double);
double R = -0.5;   // for penalty function (used only if MODE==3)

//////////////////////////////////////////////////////////////////////////////////////////
//#define MEMORY_MATLAB    // prefer using mxMalloc (from Matlab) to M1alloc (from AMPL)

/* If MEMORY_MATLAB handling is turned on, it is needed to store a list od all permanent
   allocated objects to have a possibility to release them all. These functions do so. */
#ifdef MEMORY_MATLAB

/* list of all allocated structures by mxMalloc and mexMakeMemoryPersistent */
typedef struct _perm_mem_list PM_LIST;
struct _perm_mem_list {
  void *p;
  PM_LIST *next;
};

static PM_LIST *MEX_PM_LIST=0;

/* free all the memory stored in the list and set start to 0 */
void pml_free(PM_LIST **start) {
  PM_LIST *elt=*start, *to_free;
  int i=0;

  while (elt) {
    if (elt->p)
      mxFree(elt->p);
    to_free=elt;
    elt=elt->next;
    mxFree(to_free);
    i++;
  }
  *start=0;
  mexPrintf("Persistant structures freed: %i\n",i);
}

/* allocate one element for new memory, if insufficient memory, mxMalloc() terminates the MEX
   file and control is returned to Matlab */
PM_LIST *pml_get(void *p) {
  PM_LIST *elt;

  elt = (PM_LIST *) mxMalloc(sizeof(PM_LIST));
  mexMakeMemoryPersistent(elt);
  elt->p=p;
  elt->next=0;
  return elt;
}

/* Allocate memory via mxMalloc and make it persistant and store it into the list MEX_PM_LIST
   if there is not enough memory, mxMalloc terminates and returns to Matlab prompt */
void *my_permalloc(size_t size) {
  void *ptr;
  PM_LIST *elt;

  ptr = mxMalloc(size);
  // on error terminates back to Matlab

  mexMakeMemoryPersistent(ptr);
  
  // register it into the list
  elt = pml_get(ptr);
  elt->next=MEX_PM_LIST;
  MEX_PM_LIST=elt;

  return ptr;
}

/* PROBLEM:
  !!!! mozna to nebude fungovat vzhledem k tomu, ze cast se takto naalokuje i do struktury
  ASL, ktera se sama uvolni ... to muze nadelat docela bordel, obzvlast, ze ji to uvolni
  pres normalni free() a ne mxFree() !!!!
  proto soucasti ASL struktury je potreba alokovat vzdy pres M1alloc a jen zbytek volit
  AMPL/MATLAB memory management 
  -----> PROBLEM SOLVED <------ */
#endif // of MEMORY_MATLAB

/*my_pmalloc ... permanent memory allocation using either mxMalloc or M1alloc (AMPL) */
#ifdef MEMORY_MATLAB
#define my_pmalloc my_permalloc
#else
#define my_pmalloc M1alloc
#endif

//////////////////////////////////////////////////////////////////////////////////////////

#define MY_ERROR          -1   // return flag considered as an error, 
       // e.i., out of acceptable range of parameters
#define DEFAULT_DENSE      0   // in case the dense flag is not used through interface

static char msgbuf[256];
static char *what; // for error jump while computing gradient/hessian/...

static int DENSE=0;
static int MODE=0;

/* DENSE ... 0..treat problems as sparse, 1..dense
 * MODE .... 1..raw interface (original by Ampl),
 *           2..raw transformed interface to min f(x) s.t. g(x)<=0 & h(x)=0
 *           3..Pennon mode
 *           4..ignore-inequalities-mode
 */

/* Transformation of i-th constraint lower_bound <= something <= upper_bound to something<=0 
   is done in the following way (note that bound can be equal to +/- infinity):
   somtheing-lower ==0  if (lower==upper)   --> NPERM contains i, BEQUALITY 1
   lower - something<=0 if (lower>-infty)  -->  NPERM contains -(i+1), BEQUALITY 0
   something - upper<=0 if (upper<infty)   -->  NPERM contains i, BEQUALITY 0

   N_CONSR ..... total number of all tranformed constraints (sth<=0 / sth==0)
   N_BOUNDS .... 1st N_BOUNDS from N_CONSTR are boxed constraints
   N_LNLCONSTR . the rest is linear/nonlinear
   N_EQUAL ..... total number of equality constr., should be only in L/NL part (not in box c.)
   BEQUALITY[] . dim N_CONSTR, flags of equalities among constraints
   NPERM[] ..... dim N_CONSTR, i or -(i+1) which refer to i-th box or L/NL constraint in the
     original formulation  
*/
static int *BEQUALITY=0;   
static int *NPERM=0;
static int N=0;            // no of variables
static int N_CONSTR=0;     // no of constraints in total
static int N_BOUNDS=0;     // box constraints
static int N_LNLCONSTR=0;  // linear + nonlinear constraints (inc. equalities)
static int N_EQUAL=0;      // no of equality constr.
static int N_INEQUAL=0;    // no of ineq. constraints (inc. box), MODE 2
static int OBJSIGN=0;      // 1 for minimisation, -1 for maximisation, 0 for no objective function
static double *PENALTY=0;    // penalties for MODE==3 updated by Matlab global variable
static double *U=0;
  // docela nekonzistentni ne? Kdyz z Matlabu mam nastavovat PENALTY a U kdyz ani nevim jejich rozmery? Asi bych mel alespon N_CONSTR poslat jako globalni, mozna i BEQUALITY

static ASL *asl=0;

// temporary not-nice-but-work solution
static real *gigrd_tmp=0;  // real array dim N, used only by gi_grad()

//////////////////////////////////////////////////////////////////////////////////////////

// check if the dimensions and type are OK
#define MXISOK(var,m,n)  (mxIsNumeric(var) && !mxIsComplex(var) && mxGetM(var)==(m) && mxGetN(var)==(n))

/* Returns a pointer to the global variable with 'name', 0 on error (doesn't exist,etc)
   Assume that the variable is global, real matrix with dimensions mxn (rows x columns)*/
static double *get_global(const char *name, int m, int n) {
  mxArray *var;
  
  if (name && (var=mexGetVariable("global",name))!=0 && MXISOK(var,m,n))
    return mxGetPr(var);
  else
    return 0;
}

/* "upload" a scalar as a Matlab global variable, on error returns 0, otherwise 1 */
static int set_global_scalar(const char *name, double value) {
  mxArray *var;
  int ret=1;

  var = mxCreateDoubleMatrix(1,1,mxREAL);
  /* on error it stops computing and the programme is terminated */

  *mxGetPr(var) = value;
  if (!name || mexPutVariable("global",name,var)) 
    ret=0;
  mxDestroyArray(var);
  return ret;
}

/* adjust global variable (scalar) if is not set or has different value 
   returns 0 on error, 1 was the same (adjusting not needed), 2 adjusted,
   if warning_flag, synchronization is signalized by text */
static int adjust_global_scalar(const char *name, int value, int warning_flag) {
  double *glob_var;

  if (!name) 
    return 0;
  else if ((glob_var=get_global(name,1,1))==0 || *glob_var!=value) {
    if (warning_flag) {
      if (glob_var)
        mexPrintf("WARNING: global variable %s is synchronized (%i -> %i)\n",name,(int) *glob_var,value);
      else
        mexPrintf("WARNING: global variable %s is set (-> %i)\n",name,value);
    }

    if (!set_global_scalar(name,value)) {
      mexPrintf("ERROR: Synchronization of '%s' failed\n",name);
      return 0;
    }
    return 2;
  } else
    return 1;
}

/* 'upload' (~write to matlab) integer array of dimension dim as a Matlab global
   variable 'name', written as a double array (because of a bug? in Matlab while
   working with global integer array)
   returns 0 on error, 1 otherwise */
static int set_global_iarr(const char *name, int *iarr, int dim) {
  int i;
  double *dtmp;
  mxArray *var;

  if (!name || !iarr || dim<0)
    return 0;
  var = mxCreateDoubleMatrix(dim,1,mxREAL);
  for (dtmp=mxGetPr(var),i=0; i<dim; i++)
    *dtmp++ = *iarr++;
  if (mexPutVariable("global",name,var))
    return 0;
  return 1;
}

/* Read and check global variables U (multiplicators) and PENALTY (penalty parameters),
   if MATLAB variables are inconsistent use 1.0 by default (+warning) */
void read_all_globals(void) {
  double *des, *src;
  int i;
  int warning=0;

  // no constraints --> nothing to do at all (these are empty structures so why to bother...)
  // Note: if (!N_CONSTR) then PENALTY[]/U[] is not even allocated and are equal to 0
  if (!N_CONSTR)  
    return;

  // read multipliers U
  if ((src=get_global("U",N_CONSTR,1))==0) {
    mexPrintf("WARNING: Global U is inconsistent, using default multipliers (1.0)\n");
    for (des=U, i=N_CONSTR; i>0; i--)
      *des++=1.;
  } else {
    memcpy(U,src,N_CONSTR*sizeof(double));
  } 

  // read PENALTY parameters
  if ((src=get_global("PENALTY",N_CONSTR,1))==0) {
    mexPrintf("WARNING: Global PENALTY is inconsistent, using default penalty parameters (1.0)\n");
    for (des=PENALTY, i=N_CONSTR; i>0; i--)
      *des++=1.;
  } else {
    for (des=PENALTY,i=0; i<N_CONSTR; i++, src++) {
      // *des++ = *src>0 ? *src: 1.0;
      if (!BEQUALITY[i] && *src<=0) {
	if (!warning) {
          mexPrintf("WARNING: Some of PENALTY parameters (in global array) are inconsistent,\nusing default penalty parameters (1.0)\n");
	  warning=1;
	}
        *des++ = 1.0;
      } else
        *des++=*src;
    }
  } 
}

/* Check if the parameter is a real scalar and return its value, otherwise return MY_ERROR */
static int scalarchk(const mxArray *var) {
  if (var && MXISOK(var,1,1))
    return (int) *mxGetPr(var);
  else
    return MY_ERROR;
}

/* Check if the size is m x 1 */
/* JF: shouldn't be there double instead of real?? I know that it's the same (now) but... */
/* Function from original files amplfunc.c/spamfunc.c ... why is it so strange?? */
static double* sizechk(const mxArray *mp, char *who, fint m)
{
	int m1, n1;
	m1 = mxGetM(mp);
	n1 = mxGetN(mp);
	if (m1 != m || n1 != 1 && m1) {
		sprintf(msgbuf,
			"Expected %s to be %d x 1 rather than %d x %d\n",
			who, m, m1, n1);
		mexErrMsgTxt(msgbuf);
		}
	return mxGetPr(mp);
}

//////////////////////////////////////////////////////////////////////////////////////////

/* free static structures */
static void at_end(void)
{
  if (cur_ASL)
    ASL_free(&cur_ASL);
    // frees all memory allocated by M1alloc() automaticaly as well

#ifdef MEMORY_MATLAB
  pml_free(&MEX_PM_LIST);
#endif
  // to let gi_grad() know that it must allocate gigrd_tmp again
  gigrd_tmp=0;
}

// !!!! TO DO - fix this !!!!!!!!!!!!!!
//static void usage(void)
static void usage(const char *s)
{
  // print information of the problem
  if (asl) {
    mexPrintf("Currently problem '%s' is opened, in MODE %i as %s\n",filename,MODE,DENSE?"DENSE" : "SPARSE");
    if (MODE==2 || MODE==3) {
      mexPrintf("  no. of variables: %i\n",N);
      mexPrintf("  total number of constraints: %i\n",N_CONSTR);
      mexPrintf("    including box constraints: %i\n",N_BOUNDS);
      mexPrintf("          and 'true' constr.:  %i\n",N_LNLCONSTR);
      mexPrintf("    total ineq. c.:  %i\n",N_INEQUAL);
      mexPrintf("    total equal. c.: %i\n",N_EQUAL);
      mexPrintf("  \n");

    }
  } else {
    mexPrintf("No problem assigned yet.\n");
  }
  mexPrintf("--- %s ---\n\n",s);
  mexErrMsgTxt("amplfunc usage:\n\n\
    [x,bl,bu,v,cl,cu] = amplfunc('stub')\nor\n\
    [x,bl,bu,v,cl,cu,cv] = amplfunc('stub')\nor\n\
    [f,c] = amplfunc(x,0)\nor\n\
    [g,Jac] = amplfunc(x,1)\nor\n\
    W = amplfunc(v)\nor\n\
    amplfunc('solution message',x,v)\nor\n\
    amplfunc('solution message',x,v,solve_result_num)\nwith\n\
    x = primal, v = dual variables");
}

#if 0
/* for testing */
void ww(const char *s) {
  static int run=0;

  run++;
  mexPrintf("Going (%s)...%i\n",s,run);
  mexEvalString("pause");
  mexPrintf("Cont...\n");
}
#endif

/* 'real' is probably defined as double but it would be better not to use memcpy, 
   therefore this function
   real to double copy, size of both is arrays no_items, return pointer 'to' */
double *r2d_copy(double *to, const real *from, int no_items) {
  double *dest;

  if (to && from && (double *) from!=to) {
    for (dest=to;no_items>0;no_items--)
      *dest++=*from++;
  }
  return to;
}

real *d2r_copy(real *to, const double *from, int no_items) {
  real *dest;

  if (to && from && (real *) from!=to) {
    for (dest=to;no_items>0;no_items--)
      *dest++=*from++;
  }
  return to;
}

/* Copy array fint to int, fint is probably defined as long */
int *fi2i_copy(int *to, const fint *from, int no_items) {
  int *dest;

  if (to && from && (void *)from!=(void *)to)
    for (dest=to; no_items>0; no_items--)
      *dest++=*from++;
  return to;
}

//////////////////////////////////////////////////////////////////////////////////////////

/* count number of transformed constraints if there are dim-constraints in the form
   lower[i] <= sth <= upper[i],
   (0=no restriction, 1=equality or 1 inequality, 2=two inequalities
   return number of constraints and number of equalities */
int count_constr(real *lower, real *upper, int dim, int *noequal) {
  int i, no=0;

  *noequal=0;
  for (i=0; i<dim; i++) {
    if (lower[i]>negInfinity && lower[i]!=upper[i])
      no++;
    if (upper[i]<Infinity)
      no++;
      // lower[i]==upper[i] counts here automaticaly as well
    if (lower[i]==upper[i])
      (*noequal)++;
  }
  return no;
}

/* set an equality flag in beq[dim2] and permutaion info in nprm[dim2] based on restrictions
   in lower[dim] and upper[dim], where
   dim2 is a new number of tranformed constraints
   equaility flag ~ 1 for equality, 0 ~ not an equality
   permutation info ~ i or -(i+1) for upper or lower restriction, respectively */
int set_constr(real *lower, real *upper, int dim, int *beq, int *nprm) {
  int i;
  int no_eq=0;

  for (i=0; i<dim; i++) {
    if (lower[i]>negInfinity && lower[i]!=upper[i]) {
      *beq++=0;
      *nprm++=-(i+1);
    }
    if (upper[i]<Infinity) {
      *nprm++=i;
      if (lower[i]!=upper[i])
	*beq++=0;
      else {
        *beq++=1;
	no_eq++;
      }
    }
  }
  return no_eq;
}

/* Similar ot set_constr() but rearrange constraints, all inequality constraints go first and then
 * they are followed by all equality constraints. Do NOT expect equality constraints in bounds
 * constraints otherwise it would cause serious problems (and even crash of the program)
 *
 * In contrast to set_constr(), beq[] and nprm[] should be called always 'not-shifted' 
 * (e.g., not set_constr_eq(...,NPERM+N_BOUNDS,...)), set start end eqstart instead. They say
 * where to start writing new items into the arrays and are changed approprietely if any
 * item was written.
 * returns number of written entries and changes start&eqstart */
  // do not expect any patological examples low=upp=negInfty
int set_constr_eq(real *lower, real *upper, int dim, int *beq, int *nprm, int *start, int *eqstart) {
  int i, pos, eqpos;  /* position in nprm[] ... and for equalities */

  for (pos=*start, eqpos=*eqstart, i=0; i<dim; i++) {
    if (lower[i]==upper[i]) { 
      beq[eqpos]=1;
      nprm[eqpos++]=i;
    } else {
      if (lower[i]>negInfinity) {
        beq[pos]=0;
        nprm[pos++]=-(i+1);
      }
      if (upper[i]<Infinity) {
        beq[pos]=0;
        nprm[pos++]=i;
      }
    }
  }
  i = pos - *start + eqpos - *eqstart;
  *start=pos;
  *eqstart=eqpos;
  return i;
}

/* compute value of the i-th (transformed) constraint (inc. equalitites),
   if i is out-of range returns 0 */
double gi_val(unsigned i, real *x, real *constr_val) {
  double val=0;
  int j;

  if (i<N_BOUNDS) {  // box constraint
    if ((j=NPERM[i])<0) {
      j=-(j+1);
      val = LUv[j] - x[j];
    } else
      val = x[j]-Uvx[j];
  } else if (i<N_CONSTR) {  // linear/nonlinear constraint
    if ((j=NPERM[i])<0) {
      j=-(j+1);
      val = LUrhs[j] - constr_val[j];
    } else
      val = constr_val[j]-Urhsx[j];
  }
  return val;
}

/* compute i-th gradient of (transformed) constraint at point x,
   if ind==0 then store the result in the whole vector grd[N],
   if not, store nonzeroes only and return their 0-based indices in ind[]
   return number of nonzero elements in the gradient */
int gi_grad(unsigned i, real *x, real *grd, int *ind) {
  int j, nnz=0, sign=1;
  cgrad *cg;
  real *dtmp=grd;

  if (!gigrd_tmp)
    gigrd_tmp = (real *) M1alloc(N*sizeof(real));
    // if error, M1alloc calls exit(1); directly

  if (!x || !grd)
    return 0;

  if (!ind) {
    memset(grd,0,N*sizeof(real));
    dtmp=gigrd_tmp;
  }

  if (i>=N_CONSTR)
    return 0;

  if ((j=NPERM[i])<0) {
    j=-(j+1);
    sign = -1;
  }
  what="c grad";

  if (i<N_BOUNDS) {
    if (ind) {
      ind[0]=j;
      grd[0]=sign;
      nnz=1;
    } else {
      grd[j]=sign;
      nnz=1;
    }
  } else if (i<N_CONSTR) {  // linear/nonlinear constraint
    congrd(j,x,dtmp,0);
    if (ind) {  // Note: dtmp==grd
      for (nnz=0,cg=Cgrad[j]; cg; cg=cg->next,nnz++) {
        ind[nnz]=cg->varno;
	grd[nnz]*=sign;
      } 
    } else {
      for (nnz=0,cg=Cgrad[j]; cg; cg=cg->next,nnz++) {
	      // opravdu to funguje??? je to ulozene v cg->coef???
        grd[cg->varno] = sign*dtmp[nnz];
      } 
    }
  }
  return nnz;
}

/* copmute value of penalty function F(x,u,p) = f(x) + sum u*p*phi(g(x)/p) for ineq. constr.
   in constraints_x are stored actual values of all transformed constraints,
   f_val is properly transformed based on OBJSIGN */
double penfunc_val(double f_val, real *constraints_x) {
  double val=f_val;
  unsigned i;

  for (i=0; i<N_CONSTR; i++) {
    if (!BEQUALITY[i])
      val += U[i]*PENALTY[i]*phi2(constraints_x[i]/PENALTY[i]);
  }
  // OBJSIGN applied to f_val already while calling function
  return val;
}

/* compute gradient of penalty function F(x,u,p)
   store it in res (result), expect on input x (dim N), constr_x (dim N_CONSTR, values
   of transformed constraints at x), f_grad (dim N) [Note: f_grad should be already transformed
   based on OBJSIGN, use f_grad==0 if objective function is missing.]
   dtmp & itmp (dim N)
   doesn't check input, just to make code easy to read */
void penfunc_grad(double *res, real *x, real *constr_x, real *f_grad, real *dtmp, int *itmp) {
  unsigned i,j,nnz;
  double mlt;

  if (f_grad)
    r2d_copy(res,f_grad,N);
  else
    memset(res,0,N*sizeof(double));

  for (i=0; i<N_CONSTR; i++) {
    if (BEQUALITY[i])
      continue;
    mlt = U[i]*D_phi2(constr_x[i]/PENALTY[i]);
    nnz=gi_grad(i,x,dtmp,itmp);
    for (j=0;j<nnz;j++)
      res[itmp[j]]+=mlt*dtmp[j];
  }
}

/* Functions for computation of the Jacobian of constraints
   First call jac_struc_jc() to find out the number of nonzeroes and lengths of
   columns, then jac_struc_ir() to compute row indices of elements and elements
   themselves. If the structure is known it is sufficient to call directly
   jac_values() to compute double data elements and copy the structure.
   Typical usage:
     if (!nnz) {
       nnz=jac_struc_jc(my_jc,...);
       my_ir=my_pmalloc(nnz*...);
       plhs[]=mxCreateSparse();
       jac_struc_ir();
     } else {
       plhs[]=mxCreateSparse();
       jac_values();
     }
     memcpy(,my_ir,);
     memcpy(,my_jc,);
  Common parameters:
    To the Jacobian use only subset of constraints with indices
    i=start,..,end-1 (of N_CONSTR) and only equalities based on
    BEQUALITY[i], or ignore it and use all (if (ignore_beq)).
    x .. point where to compute gradients
    dtmp, itmp ... allocated arrays of length at least N
    jc ... column structure
    ir ... row indices of elements
    elm .. (allocated) array for values of elements
*/
/* Compute total number of nonzeroes and lengths of columns in Jacobian,
   returns total nonzeroes */
static fint jac_struc_jc(int *jc, int ignore_beq, int start, int end, real *x, real *dtmp, int *itmp) {
  int i, j;

  jc[0]=0;
  for (j=1, i=start; i<end; i++)
    if (ignore_beq || BEQUALITY[i])
      jc[j++]=gi_grad(i,x,dtmp,itmp);	
  for (i=1; i<j; i++)
    jc[i] += jc[i-1];
  return jc[j-1];
}

/* Compute ir[] and fill in elm[] based on existing jc[] computed by jac_struc_jc()
   (with the same settings of start, end, ignore_beq!)
   returns number of columns filled in
   Note: in fact, jc[] is not needed and you can fill ir[] & elm[] sequentially */
static int jac_struc_ir(int *jc, int *ir, double *elm, int ignore_beq, int start, int end, real *x, real *dtmp, int *itmp) {
  int i,j, nnz;

  for (j=0,i=start; i<end; i++)
    if (ignore_beq || BEQUALITY[i]) {
      nnz = gi_grad(i, x, dtmp, itmp);
      memcpy(ir+jc[j], itmp, nnz*sizeof(int));
      r2d_copy(elm+jc[j], dtmp, nnz);
      j++;
    }
  return j;
}

/* Computes values of nonzero elements, expect existing jc[] and sufficiently
   allocated memory in elm[], returns number of columns */
static int jac_values(int *jc, double *elm, int ignore_beq, int start, int end, real *x, real *dtmp, int *itmp) {
  int i,j,nnz;

  for (j=0,i=start; i<end; i++)
    if (ignore_beq || BEQUALITY[i]) {
      nnz = gi_grad(i, x, dtmp, itmp);
      r2d_copy(elm+jc[j], dtmp, nnz);
      j++;
    }
  return j;
}

/* Fill in allocated memory of dense jacobian, meaning of parameters is the same,
   elm[] should be allocated as double array N*no_of_constraints(=columns)
   returns no of columns */
static int djac_values(double *elm, int ignore_beq, int start, int end, real *x, real *dtmp) {
  int i,j;

  for (j=0,i=start; i<end; i++)
    if (ignore_beq || BEQUALITY[i]) {
      gi_grad(i, x, dtmp, 0);
      r2d_copy(elm,dtmp,N);
      elm+=N;
      j++;
      // note: don't forget that Matlab is column-wise!
    }
  return j;
}


//////////////////////////////////////////////////////////////////////////////////////////
// NOTE: Not used since sparse hessian in MODE 3 is not finished

/* Project Hessian of Lagrangian (from sphes()) to Hessian of Augmented Lagrangian */
void projectH(double *to, real *from, double coef, int no_items, int *lookup) {
  for (;no_items>0;no_items--)
    to[*lookup++] += coef * *from++;
}

/* Project a dyadic product of sparse vector (nnz, data=values of elements[,ind=rowindices])
   to Hessian of Augmented Lagrangian */
void projectG(double *to, int nnz, real *data, double coef, int *lookup) {
  int i,j;

  for (i=0; i<nnz; i++)
    for (j=0; j<nnz; j++)
      to[*lookup++] += coef * data[i]*data[j];
}

//////////////////////////////////////////////////////////////////////////////////////////

/*
 * Notes:
 * a) If MODE==2/3/4 and structure of Jacobian of constraints is empty (e.g., no
 * equality constraints), the test if (!nnz_equal_Jac) wasn't satisfying because
 * in the case nnz_equal_Jac is always zero and the code always tries to
 * generate the structure again and again.
 * There shouldn't be a problem with functions jac_struc*(), jac_values()
 * there is just an empty loop and nothing is written down or filled in.
 * Therefore nnz=-1 is used to identify unknown structure.
 * mxCreate*(0,0,...) generates an empty structure, so it is fine.
 * Note fint is defined as long--> -1 should be OK
 *
 * b) equality constraint in box constraint WILL cause serious damage, they are not
 * expected in box coinstraints-->their treatment is very dangerous!!!
 *
 * c) box coinstraints (with i=0,..N_BOUNDS-1) shouldn't be included in hessian
 * at position \nabla^2 g_i(x) (because it is zero), otherwise something suspicious
 * is written to L_mlt() or even in an element which doesn't exist (e.g., when nc=0, 
 * e.i., no 'real' constraints is L_mlt[] empty; attempt to write there due to
 * multipliers of box constraints caused crash - see stub.nl, now it should be OK).
 * Note that in the hessian of augmented lagrangian they are included because of
 * terms sth* grad g_i * grad g_i^T (e.i., dyadic multiplication part).
 *
 * d) Jacobian of constraints should be OK, A=[grad g_1, grad g_2, ...]
 * e.i., gradients are stored in columns --> size should be N x no_con
 * Matlab stores matrices column-wise so it is OK to copy it one by one
 * as blocks of memory
 * MODE 1 (original by AMPL) has transposed Jacobian!! dimensions of nc x N !!!
 * (in dense as well as in sparse mode)
 * in Pennlp we also store A as N x no_constr, gradients in columns even if in C
 * matrices are in row-wise sense
 *
 */


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  static int nc;  // number of (untransformed) constraints, e.i., dim of congrad,...
  static char ignore_complementarity[] =
    "Warning: ignoring %d complementarity conditions.\n";

  static real *constr_val=0;    // (untransformed) constraints function values
  static real *constraints_x=0; // (transformed) constraints function values (MODE==3 only)
  static real *f_grad=0;    // gradient of the first objective
  static real *whole_J=0;     // full sparse matrix for (unsorted Jacobian)
  static real *whole_Hsp=0;   // full sparse Hessian of Lagrangian
  static fint nnz_whole_Jac;  // nnz in Jacobian of UNtrunsformed constraints (MODE==1)
  static fint nnz_whole_Hes;  // nnz in Hessian of all untr. constr. (MODE==1)
  static fint nnz_equal_Jac;  // nnz in sparse Jacobian of transformed equality constraints (MODE==2/3)
  static fint nnz_inequal_Jac;
  static fint nnz_Hes_Lagr;   // nnz in sparse Hessian of Lagrangian (MODE=2/3/4, but in MODE 3 an additional expansion of hessian is needed)
  static fint nnz_Hes_ALagr;   // nnz in sparse Hessian of Augmented Lagrangian (MODE=3)
  static real *gi_grd=0;    // temporary array for one gradient of constraints and its indices of nnz
  static int *gi_gind=0;
  static int *Ir_equal_Jac=0; // a copy of Index rows of Jacobian of all transformed equality constr.(dim nnz_equal_Jac)
  static int *Jc_equal_Jac=0; // a copy of columns structure of Jac. (dim N_EQUAL+1)
  static int *Ir_inequal_Jac=0;  // the same for MODE==2, structure of Jac. of inequalities
  static int *Jc_inequal_Jac=0;
  static real *L_mlt=0; // Lagrangian multipliers of (untransformed) constr. (useful for MODE==2/3)
  static real *X=0; // copy of x set by user when calling func value or gradients (for MODE==3, Hes)
  static int *Jc_Hes_ALagr=0;  // 'start column structure' for hessian of augmented lagr (MODE 3)
  static int *Ir_Hes_ALagr=0;  // row indices for ^^^
  static int *lookuptableH=0;  // look up table for Hessian of Lagrangian (sphes()->H)
  static int **lookuptableG=0;  // look up tables for dyadic products of grad of constr (g_i*g_i->H)
  static int lookupH_dim;      // dimensions of lookuptableH and lookuptableG, respectively
  static int *lookupG_dim=0;
  static int *bequal_m2=0;     // 'fake' BEQUALITY[] for MODE 2 with flags as generated in Pennlp/MODE 3

  static int my_obj_no=0;   // no of objective function to consider, -1 ~ no objective function at all, use constat 0
  static real *ow=0;     // array of dim n_obj (total number of objective functions in .nl file), multipliers for
    // objective functions while hessian is being assambled

  static int hes_alagr_warning=0;  // warning about being lazy and not adding dyadic gradients to hessian of aug. lagr.

  char buf[512];
  FILE *nl;
  Jmp_buf err_jmp0;
  char **whatp;
  fint nerror;

  int my_mode, my_dense;
  int i,j,k,itmp;
  int pos,eqpos;
  int *Ir, *Jc;
  double *tmp, f_val, dcoef, *t, *HAL;
  real *xx, *v;
  cgrad *cg, **cgp, **cgpe;

  asl = cur_ASL;

  /* Initialization: [...]=fampl(name,mode[,density_flag]) */
  if ((nrhs==2 || nrhs==3) && mxIsChar(prhs[0])) {

    /* find out which mode is used */
    my_mode = scalarchk(prhs[1]);
    my_dense = nrhs==3 ? scalarchk(prhs[2]) : DEFAULT_DENSE;
    if (my_mode!=1 && my_mode!=2 && my_mode!=3 && my_mode!=4 || my_dense!=0 && my_dense!=1)
      usage("mode or dense flag doesn't match");  //error
    MODE=my_mode;
    DENSE=my_dense;
    /* Store it in global variables */
    adjust_global_scalar("DENSE", DENSE, 1);
    adjust_global_scalar("MODE", MODE, 1);
    /* now, right hand side is OK and mode/dense flags are set */
    if (MODE==1 && nlhs!=6 && nlhs!=7 || (MODE==2 || MODE==3 || MODE==4) && nlhs!=2 && nlhs!=0)
      usage("init, rhs and lhs doesn't match");  //error
    if (mxGetString(prhs[0], buf, sizeof(buf)))
      usage("Expected 'stub' as the first argument\n");
    at_end();
    mexAtExit(at_end);
    asl = ASL_alloc(ASL_read_pfgh);
    return_nofile = 1;
    if (!(nl = jac0dim(buf,strlen(buf)))) {
      sprintf(msgbuf, "Can't open %.*s\n",sizeof(msgbuf)-20, buf);
      mexErrMsgTxt(msgbuf);
    }
    // note: n_obj ... total number of objective functions in nl-file, in fact n_obj ~ asl->i.n_obj_
    if (n_obj <= 0) {
      mexPrintf("Warning: no objective function (objective == 0)\n");
      OBJSIGN=0;
      ow=0;
      my_obj_no=-1;
    } else {
      my_obj_no=0;
      ow = (real *) my_pmalloc(n_obj*sizeof(real));
      memset(ow,0,n_obj*sizeof(real));
    }
    if (n_obj > 1)
      mexPrintf("Warning: %i objective functions, using the first one\n",n_obj);
    N = n_var;  // number of (all) variables
    nc = n_con; // number of (untransformed) constraints
    set_global_scalar("N",N);

    /* ASL objects */    
    X0 = (real *) M1alloc(N*sizeof(real));
    LUv = (real *) M1alloc(N*sizeof(real));
    Uvx = (real *) M1alloc(N*sizeof(real));
    if (nc) {
      pi0 = (real *) M1alloc(nc*sizeof(real));
      LUrhs = (real *) M1alloc(nc*sizeof(real));
      Urhsx = (real *) M1alloc(nc*sizeof(real));
    } else {
      pi0=0;
      LUrhs=0;
      Urhsx=0;
    }

    /* numbers of nonzeroes/sizes in Jacobian/Hessian matrices */
    nnz_whole_Jac = nzc;   // number of nnzs in Jacobian matrix of all untransformed constr.
    nnz_whole_Hes = 0;  // need to compute
    nnz_equal_Jac = -1; 
    nnz_inequal_Jac = -1; 
    nnz_Hes_Lagr = 0;
    nnz_Hes_ALagr = 0;

    /* local static */
    constr_val = 0;
    if (nc > 0)
      constr_val = (real *) my_pmalloc(nc*sizeof(real));
    f_grad = (real *) my_pmalloc(N*sizeof(real));
    X = (real *) my_pmalloc(N*sizeof(real));
    L_mlt=0;
    whole_J=0;
    whole_Hsp=0;
    U=0;
    PENALTY=0;
    BEQUALITY=0;
    bequal_m2=0;
    NPERM=0;
    constraints_x=0;
    gi_grd=0;
    gi_gind=0;
    Ir_equal_Jac=0;
    Jc_equal_Jac=0;
    Ir_inequal_Jac=0;
    Jc_inequal_Jac=0;
    Ir_Hes_ALagr=0;
    Jc_Hes_ALagr=0;
    lookuptableH=0;
    lookuptableG=0;
    lookupH_dim=0;
    lookupG_dim=0;
    hes_alagr_warning=0;

    if (MODE==1 && nc) {   // amplfunc/spamfunc mode (original interface of Ampl)
      if (nlhs == 7)
        cvar = (int*)M1alloc(nc*sizeof(int));
      else if (n_cc)
        mexPrintf(ignore_complementarity, n_cc);
    }

    pfgh_read(nl, ASL_findgroups);

    if (MODE==1) {
      r2d_copy(mxGetPr(plhs[0] = mxCreateDoubleMatrix(N, 1, mxREAL)),X0,N);
      r2d_copy(mxGetPr(plhs[1] = mxCreateDoubleMatrix(N, 1, mxREAL)),LUv,N);
      r2d_copy(mxGetPr(plhs[2] = mxCreateDoubleMatrix(N, 1, mxREAL)),Uvx,N);
      r2d_copy(mxGetPr(plhs[3] = mxCreateDoubleMatrix(nc, 1, mxREAL)),pi0,nc);
      r2d_copy(mxGetPr(plhs[4] = mxCreateDoubleMatrix(nc, 1, mxREAL)),LUrhs,nc);
      r2d_copy(mxGetPr(plhs[5] = mxCreateDoubleMatrix(nc, 1, mxREAL)),Urhsx,nc);

      if (nlhs == 7) {
        plhs[6] = mxCreateDoubleMatrix(nc, 1, mxREAL);
        tmp = mxGetPr(plhs[6]);
        for(i = 0; i < nc; i++)
          tmp[i] = cvar[i];
      }
      if (nnz_whole_Jac)
        whole_J = (real *) my_pmalloc(nnz_whole_Jac*sizeof(real));
      if (!DENSE) {
        nnz_whole_Hes = sphsetup(my_obj_no, 0, nc > 0, 0);
	if (nnz_whole_Hes)
          whole_Hsp = (real *)my_pmalloc(nnz_whole_Hes*sizeof(real));
      }
    }  else if (MODE==2 || MODE==3 || MODE==4) {
      N_BOUNDS=count_constr(LUv,Uvx,N,&N_EQUAL);
      if (N_EQUAL)
	mexPrintf("WARNING: Equality constraints for some variables x_i, treated inefficiently!\nHigh probability of instability!! Do NOT use that!\n");
      N_LNLCONSTR=count_constr(LUrhs,Urhsx,nc,&N_EQUAL);
      N_CONSTR=N_BOUNDS+N_LNLCONSTR;
      N_INEQUAL=N_CONSTR-N_EQUAL;
      set_global_scalar("N_CONSTR",N_CONSTR);
      if (!N_EQUAL)
	mexPrintf("INFO: No equality constraints.\n");
      set_global_scalar("N_EQUAL",N_EQUAL);
      if (MODE==2 || MODE==3) {
        set_global_scalar("N_BOUNDS",N_BOUNDS);
        set_global_scalar("N_INEQUAL",N_INEQUAL);
        set_global_scalar("N_LNLCONSTR",N_LNLCONSTR);
      }

      if (N_CONSTR) {
        NPERM=(int *) my_pmalloc(N_CONSTR*sizeof(int));
        BEQUALITY=(int *) my_pmalloc(N_CONSTR*sizeof(int));
        if (MODE==3 || MODE==4) {
          set_constr(LUv, Uvx, N, BEQUALITY, NPERM);
          set_constr(LUrhs, Urhsx, nc, BEQUALITY+N_BOUNDS, NPERM+N_BOUNDS);
        } else {   // MODE==2
          // original ('fake') order of equality constraints
          bequal_m2=(int *) my_pmalloc(N_CONSTR*sizeof(int));
          set_constr(LUv, Uvx, N, bequal_m2, NPERM);
          set_constr(LUrhs, Urhsx, nc, bequal_m2+N_BOUNDS, NPERM+N_BOUNDS);
          set_global_iarr("BEQUAL_M2",bequal_m2,N_CONSTR);
          // real (~used in MODE 2) order of equalitites
          pos=0;
          eqpos=N_INEQUAL;
          set_constr_eq(LUv, Uvx, N, BEQUALITY, NPERM, &pos, &eqpos);
            // now should be eqpos=N_INEQUAL, pos=N_BOUNDS
          set_constr_eq(LUrhs, Urhsx, nc, BEQUALITY, NPERM, &pos, &eqpos);
            // now eqpos=N_CONSTR, pos=N_INEQUAL
        }

        if (MODE==3) {
          U = (double *) my_pmalloc(N_CONSTR*sizeof(double));
          PENALTY = (double *) my_pmalloc(N_CONSTR*sizeof(double));
        }
        if (MODE==2 || MODE==3) {
          constraints_x = (real *) my_pmalloc(N_CONSTR*sizeof(real));
          memset(constraints_x,0,N_CONSTR*sizeof(real)); 
            // memset just for case that somone calls hessian at first
          Jc_equal_Jac = (int *) my_pmalloc((N_EQUAL+1)*sizeof(int));
          if (MODE==2)
            Jc_inequal_Jac = (int *) my_pmalloc((N_INEQUAL+1)*sizeof(int));
        }
      }

      if (n_obj>0) {
        OBJSIGN = !objtype[my_obj_no] ? 1 : -1;  //objtype==0 (min) --> objsign=1, objtype==1 (max)--> -1
	ow[my_obj_no]=OBJSIGN;
      }
      set_global_scalar("OBJSIGN",OBJSIGN);

      asl->i.congrd_mode = 1;
      gi_grd = (real *) my_pmalloc(N*sizeof(real));
      gi_gind = (int *) my_pmalloc(N*sizeof(int));
      if (nc > 0)
        L_mlt = (real *) my_pmalloc(nc*sizeof(real));

      if (nlhs==2) {
        r2d_copy(mxGetPr(plhs[0] = mxCreateDoubleMatrix(N, 1, mxREAL)),X0,N);
        r2d_copy(mxGetPr(plhs[1] = mxCreateDoubleMatrix(nc, 1, mxREAL)),pi0,nc);
      }

      if (!DENSE) {
        //nnz_Hes_Lagr = sphsetup(my_obj_no, 0, nc > 0, 0);   // ignore OBJSIGN
        //nnz_Hes_Lagr = sphsetup(my_obj_no, ow!=0, nc > 0, 0);  // ?BUG?
        nnz_Hes_Lagr = sphsetup(-1, ow!=0, nc > 0, 0);  // OK
        if (MODE==3 && nnz_Hes_Lagr)
          whole_Hsp = (real *)my_pmalloc(nnz_Hes_Lagr*sizeof(real));
      }

    }
    return;
  }  // end of initialization

  if (!asl)
    usage("No 'stub' file associated, amplfunc(\"stub\") has not been called\n");

  /* error handling */
  nerror = -1;   // if error occurs use jump instead of an error message 
  err_jmp1 = &err_jmp0;
  what = "(?)";
  whatp = &what;
  if (setjmp(err_jmp0.jb)) {
    sprintf(msgbuf, "Trouble evaluating %s\n", *whatp);
    mexErrMsgTxt(msgbuf);
  }
  /* Synchronise global flags */
  adjust_global_scalar("DENSE", DENSE, 1);
  adjust_global_scalar("MODE", MODE, 1);
  // CHECK DENSE/MODE FLAGS ??? No, must be all right

  if (MODE==3) 
    read_all_globals();   // read U and PENALTY

  /* Function values or gradient computation */
  if (nrhs == 2) {
    if ((MODE==3 || MODE==4) && nlhs != 2 || MODE==2 && nlhs!=3)
      usage("func or grad, left hand side");
    
    d2r_copy(X,sizechk(prhs[0],"x",N),N);
    t = sizechk(prhs[1],"0 or 1", 1);

    // update values of all (transformed) constraints 
    if (constr_val) {
      what = "c";
      conval(X, constr_val, &nerror);
    }
    if (MODE==2 || MODE==3) {
      for (i=0; i<N_CONSTR; i++)
	constraints_x[i]=gi_val(i,X,constr_val);
    }

    /* Function values */
    if (*t == 0.) {
      what = "f";
      f_val = my_obj_no>=0 ? objval(my_obj_no, X, &nerror) : 0.;

      if (MODE==1) {
        plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
        plhs[1] = mxCreateDoubleMatrix(nc, 1, mxREAL);
        *mxGetPr(plhs[0]) = f_val;
        r2d_copy(mxGetPr(plhs[1]),constr_val,nc);
      } else if (MODE==2) {
        plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
        plhs[1] = mxCreateDoubleMatrix(N_INEQUAL, 1, mxREAL);
        plhs[2] = mxCreateDoubleMatrix(N_EQUAL, 1, mxREAL);
        *mxGetPr(plhs[0]) = OBJSIGN*f_val;
	if (N_INEQUAL)
	  memcpy(mxGetPr(plhs[1]),constraints_x,N_INEQUAL*sizeof(double));
	if (N_EQUAL)
	  memcpy(mxGetPr(plhs[2]),constraints_x+N_INEQUAL,N_EQUAL*sizeof(double));
      } else if (MODE==4) {
        plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
        plhs[1] = mxCreateDoubleMatrix(N_EQUAL, 1, mxREAL);
        *mxGetPr(plhs[0]) = OBJSIGN*f_val;
	if (N_EQUAL) {
	  for (i=0, tmp=mxGetPr(plhs[1]);i<N_CONSTR;i++)
	    if (BEQUALITY[i])
	      *tmp++=gi_val(i,X,constr_val);
	}
      } else if (MODE==3) {
        plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
        plhs[1] = mxCreateDoubleMatrix(N_EQUAL, 1, mxREAL);
        *mxGetPr(plhs[0]) = penfunc_val(OBJSIGN*f_val,constraints_x);
	if (N_EQUAL) {
	  for (i=0, tmp=mxGetPr(plhs[1]);i<N_CONSTR;i++)
	    if (BEQUALITY[i])
	      *tmp++=constraints_x[i];
	}
      } 
      
      return;   // end of function value computation

    } else if (*t == 1.) {  /* gradient */
      what = "g";
      if (my_obj_no>=0)
        objgrd(my_obj_no, X, f_grad, &nerror);
      else
	memset(f_grad,0,N*sizeof(real));
      // swap sign if necessary
      if (OBJSIGN==-1 && (MODE==2 || MODE==3 || MODE==4)) {
        for (i=0;i<N;i++)
	  f_grad[i]*=-1;
      }

      if (MODE==1) {
	if (whole_J) {
          what = "J";
          jacval(X, whole_J, &nerror);
	}

        plhs[0] = mxCreateDoubleMatrix(N, 1, mxREAL);
        r2d_copy(mxGetPr(plhs[0]),f_grad,N);
	if (DENSE) {
          tmp = mxGetPr(plhs[1] = mxCreateDoubleMatrix(nc, N, mxREAL));
          if (nc) {
            memset(tmp, 0, nc*N*sizeof(double));
            cgp = Cgrad;
            for(cgpe = cgp + nc; cgp < cgpe; tmp++)
              for(cg = *cgp++; cg; cg = cg->next)
                tmp[nc*cg->varno] = whole_J[cg->goff];
	  }
        } else {  // sparse
	  plhs[1] = mxCreateSparse(nc, N, nnz_whole_Jac, mxREAL);
          if (nc) {
            r2d_copy(mxGetPr(plhs[1]),whole_J,nnz_whole_Jac);
            Ir = mxGetIr(plhs[1]);
            memcpy(mxGetJc(plhs[1]), A_colstarts, (N+1)*sizeof(int));
            cgp = Cgrad;
            for(i = 0; i < nc; i++)
              for(cg = *cgp++; cg; cg = cg->next)
                Ir[cg->goff] = i;
	  }
	}
      } else if (MODE==2 || MODE==3 || MODE==4) {
        plhs[0] = mxCreateDoubleMatrix(N, 1, mxREAL);
        if (MODE==2 || MODE==4) {
          r2d_copy(mxGetPr(plhs[0]),f_grad,N);
        } else { // MODE==3
	  // gradient of augmented lagrangian
	  penfunc_grad(mxGetPr(plhs[0]),X,constraints_x,f_grad,gi_grd,gi_gind);
	}
	if (MODE==2) {
          if (DENSE) {
            plhs[1] = mxCreateDoubleMatrix(N, N_INEQUAL, mxREAL);
            plhs[2] = mxCreateDoubleMatrix(N, N_EQUAL, mxREAL);
	    djac_values(mxGetPr(plhs[1]),1,0,N_INEQUAL,X,gi_grd);
	    djac_values(mxGetPr(plhs[2]),1,N_INEQUAL,N_CONSTR,X,gi_grd);
          } else {   // sparse
            if (N_INEQUAL) {
              if (nnz_inequal_Jac==-1) {  // compute structure first!
                nnz_inequal_Jac=jac_struc_jc(Jc_inequal_Jac,1,0,N_INEQUAL,X,gi_grd,gi_gind);
                Ir_inequal_Jac = (int *) my_pmalloc(nnz_inequal_Jac*sizeof(int));
                plhs[1] = mxCreateSparse(N, N_INEQUAL, nnz_inequal_Jac, mxREAL);
                jac_struc_ir(Jc_inequal_Jac,Ir_inequal_Jac,mxGetPr(plhs[1]),1,0,N_INEQUAL,X,gi_grd,gi_gind);
              } else {
                plhs[1] = mxCreateSparse(N, N_INEQUAL, nnz_inequal_Jac, mxREAL);
                jac_values(Jc_inequal_Jac,mxGetPr(plhs[1]),1,0,N_INEQUAL,X,gi_grd,gi_gind);
	      }
              memcpy(mxGetJc(plhs[1]),Jc_inequal_Jac,(N_INEQUAL+1)*sizeof(int));
              memcpy(mxGetIr(plhs[1]),Ir_inequal_Jac,nnz_inequal_Jac*sizeof(int));
	    } else {  // no inequalities
              plhs[1] = mxCreateSparse(N, 0, 0, mxREAL);
              *mxGetJc(plhs[1])=0;
            }
	    if (N_EQUAL) {
              if (nnz_equal_Jac==-1) {  // compute structure first!
                nnz_equal_Jac=jac_struc_jc(Jc_equal_Jac,1,N_INEQUAL,N_CONSTR,X,gi_grd,gi_gind);
                Ir_equal_Jac = (int *) my_pmalloc(nnz_equal_Jac*sizeof(int));
                plhs[2] = mxCreateSparse(N, N_EQUAL, nnz_equal_Jac, mxREAL);
                jac_struc_ir(Jc_equal_Jac,Ir_equal_Jac,mxGetPr(plhs[2]),1,N_INEQUAL,N_CONSTR,X,gi_grd,gi_gind);
              } else {
                plhs[2] = mxCreateSparse(N, N_EQUAL, nnz_equal_Jac, mxREAL);
                jac_values(Jc_equal_Jac,mxGetPr(plhs[2]),1,N_INEQUAL,N_CONSTR,X,gi_grd,gi_gind);
              }
              memcpy(mxGetJc(plhs[2]),Jc_equal_Jac,(N_EQUAL+1)*sizeof(int));
              memcpy(mxGetIr(plhs[2]),Ir_equal_Jac,nnz_equal_Jac*sizeof(int));
	    } else {  // no equalities
              plhs[2] = mxCreateSparse(N, 0, 0, mxREAL);
              *mxGetJc(plhs[2])=0;
            }
	  }
	} else {  // end of MODE 2  --> MODE 3/4
          if (DENSE) {
            plhs[1] = mxCreateDoubleMatrix(N, N_EQUAL, mxREAL);
	    djac_values(mxGetPr(plhs[1]),0,0,N_CONSTR,X,gi_grd);
          } else {  // sparse
            if (N_EQUAL) {
              if (nnz_equal_Jac==-1) {
                // computation for the first time --> set structure &
                // count nonzero elements in Jacobian for equal. constraints
                nnz_equal_Jac=jac_struc_jc(Jc_equal_Jac,0,0,N_CONSTR,X,gi_grd,gi_gind);
                Ir_equal_Jac = (int *) my_pmalloc(nnz_equal_Jac*sizeof(int));
                plhs[1] = mxCreateSparse(N, N_EQUAL, nnz_equal_Jac, mxREAL);
                jac_struc_ir(Jc_equal_Jac,Ir_equal_Jac,mxGetPr(plhs[1]),0,0,N_CONSTR,X,gi_grd,gi_gind);
              } else {  // structure of Jacobian of equal. constr. defined
                plhs[1] = mxCreateSparse(N, N_EQUAL, nnz_equal_Jac, mxREAL);
                jac_values(Jc_equal_Jac,mxGetPr(plhs[1]),0,0,N_CONSTR,X,gi_grd,gi_gind);
              }
	      memcpy(mxGetJc(plhs[1]),Jc_equal_Jac,(N_EQUAL+1)*sizeof(int));
	      memcpy(mxGetIr(plhs[1]),Ir_equal_Jac,nnz_equal_Jac*sizeof(int));
	    } else { // no equality constraints
              plhs[1] = mxCreateSparse(N, 0, 0, mxREAL);
              *mxGetJc(plhs[1])=0;
            }
          } 
        }  // end of MODE 2 vs. 3/4
      }  // end of MODE 1 vs 2/3/4
      return;  // end of gradient computation
    } else {  // neither function values nor gradient
      // error
      usage("should have been function or gradient values computations, but wrong number");
    }
  }  // end of if (nrhs==2), e.i., func/grad computation

  /* Write stub.sol file */
  if (nlhs == 0 && (nrhs == 3 || nrhs == 4)) {
    /* eval2('solution message', x, v): x = primal, v = dual */
    /* optional 4th arg = solve_result_num */
    if (!mxIsChar(prhs[0]))
      usage("write sol error");
    xx = sizechk(prhs[1],"x",N);
    v = sizechk(prhs[2],"v",nc);
    if (mxGetString(prhs[0], buf, sizeof(buf)))
      mexErrMsgTxt("Expected 'solution message' as first argument\n");
    solve_result_num = nrhs == 3 ? -1 /* unknown */
			: (int)*sizechk(prhs[3],"solve_result_num",1);
    write_sol(buf, xx, v, 0);
    return;
  }

  /* Compute a Hessian matrix of Lagrangian */
  if (nlhs == 1 && (nrhs == 1 || MODE==3 && nrhs==0)) {
    if (MODE==1 && nc)
      v = sizechk(prhs[0],"v",nc);
    else if (MODE==2 && N_CONSTR) {  // MODE==2: v could be of dimension N_LNLCONSTR or N_CONSTR
      if (MXISOK(prhs[0],N_CONSTR,1))
	v = mxGetPr(prhs[0]) + N_BOUNDS;  
        // dimension of v is N_CONSTR -> use only last N_LNLCONSTR multipliers
      else
	v = sizechk(prhs[0],"v",N_LNLCONSTR);
    } else if ((MODE==3 || MODE==4) && nrhs==1 && N_EQUAL)
      v = sizechk(prhs[0],"v",N_EQUAL);
    else   //MODE==3 && no v as input
      v=0;

    if (MODE==1) {
      if (DENSE) {
        plhs[0] = mxCreateDoubleMatrix(N, N, mxREAL);

        what = "W";
        fullhes(mxGetPr(plhs[0]), N, my_obj_no, 0, v);
	// JF: ^^^ double and real are mixed together, well, now I don't care ... it's the same
	// Note: Hessian is symmetric, so do not worry about how it is stored (column/row-wise)
      } else { // sparse
        plhs[0] = mxCreateSparse(N, N, nnz_whole_Hes, mxREAL);

        what = "W";
        sphes(whole_Hsp, my_obj_no, 0, v);

	itmp = sputinfo->hcolstarts[N]; // nnz in whole_Hsp, or nnz_whole_Hes
	fi2i_copy(mxGetJc(plhs[0]),sputinfo->hcolstarts,N+1);
	fi2i_copy(mxGetIr(plhs[0]),sputinfo->hrownos, itmp);
	r2d_copy(mxGetPr(plhs[0]),whole_Hsp, itmp);
      }
    } else if (MODE==2) {
      if (nc) {
        // fill in multipliers based on v[] and NPERM[], boxed constraints are ignored in hess.
        memset(L_mlt,0,nc*sizeof(real));
        for (i=N_BOUNDS; i<N_CONSTR; i++) {
          itmp=1;
          if ((j=NPERM[i])<0) {
            j=-(j+1);
            itmp=-1;
          }
          L_mlt[j] += itmp*v[i-N_BOUNDS];
        }
      }
      // compute lagrangian by Ampl function
      if (DENSE) {
        plhs[0] = mxCreateDoubleMatrix(N, N, mxREAL);
        what = "W";
        //fullhes(mxGetPr(plhs[0]), N, my_obj_no, ow, L_mlt);
        fullhes(mxGetPr(plhs[0]), N, -1 /*my_obj_no*/, ow, L_mlt);
        //fullhes(mxGetPr(plhs[0]), N, 0, 0, L_mlt);
      } else {   // sparse
	plhs[0] = mxCreateSparse(N, N, nnz_Hes_Lagr, mxREAL);
        what = "W";
        //sphes(mxGetPr(plhs[0]), my_obj_no, 0, L_mlt);  // ignore OBJSIGN
        //sphes(mxGetPr(plhs[0]), my_obj_no, ow, L_mlt); // sometimes doesn't work - BUG?
        sphes(mxGetPr(plhs[0]), -1, ow, L_mlt);   // should be OK

	fi2i_copy(mxGetJc(plhs[0]),sputinfo->hcolstarts,N+1);
	fi2i_copy(mxGetIr(plhs[0]),sputinfo->hrownos, nnz_Hes_Lagr);
      }
    } else if (MODE==3 || MODE==4) {
      if (nc) {
        // fill in Lagrangian multiplicators of untransformed constraints
        memset(L_mlt,0,nc*sizeof(real));
        if (v) {
          for (j=0,i=0; i<N_CONSTR; i++)
            if (BEQUALITY[i]) {
              L_mlt[NPERM[i]]=v[j];
              j++;
            }
        } else {  // MODE==3 without setting v[] by user --> use current
          for (i=0; i<N_CONSTR; i++)
            if (BEQUALITY[i])
              L_mlt[NPERM[i]] = U[i];
        }
        if (MODE==3) {  // fill in L.mlt of untransformed inequality constraints
          for (i=N_BOUNDS; i<N_CONSTR; i++) {  //!!!!!!!!! No BOX constraints !!!!!!!!!
            if (BEQUALITY[i])
              continue;
            itmp=1;
            if ((j=NPERM[i])<0) {
              j=-(j+1);
              itmp=-1;
            }
            L_mlt[j] += itmp*U[i]*D_phi2(constraints_x[i]/PENALTY[i]);
          }
        }
      }
      // compute hessian of lagrangian by AMPL
      if (DENSE) {
        plhs[0] = mxCreateDoubleMatrix(N, N, mxREAL);
	tmp=mxGetPr(plhs[0]);
        what = "W";
        //fullhes(tmp, N, my_obj_no, ow, L_mlt);
        fullhes(tmp, N, -1, ow, L_mlt);
	
        // add u_i/p_i*phi''(g_i(x)/p_i) * (grad g_i(x)) * (grad g_i(x))^T  (dyadic mlt.)
        if (MODE==3) {
          for (i=0; i<N_CONSTR; i++) {
            if (BEQUALITY[i])
	      continue;
	    dcoef=U[i]/PENALTY[i]*D2_phi2(constraints_x[i]/PENALTY[i]);
            //JF: ^^ hope, that constraints_x[] is filled, e.i., function values call was done before
	    itmp = gi_grad(i,X,gi_grd,gi_gind);
	    for (j=0; j<itmp; j++)
	      for (k=0; k<itmp; k++)
	        tmp[gi_gind[j] + gi_gind[k]*N] += dcoef*gi_grd[j]*gi_grd[k];
	  }
        }
      } else {  // sparse
        if (MODE==4 || MODE==3) {
          if (MODE==3 && !hes_alagr_warning) {
	    mexPrintf("WARNING: dydadic multiplication of constraints gradients not added yet!!!\nDo it in Matlab!\n");
	    hes_alagr_warning=1;
	  }
	  plhs[0] = mxCreateSparse(N, N, nnz_Hes_Lagr, mxREAL);
          what = "W";
          //sphes(mxGetPr(plhs[0]), my_obj_no, ow, L_mlt);
          sphes(mxGetPr(plhs[0]), -1, ow, L_mlt);

	  fi2i_copy(mxGetJc(plhs[0]),sputinfo->hcolstarts,N+1);
	  fi2i_copy(mxGetIr(plhs[0]),sputinfo->hrownos, nnz_Hes_Lagr);
	} 
        #if 0
	else {  // sparse && MODE==3
	  if (nnz_Hes_ALagr) {
	    plhs[0] = mxCreateSparse(N,N,nnz_Hes_ALagr,mxREAL);
	    HAL = mxGetPr(plhs[0]);
	    memset(HAL,0,nnz_Hes_ALagr*sizeof(double));
	    memcpy(mxGetJc(plhs[0]),Jc_Hes_ALagr,(N+1)*sizeof(int));
	    memcpy(mxGetIr(plhs[0]),Ir_Hes_ALagr, nnz_Hes_ALagr*sizeof(int));

            what = "W";
            sphes(whole_Hsp, my_obj_no, ow, L_mlt);
	    projectH(HAL,whole_Hsp,dcoef,nnz_Hes_Lagr,lookuptableH);

            for (i=0,j=0; i<N_CONSTR; i++) {
              if (BEQUALITY[i])
                continue;
	      dcoef=U[i]/PENALTY[i]*D2_phi2(constraints_x[i]/PENALTY[i]);
	      itmp = gi_grad(i,X,gi_grd,gi_gind);
	      projectG(HAL,itmp,gi_grd,dcoef,lookuptableG[j]);
	      j++;
            }
	  } else {  // structure not known yet :'-(
            mexPrintf("Sorry, too lazy to finish that :-(\nUse dense structure instead of it\n");

	  }  // end of MODE 3 && sparse && unknown structure
	}
        #endif
      }  // end of sparse/dense
    }  // end of MODE 4/3
    return;
  } else {  // not Hessian??
    usage("strange input/output, do not know what to do");
  }

}  // of main()


