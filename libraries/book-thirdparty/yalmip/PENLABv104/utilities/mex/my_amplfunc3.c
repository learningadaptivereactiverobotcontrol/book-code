/* AMPL Interface to Matlab as a mex file
 * 
 * Third version, my ampl-matlab interface
 * version: 0.52
 * last update: 27/01/09
 *
 * Note: As an inspiration amplfunc.c & spamfunc.c included in Ampl Solver
 * library were used and small bits of codes still remain.
 * Their original licence is included below.
 * However, the interface was adjusted to be more convinient and/or similar
 * to Pennlp/optimization code and PennonM.
 * This version has just one mode to operate known as MODE 2 in the second
 * version of the interface (my_amplfunc2.c).
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
 * get rid of all other modes except MODE 2
 * transformed v_guess (initial guess of L. multipliers) that it fits to
 *   transformed constraints and their writing as the solution as well
 * the interface doesn't use global variables to report N, N_CONSTR, ... anymore
 *   a new structure is returned while initialization instead of 
 * updated usage() and comments so that they fit to the current version
 * partly fixed the problem with crashing Matlab after 'clear all' command (because of 
 *   incorrectly opened error console), FILE *Stderr declared in asl.h is opened as a
 *   local file if possible
 *
 *
 * TO DO List:
 * -----------
 * expand it for matrix variables and SDP
 * [write proper documentation]
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
 *  THE INTERFACE (formerly known aka MODE 2)
 *  Every problem is transformed to
 *    min   f(x) 
 *    s.t.  g_i(x)<=0 
 *          h_i(x)=0
 *  even box constraints are included as g_i(x), equalities in
 *  box constraints are strictly forbidden (the problem could
 *  have been simplified), warning message is given but the behaviour
 *  of the program is uncertain (unstable)!
 *
 *  USAGE
 *    INITIALIZATION
 *    [[x_guess,v_guess,]output_struct]=amplf('name.nl'[,dense_flag])
 *        Initiates the problem, allocates sufficient memory,
 *        must be called prior to any other computation otherwise
 *        an error message is returned.
 *        First input parameter is the name of the problem (stored in a nl-file),
 *        second one is optional and specifies if the user wants to get returned
 *        sparse structures (dense_flag=0) or dense ones (dense_flag=1).
 *        First two returned variables are optional and specify initial guess of
 *        starting point and appropriate lagrangian multipliers (dual vars),
 *        but just for non-box constraints, e.i., of dim N_CONSTR - N_BOUNDS
 *        with the order such that inequalities go first, then equalities
 *        the last return variable is compulsory and introduces the output_structure
 *        whose elements are described below. 
 *    COMPUTATION
 *    [f,g,h] = amplf(x,0)
 *        returns function values of f and all constraints
 *    [grad f, grad g, grad h] = amplf(x,1)
 *        returns gradients of f and constraints (column-wise)
 *    H=amplf(v)
 *        returns hessian H=nabla^2 f + sum v_i nabla^2 g_i + sum v_(i-..) nabla^2 h_i
 *        'v' could be of size N_CONSTR, but first N_BOUNDS elements should
 *        correspond to box constraints and therefore they are ignored in hessian,
 *        or it could have dimension N_LNLCONSTR (=N_CONSTR-N_BOUNDS) and then it is
 *        expected to correspond to lagrangian multipliers of all 'true' constraints
 *        The hessian is computed at the last point where function values/gradients
 *        were called. (At least one of the calls amplf(x,0/1) must prior to hessian call.
 *    SAVING RESULTS
 *    amplf('solution message',x,v[,save_result_num])
 *        writes the results to stub.sol (solution) file.
 *
 *  Important variables 'uploaded' to Matlab via the output structure:
 *    N ...... number of variables
 *    N_BOUNDS ... number of box constraints as tranformed as 
 *       inequalities g_i(x)
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
 *
 *  Other important local/global variables in the interface:
 *    NPERM ...... (dim N_CONSTR) permutation of constraints how they
 *        are transformed
 *      first N_BOUNDS (0..N_BOUNDS-1) are transformed boxed constraints,
 *        i or -(i+1) aims to real number of variable constraint
 *      N_BOUNDS..N_CONSTR-1 are the rest inequality & equality constraints
 *        i or -(i+1) aims to the rest of linear/nonlinear constraints
 *      numbers i-th constraints are 0-based
 *    N_INEQUAL ... total number of inequalities g_i(x) (inc. box!!!)
 *    N_LNLCONSTR . total number og eq. & ineq. constraints (without box), 
 *        e.i., N_LNLCONSTR = N_CONSTR-N_BOUNDS = N_EQUAL+N_INEQUAL-N_BOUNDS
 *        not necessary, just for compatibility
 *    DENSE .. density mode (1..dense problem, 0..sparse problem)
 *    MODE ... current mode, e.i., 2 ... cancelled
 *
 *  Note: Any errors messages are forwarded to a local file asl_error.log
 *    if it was possible to open it for writing.
 *
 */

/* 
 * How is the right 'operational mode' of the interface recognized?
 *   nrhs=1/2, nlhs=1/3, nrhs[0]=string   --> initialization
 *   nrhs=2, nlhs=3                       --> values/gradients
 *   nrhs=1, nlhs=1                       --> hessian
 *   nrhs=3/4, nlhs=0, nrhs[0]=string     --> write solution to file
 */

#include "mex.h"
#undef printf
#include "asl_pfgh.h"

#ifdef _WIN32
/* Omit sw "signal" catching and x86 precision adjustment. */
#define ASL_NO_FP_INIT
#include "fpinit.c"
#endif /* _WIN32 */

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
  //mexPrintf("Amplf(): Persistant structures freed: %i\n",i);
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
  -----> PROBLEM SOLVED <------ 
  NOTE: Use M1alloc() for allocation of parts which are included in ASL stucture,
    for the other allocations use my_pmalloc (if it should be permanent/persistent)*/
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

static char mystderr_opened=0;
static char mystderr_name[]="asl_error.log";

static char msgbuf[256];
static char *what; // for error jump while computing gradient/hessian/...

static int DENSE=0;
//static int MODE=0;

/* DENSE ... 0..treat problems as sparse, 1..dense
 * MODE .... not currently used
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

/* Creates and returns the output structure (first or third return parameter
   when initializating. The structure includes fields:
   N, N_CONSTR, N_BOUNDS, N_EQUAL, BEQUAL_M2
   (meaning is the same as their global/local equivalents)
  if error occurs, returns zero */
static mxArray *set_output_struct(int *bequal_m2) {
  mxArray *strct;
  mxArray *field;
  const char *field_names[]={"N", "N_CONSTR", "N_BOUNDS", "N_EQUAL", "BEQUAL_M2"};
  double *dtmp;
  int i;

  if (N_CONSTR && !bequal_m2)
    return 0;
  if ((strct=mxCreateStructMatrix(1/*m*/,1/*n*/,5/*fields*/,field_names))==0)
    return 0;  // perhaps it would stop anyway

  /* fill in the fields */
  field = mxCreateDoubleMatrix(1,1,mxREAL);
  /* on error it stops computing and the programme is terminated */
  *mxGetPr(field) = N;
  mxSetFieldByNumber(strct,0,0 /*N        */,field);

  field = mxCreateDoubleMatrix(1,1,mxREAL);
  *mxGetPr(field) = N_CONSTR;
  mxSetFieldByNumber(strct,0,1 /*N_CONSTR */,field);

  field = mxCreateDoubleMatrix(1,1,mxREAL);
  *mxGetPr(field) = N_BOUNDS;
  mxSetFieldByNumber(strct,0,2 /*N_BOUNDS */,field);

  field = mxCreateDoubleMatrix(1,1,mxREAL);
  *mxGetPr(field) = N_EQUAL;
  mxSetFieldByNumber(strct,0,3 /*N_EQUAL  */,field);

  //field = mxCreateDoubleMatrix(1,1,mxREAL);
  //*mxGetPr(field) = DENSE;
  //mxSetFieldByNumber(strct,0,4 /*DENSE    */,field);

  field = mxCreateDoubleMatrix(N_CONSTR /*bequal dim*/,1,mxREAL);
  for (dtmp=mxGetPr(field),i=0;i<N_CONSTR;i++)
    *dtmp++ = *bequal_m2++;
  mxSetFieldByNumber(strct,0,4 /*BEQUAL_M2*/,field);

  return strct;
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

  // open a local file as an error stream if possible
  if (mystderr_opened && Stderr) {
    fclose(Stderr);
    Stderr=0;
    mystderr_opened=0;
  }

#ifdef MEMORY_MATLAB
  pml_free(&MEX_PM_LIST);
#endif
  // to let gi_grad() know that it must allocate gigrd_tmp again
  gigrd_tmp=0;
}

/* Reports information about the opened problem, the usage and (optionally)
   the error message 's' */
static void usage(const char *s)
{
  // print information of the problem
  if (asl) {
    mexPrintf("Currently problem '%s' is opened as %s\n",filename,DENSE?"DENSE" : "SPARSE");
    mexPrintf("  no. of variables: %i\n",N);
    mexPrintf("  total number of constraints: %i\n",N_CONSTR);
    mexPrintf("    including box constraints: %i\n",N_BOUNDS);
    mexPrintf("          and 'true' constr.:  %i\n",N_LNLCONSTR);
    mexPrintf("    total ineq. c.:  %i\n",N_INEQUAL);
    mexPrintf("    total equal. c.: %i\n",N_EQUAL);
    mexPrintf("  \n");
  } else {
    mexPrintf("No problem assigned yet.\n");
  }
  if (s)
    mexPrintf("--- Error: %s ---\n\n",s);
  // usage
  mexErrMsgTxt("amplf interface usage:\n\n\
    [[x_guess,v_guess,]problem_struct] = amplf('stub'[,dense_flag])\n\
       to init the problem\n\
    [f,g,h] = amplf(x,0)                   or\n\
    [grad f, grad g, grad h] = amplf(x,1)  or\n\
    H = amplf(v)\n\
       to compute function values/gradients/hessian\n\
    amplf('solution message',x,v[,solve_result_num])\n\
       to write the solution to stub.sol file");
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
 * in Pennlp we also store A as N x no_constr, gradients in columns even if in C
 * matrices are in row-wise sense
 *
 */


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  static int nc;  // number of (untransformed) constraints, e.i., dim of congrad,...

  static real *constr_val=0;    // (untransformed) constraints function values
  static real *constraints_x=0; // (transformed) constraints function values
  static real *f_grad=0;    // gradient of the first objective
  static fint nnz_equal_Jac;  // nnz in sparse Jacobian of transformed equality constraints (MODE==2/3)
  static fint nnz_inequal_Jac;
  static fint nnz_Hes_Lagr;   // nnz in sparse Hessian of Lagrangian (MODE=2/3/4)
  static real *gi_grd=0;    // temporary array for one gradient of constraints and its indices of nnz
  static int *gi_gind=0;
  static int *Ir_equal_Jac=0; // a copy of Index rows of Jacobian of all transformed equality constr.(dim nnz_equal_Jac)
  static int *Jc_equal_Jac=0; // a copy of columns structure of Jac. (dim N_EQUAL+1)
  static int *Ir_inequal_Jac=0;  // the same, structure of Jac. of inequalities
  static int *Jc_inequal_Jac=0;
  static real *L_mlt=0; // Lagrangian multipliers of (untransformed) constr. (useful for MODE==2/3)
  static real *X=0; // copy of x set by user when calling func value or gradients (for MODE==3, Hes)
  static int *bequal_m2=0;     // 'fake' BEQUALITY[] for MODE 2 with flags as generated in Pennlp/MODE 3

  static int my_obj_no=0;   // no of objective function to consider, -1 ~ no objective function at all, use constat 0
  static real *ow=0;     // array of dim n_obj (total number of objective functions in .nl file), multipliers for objective functions while hessian is being assambled

  char buf[512];
  FILE *nl;
  Jmp_buf err_jmp0;
  char **whatp;
  fint nerror;

  int my_mode, my_dense;
  int i,j,itmp;
  int pos,eqpos;
  double f_val, *t;
  real *xx, *v;
  double *utmp;

  asl = cur_ASL;

  /* Initialization: [...,problem_struct]=amplf(name[,density_flag]) */
  if ((nrhs==1 || nrhs==2) && mxIsChar(prhs[0]) && (nlhs==3 || nlhs==1)) {

    /* find out which mode is used */
    //my_mode = nrhs==2 ? scalarchk(prhs[1]) : 2;
    my_dense = nrhs==2 ? scalarchk(prhs[1]) : DEFAULT_DENSE;
    if (my_dense!=0 && my_dense!=1)
      usage("dense flag doesn't match");  //error
    //MODE=my_mode;
    DENSE=my_dense;
    /* now, right hand side is OK and mode/dense flags are set */
    if (mxGetString(prhs[0], buf, sizeof(buf)))
      usage("Expected 'stub' as the first argument\n");
    at_end();
    mexAtExit(at_end);
    // alloc Stderr to a file if possible
    if (!Stderr) {
      Stderr = fopen(mystderr_name,"w");
      if (Stderr) {
        mystderr_opened=1;
        mexPrintf("Stderr opened to %s.\n",mystderr_name);  /// ??? delete later
      } else {
        mexPrintf("Stderr wasn't opened to a file...\n");
      }
    }
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
    //set_global_scalar("N",N);

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
    nnz_equal_Jac = -1; 
    nnz_inequal_Jac = -1; 
    nnz_Hes_Lagr = 0;

    /* local static */
    constr_val = 0;
    if (nc > 0)
      constr_val = (real *) my_pmalloc(nc*sizeof(real));
    f_grad = (real *) my_pmalloc(N*sizeof(real));
    X = (real *) my_pmalloc(N*sizeof(real));
    L_mlt=0;
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

    pfgh_read(nl, ASL_findgroups);

      N_BOUNDS=count_constr(LUv,Uvx,N,&N_EQUAL);
      if (N_EQUAL)
	mexPrintf("WARNING: Equality constraints for some variables x_i, treated inefficiently!\nHigh probability of instability!! Do NOT use that!\n");
      N_LNLCONSTR=count_constr(LUrhs,Urhsx,nc,&N_EQUAL);
      N_CONSTR=N_BOUNDS+N_LNLCONSTR;
      N_INEQUAL=N_CONSTR-N_EQUAL;
      //set_global_scalar("N_CONSTR",N_CONSTR);
      //set_global_scalar("N_EQUAL",N_EQUAL);
      //set_global_scalar("N_BOUNDS",N_BOUNDS);
      //set_global_scalar("N_INEQUAL",N_INEQUAL);
      //set_global_scalar("N_LNLCONSTR",N_LNLCONSTR);
      //if (!N_EQUAL)
      //  mexPrintf("INFO: No equality constraints.\n");

      if (N_CONSTR) {
        NPERM=(int *) my_pmalloc(N_CONSTR*sizeof(int));
        BEQUALITY=(int *) my_pmalloc(N_CONSTR*sizeof(int));
          // original ('fake') order of equality constraints
          bequal_m2=(int *) my_pmalloc(N_CONSTR*sizeof(int));
          set_constr(LUv, Uvx, N, bequal_m2, NPERM);
          set_constr(LUrhs, Urhsx, nc, bequal_m2+N_BOUNDS, NPERM+N_BOUNDS);
          //set_global_iarr("BEQUAL_M2",bequal_m2,N_CONSTR);
          // real (~used in MODE 2) order of equalitites
          pos=0;
          eqpos=N_INEQUAL;
          set_constr_eq(LUv, Uvx, N, BEQUALITY, NPERM, &pos, &eqpos);
            // now should be eqpos=N_INEQUAL, pos=N_BOUNDS
          set_constr_eq(LUrhs, Urhsx, nc, BEQUALITY, NPERM, &pos, &eqpos);
            // now eqpos=N_CONSTR, pos=N_INEQUAL

          constraints_x = (real *) my_pmalloc(N_CONSTR*sizeof(real));
          memset(constraints_x,0,N_CONSTR*sizeof(real)); 
            // memset just for case that somone calls hessian at first
          Jc_equal_Jac = (int *) my_pmalloc((N_EQUAL+1)*sizeof(int));
          Jc_inequal_Jac = (int *) my_pmalloc((N_INEQUAL+1)*sizeof(int));
      }

      if (n_obj>0) {
        OBJSIGN = !objtype[my_obj_no] ? 1 : -1;  //objtype==0 (min) --> objsign=1, objtype==1 (max)--> -1
	ow[my_obj_no]=OBJSIGN;
      }
      //set_global_scalar("OBJSIGN",OBJSIGN);

      asl->i.congrd_mode = 1;
      gi_grd = (real *) my_pmalloc(N*sizeof(real));
      gi_gind = (int *) my_pmalloc(N*sizeof(int));
      if (nc > 0)
        L_mlt = (real *) my_pmalloc(nc*sizeof(real));

      if (nlhs==3) {
        r2d_copy(mxGetPr(plhs[0] = mxCreateDoubleMatrix(N, 1, mxREAL)),X0,N);
        //r2d_copy(mxGetPr(plhs[1] = mxCreateDoubleMatrix(nc, 1, mxREAL)),pi0,nc);
        utmp=mxGetPr(plhs[1] = mxCreateDoubleMatrix(N_LNLCONSTR, 1, mxREAL));
        for (i=0;i<N_LNLCONSTR;i++) {
          if ((j=NPERM[i+N_BOUNDS])<0)
            j=-(j+1);
          utmp[i]=pi0[j];
        }
	plhs[2]=set_output_struct(bequal_m2);
      } else { // nlhs==1, just structure is on ouput
        plhs[0]=set_output_struct(bequal_m2);
      }

      if (!DENSE) {
        //nnz_Hes_Lagr = sphsetup(my_obj_no, 0, nc > 0, 0);   // ignore OBJSIGN
        //nnz_Hes_Lagr = sphsetup(my_obj_no, ow!=0, nc > 0, 0);  // ?BUG?
        nnz_Hes_Lagr = sphsetup(-1, ow!=0, nc > 0, 0);  // OK
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

  /* Function values or gradient computation */
  if (nrhs==2 && nlhs==3) {
    
    d2r_copy(X,sizechk(prhs[0],"x",N),N);
    t = sizechk(prhs[1],"0 or 1", 1);

    // update values of all (transformed) constraints 
    if (constr_val) {
      what = "c";
      conval(X, constr_val, &nerror);
    }
    for (i=0; i<N_CONSTR; i++)
      constraints_x[i]=gi_val(i,X,constr_val);

    /* Function values */
    if (*t == 0.) {
      what = "f";
      f_val = my_obj_no>=0 ? objval(my_obj_no, X, &nerror) : 0.;

      plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
      plhs[1] = mxCreateDoubleMatrix(N_INEQUAL, 1, mxREAL);
      plhs[2] = mxCreateDoubleMatrix(N_EQUAL, 1, mxREAL);
      *mxGetPr(plhs[0]) = OBJSIGN*f_val;
      if (N_INEQUAL)
        memcpy(mxGetPr(plhs[1]),constraints_x,N_INEQUAL*sizeof(double));
      if (N_EQUAL)
        memcpy(mxGetPr(plhs[2]),constraints_x+N_INEQUAL,N_EQUAL*sizeof(double));
      
      return;   // end of function value computation

    } else if (*t == 1.) {  /* gradient */
      what = "g";
      if (my_obj_no>=0)
        objgrd(my_obj_no, X, f_grad, &nerror);
      else
	memset(f_grad,0,N*sizeof(real));
      // swap sign if necessary
      if (OBJSIGN==-1) {
        for (i=0;i<N;i++)
	  f_grad[i]*=-1;
      }

      plhs[0] = mxCreateDoubleMatrix(N, 1, mxREAL);
      r2d_copy(mxGetPr(plhs[0]),f_grad,N);

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
      return;  // end of gradient computation
    } else   // neither function values nor gradient --> error
      usage("should have been function or gradient values computations, but wrong number");

  }  // end of if (nrhs==2), e.i., func/grad computation

  /* Write stub.sol file */
  if (nlhs == 0 && (nrhs == 3 || nrhs == 4) && mxIsChar(prhs[0])) {
    /* eval2('solution message', x, v): x = primal, v = dual */
    /* optional 4th arg = solve_result_num */
    xx = sizechk(prhs[1],"x",N);
    //v = sizechk(prhs[2],"v",nc);
    //v should be transformed back - the same way as in pennon_ampl
    // Let's be generous - dimension of v could be N_LNLCONSTR or N_CONSTR (then use only last N_LNLCONSTR multipliers and ignore first N_BOUNDS box-constr multipliers)
    if (MXISOK(prhs[2],N_CONSTR,1))
      v = mxGetPr(prhs[2]) + N_BOUNDS;  
    else
      v = sizechk(prhs[2],"v",N_LNLCONSTR);
    memset(L_mlt,0,nc*sizeof(real));
    for (i=N_BOUNDS; i<N_CONSTR; i++) {
      itmp=1;
      if ((j=NPERM[i])<0) {
        j=-(j+1);
        itmp=-1;
      }
      L_mlt[j] += itmp*v[i-N_BOUNDS];
    }
    if (mxGetString(prhs[0], buf, sizeof(buf)))
      mexErrMsgTxt("Expected 'solution message' as first argument\n");
    solve_result_num = nrhs == 3 ? -1 /* unknown */
			: (int)*sizechk(prhs[3],"solve_result_num",1);
    write_sol(buf, xx, L_mlt, 0);
    return;
  }

  /* Compute a Hessian matrix of Lagrangian */
  if (nlhs == 1 && nrhs == 1) {
    if (N_CONSTR) {  // MODE==2: v could be of dimension N_LNLCONSTR or N_CONSTR
      if (MXISOK(prhs[0],N_CONSTR,1))
	v = mxGetPr(prhs[0]) + N_BOUNDS;  
        // dimension of v is N_CONSTR -> use only last N_LNLCONSTR multipliers
      else
	v = sizechk(prhs[0],"v",N_LNLCONSTR);
    } else   // no constraints -> no v
      v=0;

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
    return;
  } else {  // not Hessian??
    usage("strange input/output, do not know what to do");
  }

}  // of main()


