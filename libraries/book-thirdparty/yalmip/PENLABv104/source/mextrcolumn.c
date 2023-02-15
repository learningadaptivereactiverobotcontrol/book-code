/*
 * Mex file to perform operation
 *    trace(i) = trace(A*S1*B*Si) + trace(A*Si*B*S1)   for i=1,...,m
 * where A, B, S1, Si are symmetric matrices of the same dimension, 
 * A & B are dense, S1 and Si are sparse. This is used to compute one 
 * column of contributions of a matrix constraint to the Hessian of 
 * the Augmented Lagrangian (here, Si matrices will be the derivatives
 * of the matrix constraint).
 *
 * Compilation (assumes that mex command is setup, if not call mex -setup):
 *    on 32-bit systems: mex -O mextrcolumn.c
 *    on 64-bit systems: mex -largeArrayDims -O mextrcolumn.c
 *
 * The strategy used here is a revised approach of 
 *    Fujisawa et al. - Exploiting sparsity in primal-dual interior-point
 *    methods for semidefinite programming, 1997,
 * as described in
 *    Fiala, Kocvara, Stingl - PENLAB: A Matlab solver for nonlinear
 *    semidefinite optimization, to appear.
 * We refer to it as a "look-ahead strategy with caching".
 *
 * Usage:
 *    column = mextrcolumn(A,S1,B,cell_Si)
 * where A,S1,B,Si are as described above and cell_Si is a 1-D cell array
 * of Si matrices. The matrices are not checked if they are symmetric
 * so be careful! Both triangles need to be present we heavily rely on that.
 *
 * This file is a part of PENLAB solver version 1.0 (20130217).
 * Copyright (c) 2013 by J. Fiala, M. Kocvara and M. Stingl
 *
 * PENLAB is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * The following is used during the computations:
 *    S1*B*Si = (Si*B*S1)'
 *    ==> trace(i) = 2*trace(A*S1*B*Si)
 * the evaluation is computed as follows
 *    (1) find nonzero rows in S1
 *    (2) find all row numbers which are nonzero in as least one Si
 *    (3) compute row by row of (S1*B) but only these elements (columns)
 *        which will be used in one of the multiplication (S1*B)*Si,
 *        i.e., only with the indices as Si has nonempty rows
 *    (4) for given row and for given nonempty S2 column, compute sum=(S1*B)*S2
 *        and push it to the trace tr(A*sum_ij)
 *    (3) and because of symmetry + tr(A*sum_ji)
 *
 *
 * last update: 19/2/2013 by JF
 */
/* Attempts for Matlab's mex adjustments for previous TraceDSDS
 * but now computing the whole column.
 *
 * So far just test with cell arrays, assumes on input:
 * [] = mextrcolumn(Q), Q=cell{...} of dense/sparse matrices or []
 */
#include <stdio.h>
#include <string.h>
#include "mex.h"
 
int compute_trace_vec(const mxArray *mxA,const mxArray *mxS1,const mxArray *mxB,const mxArray *mxSi, double *trace, mwIndex len) {

  mwIndex n=0;  /* reference dimension of all the matrices */
  mwIndex i, j, k, r;
  mwIndex idx, idxend, jidx;
  char msg[256];

  double *S1Bcol=0, *column=0;
  mwIndex *col=0, *row=0, *bReady=0;
  mwIndex ncols, nrows;

  mwIndex *S1jc=0, *S1ir=0;
  double *S1pr=0, *Apr=0, *Bpr=0;
  mwIndex **Sijc=0, **Siir=0;
  double **Sipr=0;

  mxArray *Si;

  for (i=0; i<len; i++)
    trace[i] = 0.0;

  /* input: all matrices were already checked (right dimensions, sparsity etc.) */
  n=(mwIndex) mxGetM(mxA);

  if (n>0) {
    /* allocate temp memory, in MEX it should automatically generate Matlab 
     * error if there is not enough memory, but let's be nice */
    row=(mwIndex *) mxMalloc(n*sizeof(mwIndex));
    col=(mwIndex *) mxMalloc(n*sizeof(mwIndex));
    S1Bcol=(double *) mxMalloc(n*sizeof(double));
    bReady=(mwIndex *) mxMalloc(n*sizeof(mwIndex));
    column=(double *) mxMalloc(n*sizeof(double));

    Sijc = (mwIndex **) mxMalloc(len*sizeof(mwIndex*));
    Siir = (mwIndex **) mxMalloc(len*sizeof(mwIndex*));
    Sipr = (double **) mxMalloc(len*sizeof(double*));

    if (!row || !col || !S1Bcol || !bReady || !column || 
        !Sijc || !Siir || !Sipr)
      mexErrMsgTxt("Error: Insufficient memory\n");

    Apr = mxGetPr(mxA);
    Bpr = mxGetPr(mxB);
    S1pr = mxGetPr(mxS1);
    S1ir = mxGetIr(mxS1);
    S1jc = mxGetJc(mxS1);

    if (!Apr || !Bpr || !S1pr || !S1ir || !S1jc)
      mexErrMsgTxt("Error: Matrices are corrupted?\n");

    for (i=0; i<len; i++) {
      Si = mxGetCell(mxSi, i);
      if (!Si) {
        sprintf(msg,"Error: %u element of the cell array (4th arg.) is empty.\n",(unsigned) i+1);
        mexErrMsgTxt(msg);
      }
      Sipr[i] = mxGetPr(Si);
      Siir[i] = mxGetIr(Si);
      Sijc[i] = mxGetJc(Si);
      if (!Sipr[i] || !Siir[i] || !Sijc[i])
        mexErrMsgTxt("Error: Matrices from the cell array are corrupted?\n");
    }

    /* look-ahead part of the strategy: create a list of all nonzero rows
     * of S1 (and store them in row[]) & remember all nonzeros columns
     * in any of Si matrices (and store them in col[]) */
    for (nrows=0,i=0; i<n; i++) {
      if (S1jc[i]<S1jc[i+1])
        row[nrows++]=i;
    }

    for (i=0; i<n; i++)
      col[i]=0;

    for (k=0; k<len; k++)
      for (i=0; i<n; i++)
        if (Sijc[k][i]<Sijc[k][i+1])
          col[i]=1;

    for (ncols=0,i=0; i<n; i++) {
      if (col[i])
        col[ncols++]=i;
    }

    /* reset caching flags, 
     * bReady[i]<=jdx ==> column[i] is not computed in jdx-th column yet 
     * [careful here, mwIndex is on 64-bit defined as size_t ==> unsigned
     * so don't use -1 as a flag] */
    for (i=0; i<n; i++)
      bReady[i]=0;

    /* go through all columns j listed in col[] (i.e., these that at least
     * one Si has some nonzeros in it) ==> will need some of column[] 
     * elements which stores j-th column of (A*S1*B) */
    for (jidx=0; jidx<ncols; jidx++) {
      j=col[jidx];

      /* compute j-th column of S1*B   (note S1 is symmetric) */
      for (r=0; r<nrows; r++) {
        S1Bcol[r]=0.0;
        idxend=S1jc[row[r]+1];
        for (idx=S1jc[row[r]]; idx<idxend; idx++)
          S1Bcol[r] += S1pr[idx]*Bpr[j*n+S1ir[idx]];
      }

      /* go through all j-th columns of all Si matrices */
      for (k=0; k<len; k++) {
        idx=Sijc[k][j];
        idxend=Sijc[k][j+1];
        if (idx==idxend)
          continue;

        /* nonempty j-th column ==> go through all nonzero (row) elements */
        for (; idx<idxend; idx++) {
          i=Siir[k][idx];
          if (bReady[i]<=jidx) {
            /* caching part of the strategy: column[i] is not computed yet */
            /* column[i] = sum_r A[i][r]*S1B[r][j]   (note A is symmetric) */
            column[i]=0.0;
            for (r=0; r<nrows; r++)
              column[i] += Apr[n*i+row[r]]*S1Bcol[r];

            bReady[i]=jidx+1;
          }
          trace[k] += column[i]*Sipr[k][idx];
        }
      }
    }

    /* we want trace(k) = tr(A*S1...) + tr(A*Si...) ==> multiply by 2 */
    for (k=0; k<len; k++)
      trace[k]*=2;

    /* mxMalloc should be freed automatically but anyway, ... */
    mxFree(row);
    mxFree(col);
    mxFree(S1Bcol);
    mxFree(bReady);
    mxFree(column);
    mxFree(Sijc);
    mxFree(Siir);
    mxFree(Sipr);
  }

  return 0;
}


/* mexFunction is the gateway routine for the MEX-file. */ 
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

  size_t n=0;  /* reference dimension of all the matrices */
  size_t dim1, dim2, len;
  mwIndex i;
  char msg[256];
  mxArray *mxSi;
  double *trace;

  /* Check input */
  if (nlhs!=1 && nlhs!=0)
    mexErrMsgTxt("Error: Expecting one output argument.\n");
  if (nrhs!=4)
    mexErrMsgTxt("Error: Expecting 4 input arguments.\n");

  /* input: matrices & right dimensions? */
  n=mxGetM(prhs[0]);
  for (i=0;i<3;i++)
    if (!mxIsNumeric(prhs[i]) || mxIsComplex(prhs[i]) ||
	    mxGetM(prhs[i])!=n || mxGetN(prhs[i])!=n) {
      sprintf(msg,"Error: Input %u is expected to be a symmetric matrix %ux%u (%ux%u).\n",(unsigned) i+1,(unsigned) n,(unsigned) n,(unsigned) mxGetM(prhs[i]),(unsigned) mxGetN(prhs[i]));
      mexErrMsgTxt(msg);
    }

  /* last one should be a cell vector */
  if (!mxIsCell(prhs[3])) {
    mexErrMsgTxt("Error: 4th input arguments should be a cell array.\n");
  }
  dim1=mxGetM(prhs[3]);
  dim2=mxGetN(prhs[3]);
  if (dim1<1 || dim2<1 || dim1>1 && dim2>1) {
    mexErrMsgTxt("Error: 4th input arguments should be a cell vector.\n");
  }
  len = dim1>1 ? dim1 : dim2;

  /* input: is it dense, sparse, dense, {sparse,...} */
  if (mxIsSparse(prhs[0])) {
    mexErrMsgTxt("Error: 1st input argument is expected to be a dense matrix\n");
  }
  if (!mxIsSparse(prhs[1])) {
    mexErrMsgTxt("Error: 2nd input argument is expected to be a sparse matrix\n");
  }
  if (mxIsSparse(prhs[2])) {
    mexErrMsgTxt("Error: 3rd input argument is expected to be a dense matrix\n");
  }

  /* check that all elements of cell array are as expected */
  for (i=0; i<len; i++)  {
    mxSi = mxGetCell(prhs[3], i);
    if (!mxSi) {
      sprintf(msg,"Error: %u element of the cell array (4th arg.) is empty.\n",(unsigned) i+1);
      mexErrMsgTxt(msg);
    }
    if (!mxIsNumeric(mxSi) || mxIsComplex(mxSi) ||
        mxGetM(mxSi)!=n || mxGetN(mxSi)!=n || !mxIsSparse(mxSi)) {
      sprintf(msg,"Error: All element of the cell array should be sparse matrices %ux%u (%u not).\n",(unsigned) n,(unsigned) n,(unsigned) i+1);
      mexErrMsgTxt(msg);
    }
  }

  /* alloc space for the results, if unsuccessful -> it stops itself */
  plhs[0]=mxCreateDoubleMatrix(len,1,mxREAL);
  trace = mxGetPr(plhs[0]);

  compute_trace_vec(prhs[0],prhs[1],prhs[2],prhs[3],trace,len);


}

