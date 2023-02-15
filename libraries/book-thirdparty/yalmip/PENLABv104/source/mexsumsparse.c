/*
 * Mex file to perform operation
 *   S = sum_i=1^n w_i*S_i
 * where w_i are real numbers and S_i are (sparse) symmetric matrices
 * of the same dimension (i.e., weighted sum of sparse matrices). 
 * Typically, it is used to evaluate value and derivatives of semidefinite 
 * consraints (e.g., A=A_0 + sum x_i*A_i). The matrices are sparse and
 * summing them in Matlab in a loop is quite expensive.
 *
 * Compilation (assumes that mex command is setup, if not call mex -setup):
 *    on 32-bit systems: mex -O mexsumsparse.c
 *    on 64-bit systems: mex -largeArrayDims -O mexsumsparse.c
 *
 * The algorithm used here is adapted version of summing sparse matrices from
 *    Tim Davis - Direct Methods for Sparse Linear Systems.
 *
 * Usage:
 *    S = mexsumsparse(w,cell_Si[,nnz])
 * where w is a double vector of weigths, cell_Si is 1D cell array of Si
 * matrices. The third argument is optional, if present and positive, it
 * is used as an estimate of total number of nonzeros of sparse result S.
 * If negative, the result S will be stored as a dense matrix (and we
 * will not bother with summing it as sparse). Note that symmetry is used
 * only to save one transposition to sort resulting sparse matrix. It is
 * assumed that both triangles of Si are present.
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
 *
 * last update: 19/2/2013 by JF
 */

#include <stdio.h>
#include <string.h>
#include "mex.h"
 
/* For whatever reason, store the result as a dense matrix S (already 
 * allocated). Some of Si are allowed to be dense as well. The size of
 * all matrices is (n x n) and there is nmat of them to be summed. */
int sumAsDense(const mwIndex n, const mwIndex nmat, const double *w, const mxArray *mxSi, double *S) {

  char msg[256];
  mwIndex i, j, idx, idxend;
  mxArray *Si;
  mwIndex *Sijc=0, *Siir=0;
  double *Sipr=0;
  double wi;

  /* clear the result */
  idxend = n*n;
  for (idx=0; idx<idxend; idx++)
    S[idx]=0.0;

  /* add each of Si matrix with appropriate weight */
  for (i=0; i<nmat; i++) {
    wi = w[i];
    if (wi==0.0)
      continue;

    Si = mxGetCell(mxSi, i);
    if (!Si) {
      sprintf(msg,"Error: %u element of the cell array (2nd arg.) is empty.\n",(unsigned) i+1);
      mexErrMsgTxt(msg);
    }

    if (mxIsSparse(Si)) {
      /* sparse case */
      Sipr = mxGetPr(Si);
      Siir = mxGetIr(Si);
      Sijc = mxGetJc(Si);
      if (!Sipr || !Siir || !Sijc)
        mexErrMsgTxt("Error: Sparse matrix from the cell array is corrupted?\n");
      /* dimension should be fine - checked before */
      for (j=0; j<n; j++) {
        idxend=Sijc[j+1];
        for (idx=Sijc[j]; idx<idxend; idx++)
          S[j*n + Siir[idx]] += wi*Sipr[idx];
      }
    }
    else {
      /* dense case */
      Sipr = mxGetPr(Si);
      if (!Sipr)
        mexErrMsgTxt("Error: Dense matrix from the cell array is corrupted?\n");

      /* dimension should be n - checked before */
      /* just a simple addition, perhaps use BLAS? */
      idxend = n*n;
      for (idx=0; idx<idxend; idx++)
        S[idx] += wi*Sipr[idx];
    }
  }
  return 0;
}

/* Store the result as a sparse matrix S (will be allocated here). 
 * All Si must be sparse (already checked before). The size of
 * all matrices is (n x n) and there is nmat of them to be summed. */
int sumAsSparse(const mwIndex n, const mwIndex nmat, mwIndex nnzS, const double *w, const mxArray *mxSi, mxArray **mxS) {

  char msg[256];
  mwIndex i, j, idx, idxend, irow, nnz, nnzmax;
  double wi;
  mxArray *Si;
  mwIndex **Sijc=0, **Siir=0;
  double **Sipr=0;
  mxArray *S=0;
  mwIndex *Sir=0, *Sjc=0, *irowS=0, *iccolS=0, *bReady=0;
  double *Spr=0, *dataS=0, *column=0;

  /* clear the result */
  *mxS=0;

  /* alloc temp space for sparse matrix for the result (estimated nnzS) */
  nnzmax = nnzS+n;
  iccolS = (mwIndex *) mxMalloc((n+1)*sizeof(mwIndex));
  irowS = (mwIndex *) mxMalloc(nnzmax*sizeof(mwIndex));
  dataS = (double *) mxMalloc(nnzmax*sizeof(double));
  if (!iccolS || !irowS || !dataS)
    mexErrMsgTxt("Error: Insufficient memory\n");

  /* alloc all remaining temp memory */
  bReady = (mwIndex *) mxMalloc(n*sizeof(mwIndex));
  column = (double *) mxMalloc(n*sizeof(double));

  Sijc = (mwIndex **) mxMalloc(nmat*sizeof(mwIndex*));
  Siir = (mwIndex **) mxMalloc(nmat*sizeof(mwIndex*));
  Sipr = (double **) mxMalloc(nmat*sizeof(double*));

  if (!bReady || !column || !Sijc || !Siir || !Sipr)
    mexErrMsgTxt("Error: Insufficient memory\n");

  for (i=0; i<nmat; i++) {
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

  /* reset flag array, 
   * bReady[i]<=j  ==> i-th row element in j-th column is not yet present */
  for (idx=0; idx<n; idx++)
      bReady[idx]=0;

  /* Create j-th column of S by summing j-th columns of Si as sparse vectors */
  nnz=0;        /* number of elements written so far */
  iccolS[0]=0;
  for (j=0; j<n; j++) {
    if (nnz+n>nnzmax) {
      /* not enough space to fit the whole column -> rather realloc,
       * and copy all the data */
      nnzmax = nnz + n*(1 + (n-j)/10);
      irowS = (mwIndex *) mxRealloc(irowS, nnzmax*sizeof(mwIndex));
      dataS = (double *)  mxRealloc(dataS, nnzmax*sizeof(double));
      if (!irowS || !dataS)
        mexErrMsgTxt("Error: Insufficient memory\n");
    }

    /* add j-th column of i-th matrix */
    for (i=0; i<nmat; i++) {
      wi=w[i];
      if (wi==0.0)
        continue;

      idxend = Sijc[i][j+1];
      for (idx=Sijc[i][j]; idx<idxend; idx++) {
        irow = Siir[i][idx];
        if (bReady[irow]<=j) {
          /* new element created */
          column[irow]=0.0;
          irowS[nnz++]=irow;
          bReady[irow]=j+1;
        }
        column[irow] += wi*Sipr[i][idx];
      }

    }

    /* column j done -> copy (gather) reals & update */
    for (idx=iccolS[j]; idx<nnz; idx++)
      dataS[idx]=column[irowS[idx]];
    iccolS[j+1]=nnz;
  }

  /* need to sort S, so far sorted by columns but not within the columns,
   * Si are symmetric ==> S is symmetric, it gets sorted by transposition */
  S = mxCreateSparse(n,n,nnz,mxREAL);
  if (!S)
    mexErrMsgTxt("Error: Insufficient memory\n");
  Spr = mxGetPr(S);
  Sir = mxGetIr(S);
  Sjc = mxGetJc(S);
  if (!Spr || !Sir || !Sjc)
    mexErrMsgTxt("Error: Sparse matrix for the result is corrupted?\n");
  for (j=0; j<=n; j++)
    Sjc[j]=0;
  /* count number of elements in j-th column in Sjc[j+1] */
  for (idx=0; idx<nnz; idx++)
    Sjc[irowS[idx]+1]++;
  /* accummulate the numbers */
  for (j=1; j<=n; j++)
    Sjc[j] += Sjc[j-1];
  /* finally copy everything */
  for (j=0; j<n; j++) {
    idxend=iccolS[j+1];
    for (idx=iccolS[j]; idx<idxend; idx++) {
      irow=irowS[idx];
      Sir[Sjc[irow]]=j;
      Spr[Sjc[irow]]=dataS[idx];
      Sjc[irow]++;
    }
  }
  /* fix Sjc (=shift it) and we are ready */
  for (j=n; j>=1; j--)
    Sjc[j]=Sjc[j-1];
  Sjc[0]=0;
  *mxS = S;

  /* mxMalloc should be freed automatically but anyway, ... */
  mxFree(irowS);
  mxFree(iccolS);
  mxFree(dataS);
  mxFree(bReady);
  mxFree(column);
  mxFree(Sijc);
  mxFree(Siir);
  mxFree(Sipr);

  return 0;
}

/* mexFunction is the gateway routine for the MEX-file. */ 
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

  size_t n, m, dim1, dim2;
  mwIndex i, nnzS=0, mynnzS=0, nmat;
  char msg[256];
  mxArray *mxSi;
  double *S=0, *w=0;

  /* Check input */
  if (nlhs!=1 && nlhs!=0)
    mexErrMsgTxt("Error: Expecting one output argument.\n");
  if (nrhs!=2 && nrhs!=3)
    mexErrMsgTxt("Error: Expecting 2 or 3 input arguments.\n");

  /* Check arguments */
  /* 3rd argument optional - a scalar */
  if (nrhs==3) {
    if (!mxIsNumeric(prhs[2]) || mxIsComplex(prhs[2]) ||
        mxGetM(prhs[2])!=1 || mxGetN(prhs[2])!=1)
      mexErrMsgTxt("Third argument should be a (numeric) scalar.\n");

    nnzS = (mwIndex) mxGetScalar(prhs[2]);
  }
  /* 1st: real vector - length as the number of matrices */
  if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]))
    mexErrMsgTxt("First argument should be a real vector.\n");
  /* 2nd: cell array */
  if (!mxIsCell(prhs[1]))
    mexErrMsgTxt("Second arguments should be a cell array.\n");

  /* matching number of matrices/weights? */
  m=mxGetM(prhs[0]);
  n=mxGetN(prhs[0]);
  dim1=mxGetM(prhs[1]);
  dim2=mxGetN(prhs[1]);
  if (n*m==0 || dim1*dim2==0) {
    if (n*m!=0 || dim1*dim2!=0)
      mexErrMsgTxt("First and second arguments don't match, one is empty.\n");
    else {
      /* return empty matrix */
      plhs[0]=mxCreateDoubleMatrix(0,0,mxREAL);
      return;
    }
  }
  if (n!=1 && m!=1)
    mexErrMsgTxt("First argument should be a vector.\n");
  if (dim1!=1 && dim2!=1)
    mexErrMsgTxt("Second argument should be a 1D cell array.\n");
  nmat = n>1 ? (mwIndex) n : (mwIndex) m;
  if (dim1!=nmat && dim2!=nmat)
    mexErrMsgTxt("First and second arguments don't match in size.\n");
  w=mxGetPr(prhs[0]);
  if (!w)
    mexErrMsgTxt("First argument is corrupted?\n");

  /* Si matrices & right dimensions? */
  for (i=0; i<nmat; i++)  {
    mxSi = mxGetCell(prhs[1], i);
    if (!mxSi) {
      sprintf(msg,"Error: %u element of the cell array (2nd arg.) is empty.\n",(unsigned) i+1);
      mexErrMsgTxt(msg);
    }
    /* get the dimension which should be the same for all */
    if (i==0)
      n=mxGetM(mxSi);
    if (!mxIsDouble(mxSi) || mxIsComplex(mxSi) ||
        mxGetM(mxSi)!=n || mxGetN(mxSi)!=n) {
      sprintf(msg,"Error: All element of the cell array should be matrices %ux%u (%u not).\n",(unsigned) n,(unsigned) n,(unsigned) i+1);
      mexErrMsgTxt(msg);
    }
    if (!mxIsSparse(mxSi)) {
      /* switch to dense result */
      nnzS=-1;
    }
    else if (!nnzS) {
      /* get my own estimate of number of nonzeros */
      mynnzS += mxGetNzmax(mxSi);
    }
  }

  if (nnzS<0) {
    /* compute as dense */
    plhs[0]=mxCreateDoubleMatrix(n,n,mxREAL);
    if (!plhs[0])
      mexErrMsgTxt("Allocation error.\n");
    S = mxGetPr(plhs[0]);
    if (!S)
      mexErrMsgTxt("Matrix for the result corrupted?\n");

    sumAsDense((mwIndex) n, nmat, w, prhs[1], S);
  } 
  else {
    /* Result is sparse and the estimate is provided or internal */
    if (nnzS==0)
      nnzS = mynnzS*4/nmat;
    sumAsSparse((mwIndex) n, nmat, nnzS, w, prhs[1], plhs);
  }
}

