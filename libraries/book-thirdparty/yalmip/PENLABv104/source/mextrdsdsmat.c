/*
 * Mex file to perform operation
 *   tr = trace(A*S1*B*S2) + trace(A*S2*B*S1)
 * where A, B, S1, S2 are symmetric matrices, A & B are dense,
 * S1 and S2 are sparse
 *
 * The following is used during the computations:
 *    S1*B*S2 = (S2*B*S1)'
 * the evaluation is computed as
 *    (1) compute one row of (S1*B) but only these elements (columns)
 *        which will be used in the multiplication (S1*B)*S2,
 *        i.e., only with the indices as S2 has nonempty rows
 *    (2) for given row and for given nonempty S2 column, compute sum=(S1*B)*S2
 *        and push it to the trace tr(A*sum_ij)
 *    (3) and because of symmetry + tr(A*sum_ji)
 *
 * Compilation (assumes that mex command is setup, if not call mex -setup):
 *   on 32-bit systems: mex -O mextrdsdsmat.c
 *   on 64-bit systems: mex -largeArrayDims -O mextrdsdsmat.c
 *
 * Changes:
 * - adjusted int -> mwSize/mwIndex to allow compatibility with 64-bit Matlab
 *
 * last update: 7/1/2013 by JF
 */

#include "mex.h"
#include <stdio.h>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

  size_t n=0;  /* reference dimension of all the matrices */
  mwIndex i, j;
  mwIndex irS1, idx, idxend, jidx;
  char msg[256];

  double *row=0, rowelem;
  double sum, trace=0.0;
  mwIndex *cols=0, *colsall=0;
  mwIndex ncols=0;

  mwIndex *S1jc=0, *S1ir=0, *S2jc=0, *S2ir=0;
  double *S1pr=0, *S2pr=0, *Apr=0, *Bpr=0;

  /* Check input */
  if (nlhs!=1 && nlhs!=0)
    mexErrMsgTxt("Error: Expecting one output argument.\n");
  if (nrhs!=4)
    mexErrMsgTxt("Error: Expecting 4 input arguments.\n");

  
  /* input: matrices & right dimensions? */
  n=mxGetM(prhs[0]);
  for (i=0;i<4;i++)
    if (!mxIsNumeric(prhs[i]) || mxIsComplex(prhs[i]) ||
	    mxGetM(prhs[i])!=n || mxGetN(prhs[i])!=n) {
      sprintf(msg,"Error: Input %u is expected to be a symmetric matrix %ux%u (%ux%u).\n",(unsigned) i+1,(unsigned) n,(unsigned) n,(unsigned) mxGetM(prhs[i]),(unsigned) mxGetN(prhs[i]));
      mexErrMsgTxt(msg);
    }

  /* input: is it dense, sparse, dense, sparse */
  for (i=0;i<2;i++)
    if (!mxIsSparse(prhs[2*i+1])) {
      sprintf(msg,"Error: Input %u is expected to be a sparse matrix\n", (unsigned) 2*i+2);
      mexErrMsgTxt(msg);
    }
  for (i=0;i<2;i++)
    if (mxIsSparse(prhs[2*i])) {
      sprintf(msg,"Error: Input %u is expected to be a dense matrix\n", (unsigned) 2*i+1);
      mexErrMsgTxt(msg);
    }

  if (n>0) {
    /* allocate temp memory, in MEX it should automatically generate Matlab 
     * error if there is not enough memory, but let's be nice */
    row=(double *) mxMalloc(n*sizeof(double));
    cols=(mwIndex *) mxMalloc(n*sizeof(mwIndex));
    /*colsall=(mwIndex *) mxCalloc(n,sizeof(mwIndex));*/
    if (!row || !cols /*|| !colsall*/)
      mexErrMsgTxt("Error: Insufficient memory\n");

    Apr = mxGetPr(prhs[0]);
    S1pr = mxGetPr(prhs[1]);
    S1ir = mxGetIr(prhs[1]);
    S1jc = mxGetJc(prhs[1]);
    Bpr = mxGetPr(prhs[2]);
    S2pr = mxGetPr(prhs[3]);
    S2ir = mxGetIr(prhs[3]);
    S2jc = mxGetJc(prhs[3]);
    if (!Apr || !Bpr || !S1pr || !S1ir || !S1jc || !S2pr || !S2ir || !S2jc)
      mexErrMsgTxt("Error: Matrices are corrupted?\n");

    /* remember nonempty rows in S2 because only these columns will be needed
     * from multiplication (S1*B) to get ((S1*B)*S2); 
     * S2 is symmetric so it's the same as nonempty columns in S2 */
    for (i=0; i<(mwIndex) n; i++)
      if (S2jc[i+1]-S2jc[i]>0) {
        cols[ncols]=i;
        /*colsall[i]=1;*/   /* otherwise initialized by 0 */
        ncols++;
      }

    /* go row by row in S1 (S1(irS1,:)) (S1 is symmetric --> by columns); 
     * for each nonempty one, compute row=S1(irS1,:)*B 
     * but only columns named in cols are needed so in fact, compute
     * row=S1(irS1,:)*B(:,cols);
     * compute row*S2 and add it into the trace */
    for (irS1=0; irS1<(mwIndex) n; irS1++) {
      idx=S1jc[irS1];
      idxend=S1jc[irS1+1];
      if (S1jc[irS1+1] == S1jc[irS1])
        continue;  /* nothing here -> skip */

      /* this is to assure that only elements filled-in in row are used */
      /*for (j=0; j<(mxIndex) n; j++)
          row[j]=-1e+30;               */

      for (jidx=0; jidx<ncols; jidx++) {
        j=cols[jidx];
        /*row[j]=0.0;*/
        rowelem=0.0;

        /* row(j)=S1(irS1,:)*B(:,j) */
        for (idx=S1jc[irS1]; idx<idxend; idx++) {
          /* rowelem += B(S1ir[idx],j) * ...?  */
          rowelem += S1pr[idx] * Bpr[n*j+S1ir[idx]];
        }
        row[j]=rowelem;
      }

      /* row*S2 */
      for (j=0; j<(mwIndex) n; j++) {
        idx=S2jc[j];
        idxend=S2jc[j+1];
        sum=0.0;
        if (idx==idxend)
          continue;

        for (idx=S2jc[j]; idx<idxend; idx++) {
          /* check if the element was filled in */
          /*if (!colsall[S2ir[idx]]) {
              sprintf(msg,"ERR! in row irS1=%i, column=%i element wasn't filled in rowel=%e!\n",irS1,S2ir[idx],row[S2ir[idx]]);
              in fact, this should be an error 
              mexWarnMsgTxt(msg);
            }   */

          sum += row[S2ir[idx]] * S2pr[idx];
        }

        /* sum is now final element of S1*B*S2 at irS1,j 
         * and because S2*B*S1 = (S1*B*S2)' it is also element at j,irS1
         * --> add it to trace twice */
        trace += (Apr[n*j+irS1] + Apr[n*irS1+j])*sum;
      }

    }

    /* mxMalloc should be freed automatically but anyway, ... */
    mxFree(row);
    mxFree(cols);
    /*mxFree(colsall);*/
  }

  plhs[0]=mxCreateDoubleScalar(trace);

}

