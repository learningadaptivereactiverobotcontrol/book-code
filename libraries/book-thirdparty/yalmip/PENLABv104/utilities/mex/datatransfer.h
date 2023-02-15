/*
 *  Data Transfer - module for exchanging matrices, vectors, sparse matrices, ...
 *  among several applications in one Data Container File (DCF)
 *  (Note: Original purpose was for communication Pennlp -> Matlab.)
 *
 *  datatransfer.h
 *
 *  version: 0.6
 *  last modification: 07/03/2008
 *  
 *  Change list:
 *  ------------
 *  + added an empty object & DCF_SILENT mode (see Matlab usage())
 */

/*
 * File format of Data Container File (DCF)
 *
 * Header of the file is followed by data blocks of each data items.
 * Data blocks are for double scalar/vector/matrix, integer scalar/vector, sparse
 * vector and matrix.
 *
 * Header:
 *   unsigned no_items        no of items
 *   unsigned com_len         length of comment
 *   char comment[com_len]    comment without terminating 0, just characters
 *   unsigned long offsets[no_items]   offsets in the file where data items start
 *
 * Data block for DARR:
 *   DTYPES  type==DARR       identificator
 *   unsigned dimensions[2]   no of rows and columns
 *   double data[rows*columns]   in C order, e.i., row wise
 *
 * Data block for IARR:
 *   DTYPES type==IARR
 *   unsigned dim             dimension of the vector
 *   int data[dim]            real values of the vector elements
 *
 * Data block for SMAT:  (in compressed column format, indices start with 0)
 *   DTYPES type==SMAT
 *   unsigned dimensions[2]   rows, columns
 *   unsigned nnz             nonzeroes in the matrix
 *   unsigned colstarts[no_columns + 1]   where a compressed column starts in rowind[] & elements[] 
 *   unsigned rowind[nnz]     row indicies (0-based) of nonzero elements
 *   double elements[nnz]     real values of nonzero elements
 * 
 * Data block for SVEC
 *   DTYPES type==SVEC
 *   unsigned dim             dimension
 *   unsigned nnz             number of nonzero elements
 *   unsigned ind[nnz]        indices of nonzeros (0-based)
 *   double elements[nnz]     real values of nonzeros
 *
 * Data block for EMPT
 *   DTYPES type==EMPT        just indicate an empty object
 */

#ifndef __DATATRANSFER_H
#define __DATATRANSFER_H

#include <stdio.h>

// Possible types of data to save/load
typedef enum {
  DARR,   // double array = scalar, vector, full matrix
  IARR,   // index array = integer vector
  SMAT,   // sparse double matrix
  SVEC,   // sparse double vector
  EMPT,   // empty object
  _LAST_DTYPE  // just for checking of number of data types, add any new dtypes above
} DTYPES;	

// Data Container File
typedef struct {
  char *filename;          // name of the Data Container File
  FILE *f;                 // opened stream
  char mode;               // 0..reading, 1..writing, -1..error while opening
  unsigned wrt_items;      // number of written items, not used if mode==0
  unsigned no_items;       // number of [planned] items in the file
  char *comment;           // comment[-to-be] in the file (incl. termin. 0)
  unsigned long *offset;   // offsets of data blocks, note: offset[0]=header length
  unsigned long offset_offset;  // offset where array offset[] starts
} DCFILE;

/* Set error stream by user, expect opened stream, it is responsibility of the user to
   ensure proper opening/closing or to set another stream if this one is closed */
void set_errstream(FILE *my);

/* Close stream and free memory of allocated DCF */
void DCF_close(DCFILE *dcf);

/* Opens DCF for writing, write a header of the file and set the cursor of te file to the first
   data block-to-be-written */
DCFILE *DCF_wopen(const char *filename, const char *comment, unsigned no_items);

/* Opens DCF for reading, reads the header from a file, sets the cursor of the stream to the
   first block, returns 0 when error */
DCFILE *DCF_ropen(const char *filename);

/* Reads from the open DCF data item 'no', returns its type and leave the cursor just behind type
   in data block, on error returns _LAST_DTYPE, note: first data item has no==0 */
DTYPES DCF_seek(DCFILE *dcf, unsigned no);

/* Save an empty object (just the indentificator DTYPE, nothing else) */
int DCF_save_EMPT(DCFILE *dcf);

/* Save double matrix/vector ... m rows, n columns, data ~ array of m*n double elements stored
   in C-style, e.i., row-wise */
int DCF_save_DARR(DCFILE *dcf, unsigned m, unsigned n, double *data);

/* Save double matrix/vector ... m rows, n columns, data ~ array of m*n double elements stored
   in Matlab/Fortran-style, e.i., column-wise */
int DCF_save_DARRt(DCFILE *dcf, unsigned m, unsigned n, double *data);

/* Save index (int) array of length dim */
int DCF_save_IARR(DCFILE *dcf, unsigned dim, int *data);

/* Save sparse vector - dim, nnz; ind and data are arrays of length nnz with 0-based indices
   and real data */
int DCF_save_SVEC(DCFILE *dcf, unsigned dim, unsigned nnz, unsigned *ind, double *data);

/* Save sparse matrix in column compressed format - m rows, n columns, nnz nonzeroes,
   colstarts column indices of starts columns (dim n+1), rowind row indices (dim nnz),
   data real values of elements (dim nnz) */
int DCF_save_SMAT(DCFILE *dcf, unsigned m, unsigned n, unsigned nnz, unsigned *colstarts, unsigned *rowind, double *data);

/* Save (special) sparse matrix in column compressed format - m rows, n columns, nnz nonzeroes,
   colstarts column indices of starts columns (dim n+1), rowind row indices (dim nnz),
   data real values of elements (dim nnz)
   Possible symmetrization if square matrix (& if (symmetrization),
   possible 1-based indices correction (in colstarts, rowind) if (one_based) */
int DCF_save_SMATs(DCFILE *dcf, unsigned m, unsigned n, unsigned nnz, unsigned *colstarts, unsigned *rowind, double *data,int symmetrization, int one_based);

////////////  ROUGH INTERFACE - use with care /////////////////

/* Series of functions to save/read all types of data blocks to/from an opened DCF
   in general, all read functions expect sufficient allocated memory by user
   return 1..successful, 0..error */


/* Save double matrix/vector ... m rows, n columns, data ~ array of m*n double elements stored
   in Matlab/Fortran-style, e.i., column-wise */
int DCF_save_DARRt(DCFILE *dcf, unsigned m, unsigned n, double *data);

/* Save index (signed integer) array of length where the size of each element
   doesn't necessarily have to be same as sizeof(int), size is in bytes */
int DCF_save_IARRds(DCFILE *dcf, unsigned dim, void *data, int size);

/* Read up to 3 unsigned int parameters (usually dimension/number of nnz etc.),
   a parameter is read only if its destination is not null --> use (f,&u1,&u2,&u3)
   or (f,&u1,&u2,0) or (f,&u1,0,0) to read 3/2/1 parameters,
   return number of read parameters, 0 on error */
int DCF_read_dim(DCFILE *dcf, unsigned *one, unsigned *two, unsigned *three);

/* Read data block of DARR */
int DCF_read_DARR(DCFILE *dcf, unsigned m, unsigned n, double *data);

/* Read IRR */
int DCF_read_IARR(DCFILE *dcf, unsigned dim, int *data);

/* Read SVEC */
int DCF_read_SVEC(DCFILE *dcf, unsigned nnz, unsigned *ind, double *data);

/* Read SMAT, n==no of columns */
int DCF_read_SMAT(DCFILE *dcf, unsigned n, unsigned nnz, unsigned *colstarts, unsigned *rowind, double *data);


////////////////////////////////// MATLIB INTERFACE ////////////////////////////

/* Simple interface for Matlib. Sparse vectors are not supported, therefore are
   saved as normal ones. Integer vectors don't exist in Matlib context, needed to
   use 'direct' interface, e.i., allocate its memory in the program... */

#ifdef MATLIB

#include "matrix.h"
#include "spmat.h"

/* Saves VEC into an opened DCF, returns 0..error, 1..OK */
int DCF_save_VEC(DCFILE *dcf, VEC *v);

/* Saves MAT, same return flag */
int DCF_save_MAT(DCFILE *dcf, MAT *m);

/* Save SPMAT */
int DCF_save_SPMAT(DCFILE *dcf, SPMAT *spm);

/* Save double (scalar) as DARR 1x1 */
int DCF_save_double(DCFILE *dcf, double d);

/* Save int (scalar) as IARR dim 1 */
int DCF_save_int(DCFILE *dcf, int i);

/* Interface for reading expects that you know what is where stored in file,
   you call 'read VEC at position no' and it will check if there is a vector,
   allocate memory and read it. If information of the type of data items is needed
   then use DCF_seek() at first and then the following read interface.
   Since there are no SVEC/IARR, they will be stored as normal VEC 
   Note: because of SVEC and DARR can go both to VEC, there cannot be interface
     'just read a VEC from actual position.
   If error occurs, function returns 0, otherwise allocated structure. 
   'no' is 0-based! */

/* Read double (e.i., DARR 1x1, IARR 1; it could be taken from SVEC/SMAT but I don't expect
   to use that for practical purposes therefore just DARR/IARR is possible) from the item 'no' */
double DCF_read_double(DCFILE *dcf, unsigned no);

/* Read one integer from IARR of dim 1 ... could be useful */
int DCF_read_int(DCFILE *dcf, unsigned no);

/* Read a vector to VEC from MAT dimx1 or SVEC */
VEC *DCF_read_VEC(DCFILE *dcf, unsigned no);

/* Read a matrix to MAT from DARR mxn (no other possibility, SMAT is stored to SPMAT) */
MAT *DCF_read_MAT(DCFILE *dcf, unsigned no);

/* Read a sparse matrix to SPMAT from SMAT in DCF */
SPMAT *DCF_read_SPMAT(DCFILE *dcf, unsigned no);

/* Just for debugging purposes - read IARR and print it on the screen (nothing else) */
void DCF_print_IARR(DCFILE *dcf, unsigned no);

/* List the content of a DCF file, returns number of items in file
   if(content) then content of each data item is read and printed as well */
int DCF_print(DCFILE *dcf,int content);

#endif // of MATLIB


#endif

