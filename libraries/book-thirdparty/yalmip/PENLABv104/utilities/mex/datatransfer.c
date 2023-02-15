/*
 * Data Transfer, see *.h file for a description
 *
 * version: 0.6
 * last modification: 07/03/2008
 *
 */

/*
 * My notes - what not to forget!!!
 * While reading a data block:
 *   - check the mode if opened for reading
 *   - check if the offset could be valid (done in DCF_seek())
 * While writing a data block:
 *   - check the mode
 *   - calculate a new offset (include DTYPES when computing its size)
 *   - check if there is any space to write a new data block
 *   - write DTYPE!
 *
 * My idea of using read-functions:
 *   a) open a file for reading and find number of items/comments/...
 *   b) choose number of data item
 *   c) jump to it with DCF_seek() and get its DTYPE
 *   d) based on its type read dimensions/no of nonzeroes/... by DCF_read_dim()
 *   e) allocate sufficient memory
 *   f) read the rest of the data item by rough interface
 *
 * My idea of (the reason of) an empty object EMPT
 *   Empty objects are useful when automatically transfering several items but some
 *   of them do not have to always exist. In 0.51 version such empty items could
 *   be only skipped but then offsets (positions of other blocks in the file) were
 *   shifted.
 *   Therefore a new "empty object" is introduced to fill in the gap if the object
 *   doesn't exist. The empty object will be allocated in Matlab as a matrix 0x0,
 *   in Matlib it will be just reported but there is nothing to do with it in Matlib.
 *   
 */

/*
 * Define MATLAB, MATLIB or nothing (for rough interface only) in Makefile 
 * to activate a corresponing part
 *
 * Warning! One thing is not very nice - if an error occurs during the writing phase
 * in an item (i.e., DCF_save_something() returns 0), the offset of the item is not properly
 * written (is left unfilled and wrt_items is not increased) so the following item starts
 * somewhere in the middle and later cannot be read. On the other hand, the ('new') item
 * repairs the offsets so the following is OK. If the wrong item is being read, it will
 * process nonsence data...
 *
 */


#define PRINT_WARNING    1   // if true, program prints errors/warning on screen with printf

#include "datatransfer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static FILE *errstream=0; // should set stderr or stdout everytime it is necessary

/* Set error stream by user, expect opened stream, it is responsibility of the user to
   ensure proper opening/closing or to set another stream if this one is closed */
void set_errstream(FILE *my) {
  if (my) {
    errstream=my;
    setvbuf(errstream,0,_IONBF,0);
  }
}

/* Close stream and free memory of allocated DCF */
void DCF_close(DCFILE *dcf) {
  unsigned old_items;

  if (dcf) {
    if (dcf->mode==1) {  // write offsets to file
      old_items = dcf->wrt_items;
      while (dcf->wrt_items < dcf->no_items)
	DCF_save_EMPT(dcf);
      fseek(dcf->f,dcf->offset_offset,SEEK_SET);
      if (fwrite(dcf->offset,sizeof(unsigned long),dcf->no_items,dcf->f)!=dcf->no_items)
	if (PRINT_WARNING)
	  fprintf(errstream,"Error: File '%s' is corrupted (offsets not written)!\n", dcf->filename);
      if (PRINT_WARNING && old_items<dcf->no_items)
        fprintf(errstream,"Warning: The file was planned for %u items but only %u were written.\nThe rest was filled with empty items.\n",dcf->no_items, old_items);
    }
    if (dcf->filename) free(dcf->filename);
    if (dcf->f) fclose(dcf->f);
    if (dcf->comment) free(dcf->comment);
    if (dcf->offset) free(dcf->offset);
    free(dcf);
  }
}

/* Allocates a new container & fills in basic data */
static DCFILE *DCF_get(const char *filename, unsigned no_items, int comm_len) {
  DCFILE *dcf;

  if ((dcf=(DCFILE *) malloc(sizeof(DCFILE)))==0)
    return 0;
  dcf->f=0;
  dcf->mode=-1;
  dcf->filename = (char *) malloc((strlen(filename)+1)*sizeof(char));
  dcf->comment = (char *) malloc((comm_len+1)*sizeof(char));
  dcf->offset = (unsigned long *) calloc(no_items+1,sizeof(unsigned long));
  if (!dcf->filename || !dcf->comment || !dcf->offset) {
    DCF_close(dcf);
    return 0;
  }
  strcpy(dcf->filename, filename);
  dcf->wrt_items=0;
  dcf->no_items=no_items;
  dcf->comment[0]=0;
  dcf->offset_offset=2*sizeof(unsigned) + comm_len*sizeof(char);
  dcf->offset[0]=dcf->offset_offset + no_items*sizeof(unsigned long);
  //other dcf->offset[] are set to zero;
  return dcf;
}

/* Opens DCF for writing, write a header of the file and set the cursor of te file to the first
   data block-to-be-written */
DCFILE *DCF_wopen(const char *filename, const char *comment, unsigned no_items) {
  DCFILE *dcf;
  char empty_string[]="";
  unsigned comm_len;

  if (!errstream)
    errstream=stderr;

  if (!filename || !no_items)
    return 0;
  if (!comment)
    comment=empty_string;

  comm_len=strlen(comment);
  if ((dcf=DCF_get(filename,no_items,comm_len))==0)
    return 0;
  dcf->mode=1;
  if ((dcf->f=fopen(filename,"wb"))==0) {
    if (PRINT_WARNING)
      fprintf(errstream,"Error: File '%s' cannot be opened!\n",filename);
    dcf->mode=-1;
    DCF_close(dcf);
  }
  strcpy(dcf->comment, comment);

  fwrite(&no_items, sizeof(unsigned),1,dcf->f);
  fwrite(&comm_len, sizeof(unsigned),1,dcf->f);
  if (comm_len) fwrite(comment, sizeof(char), comm_len, dcf->f);
  fwrite(dcf->offset, sizeof(long), no_items, dcf->f);

  return dcf;
}

/* Opens DCF for reading, reads the header from a file, sets the cursor of the stream to the
   first block, returns 0 when error */
DCFILE *DCF_ropen(const char *filename) {
  DCFILE *dcf;
  FILE *f;
  unsigned tmp[2];
  int iii;

  if (!errstream)
    errstream=stderr;

  if (!filename || (f=fopen(filename,"rb"))==0) {
    if (PRINT_WARNING)
      fprintf(errstream,"Error: File '%s' cannot be opened!\n",filename);
    return 0;
  }

  iii=fread(tmp,sizeof(unsigned),2,f);
  if (/*fread(tmp,sizeof(unsigned),2,f)*/iii!=2 || *tmp==0) {
    if (PRINT_WARNING)
      fprintf(errstream,"Error: File '%s' is not a valid Data Container File!\n",filename);
    fclose(f);
    return 0;
  }

  if ((dcf=DCF_get(filename,tmp[0],tmp[1]))==0)
    return 0;
  dcf->f=f;
  dcf->mode=0;
  if (tmp[1])
    if (fread(dcf->comment,sizeof(char),tmp[1],f)!=tmp[1])
      if (PRINT_WARNING)
        fprintf(errstream,"Error: File '%s' is not a valid DCF (@comment)!\n",filename);
  dcf->comment[tmp[1]]=0;
  if (fread(dcf->offset,sizeof(unsigned long),tmp[0],f)!=tmp[0]) {
    if (PRINT_WARNING)
      fprintf(errstream,"Error: File '%s' is not a valid DCF (@offset)!\n",filename);
    DCF_close(dcf);
    return 0;
  }
  return dcf;
}

/* Reads from the open DCF data item 'no', returns its type and leave the cursor just behind type
   in data block, on error returns _LAST_DTYPE, note: first data item has no==0 */
DTYPES DCF_seek(DCFILE *dcf, unsigned no) {
  DTYPES t;

  if (!dcf || dcf->mode!=0 || no>=dcf->no_items)
    return _LAST_DTYPE;

  if (dcf->offset[no]<dcf->offset_offset || dcf->offset[no]<dcf->offset[0] 
	 || fseek(dcf->f,dcf->offset[no],SEEK_SET)
	 || fread(&t,sizeof(DTYPES),1,dcf->f)!=1 || t>=_LAST_DTYPE) {
    if (PRINT_WARNING)
      fprintf(errstream,"Error: File '%s' inconsistency (invalid data block offset or type).\n",dcf->filename);
    return _LAST_DTYPE;
  }

  return t;
}

///////////// ROUGH INTERFACE for data block /////////////////

/* Series of functions to save/read all types of data blocks to/from an opened DCF
   in general, all read functions expect sufficient allocated memory by user
   return 1..successful, 0..error */

/* Just for simplification of often repeated error messages */
typedef enum {
  NO_WRITE,
  NO_READ,
  DCF_FULL,
  DATA_WRITE,
  UNSUC_DATA_ENTRY,
  POSITION_MISMATCH,
  READ_ERROR,
  INVALID_INPUT,
  INCOMPATIBLE_TYPE,
  READ_DIM,
  READ_DATA,
  WRITE,
  MEMORY,
  UNSPEC
} ERRMESSAGE;

static int errprn(ERRMESSAGE errmsg) {
  if (!errstream)
    errstream=stderr;

  if (PRINT_WARNING)
    switch (errmsg) {
    case NO_WRITE:
      fprintf(errstream,"Error: Container file not ready for writing.\n");
      break;
    case NO_READ:  
      fprintf(errstream,"Error: Container file not ready for reading.\n");
      break;
    case DCF_FULL:
      fprintf(errstream,"Error: Container file is full, not ready for writing a new item.\n");
      break;
    case DATA_WRITE:
      fprintf(errstream,"Error: Invalid input data for writing.\n");
      break;
    case UNSUC_DATA_ENTRY:
      fprintf(errstream,"Error: Data was not written properly.\n");
      break;
    case POSITION_MISMATCH:
      fprintf(errstream,"Error: Position, offset and length of the data block don't match.\n");
      break;
    case READ_ERROR:
      fprintf(errstream,"Error: Reading error, unexpected end of file or file cannot be read.\n");
      break;
    case INVALID_INPUT:
      fprintf(errstream,"Error: Invalid input of a function.\n");
      break;
    case INCOMPATIBLE_TYPE:
      fprintf(errstream,"Error: Incompatible input data type. Cannot write the item.\n");
      break;
    case READ_DIM:
      fprintf(errstream,"Reading Error: Data item corrupted (in dim/nnz/...).\n");
      break;
    case READ_DATA:
      fprintf(errstream,"Reading Error: Data item corrupted (data itself).\n");
      break;
    case WRITE:
      fprintf(errstream,"Error: Couldn't write to file.\n");
      break;
    case MEMORY:
      fprintf(errstream,"Error: Out of memory.\n");
      break;
    default:
      fprintf(errstream,"Unspecified error.\n");
      break;
    }
  return 0;
}

/* Save an empty object (just the indentificator DTYPE, nothing else) */
int DCF_save_EMPT(DCFILE *dcf) {
  DTYPES t=EMPT;
  unsigned long length=0;

  if (!dcf || dcf->mode!=1) {
    errprn(NO_WRITE);
    return 0;
  } else if (dcf->wrt_items>=dcf->no_items) {
    errprn(DCF_FULL);
    return 0;
  }
  
  if (fwrite(&t, sizeof(DTYPES), 1, dcf->f)!=1) {
    errprn(UNSUC_DATA_ENTRY);
    return 0;
  }
  length = sizeof(DTYPES);
  if (length+dcf->offset[dcf->wrt_items] != ftell(dcf->f))
    errprn(POSITION_MISMATCH);
  dcf->offset[++dcf->wrt_items] = ftell(dcf->f);
  return 1;
}

/* Save double matrix/vector ... m rows, n columns, data ~ array of m*n double elements stored
   in C-style, e.i., row-wise */
int DCF_save_DARR(DCFILE *dcf, unsigned m, unsigned n, double *data) {
  DTYPES t=DARR;
  unsigned long length=0;

  if (!dcf || dcf->mode!=1) {
    errprn(NO_WRITE);
    return 0;
  } else if (dcf->wrt_items>=dcf->no_items) {
    errprn(DCF_FULL);
    return 0;
  } else if (!data || m==0 || n==0) {
    errprn(DATA_WRITE);
    return 0;
  }
  
  if (fwrite(&t, sizeof(DTYPES), 1, dcf->f)!=1 || 
	fwrite(&m, sizeof(unsigned), 1, dcf->f)!=1 || 
	fwrite(&n, sizeof(unsigned), 1, dcf->f)!=1 || 
	fwrite(data,sizeof(double),m*n,dcf->f)!=m*n) {
    errprn(UNSUC_DATA_ENTRY);
    return 0;
  }
  length = sizeof(DTYPES) + 2*sizeof(unsigned) + m*n*sizeof(double);
  if (length+dcf->offset[dcf->wrt_items] != ftell(dcf->f))
    errprn(POSITION_MISMATCH);
  dcf->offset[++dcf->wrt_items] = ftell(dcf->f);
  return 1;
}

/* Save double matrix/vector ... m rows, n columns, data ~ array of m*n double elements stored
   in Matlab/Fortran-style, e.i., column-wise */
int DCF_save_DARRt(DCFILE *dcf, unsigned m, unsigned n, double *data) {
  DTYPES t=DARR;
  unsigned long length=0;
  unsigned i,j;

  if (!dcf || dcf->mode!=1) {
    errprn(NO_WRITE);
    return 0;
  } else if (dcf->wrt_items>=dcf->no_items) {
    errprn(DCF_FULL);
    return 0;
  } else if (!data || m==0 || n==0) {
    errprn(DATA_WRITE);
    return 0;
  }
  
  if (fwrite(&t, sizeof(DTYPES), 1, dcf->f)!=1 || 
	fwrite(&m, sizeof(unsigned), 1, dcf->f)!=1 || 
	fwrite(&n, sizeof(unsigned), 1, dcf->f)!=1) {
    errprn(UNSUC_DATA_ENTRY);
    return 0;
  }
  for (i=0; i<m; i++)
    for (j=0; j<n; j++)
      if (fwrite(data+i+j*m,sizeof(double),1,dcf->f)!=1) {
        errprn(UNSUC_DATA_ENTRY);
        return 0;
      }
  length = sizeof(DTYPES) + 2*sizeof(unsigned) + m*n*sizeof(double);
  if (length+dcf->offset[dcf->wrt_items] != ftell(dcf->f))
    errprn(POSITION_MISMATCH);
  dcf->offset[++dcf->wrt_items] = ftell(dcf->f);
  return 1;
}

/* Save index (int) array of length dim */
int DCF_save_IARR(DCFILE *dcf, unsigned dim, int *data) {
  DTYPES t=IARR;

  if (!dcf || dcf->mode!=1) {
    errprn(NO_WRITE);
    return 0;
  } else if (dcf->wrt_items>=dcf->no_items) {
    errprn(DCF_FULL);
    return 0;
  } else if (!data || dim==0) {
    errprn(DATA_WRITE);
    return 0;
  }

  if (fwrite(&t, sizeof(DTYPES), 1, dcf->f)!=1 || 
        fwrite(&dim,sizeof(unsigned),1,dcf->f)!=1 || 
	fwrite(data,sizeof(int),dim,dcf->f)!=dim) {
    errprn(UNSUC_DATA_ENTRY);
    return 0;
  }
  dcf->offset[++dcf->wrt_items] = ftell(dcf->f);
  return 1;
}

/* Save index (signed integer) array of length where the size of each element
   doesn't necessarily have to be same as sizeof(int), size is in bytes */
int DCF_save_IARRds(DCFILE *dcf, unsigned dim, void *data, int size) {
  DTYPES t=IARR;
  int itmp;
  unsigned i;

  if (!dcf || dcf->mode!=1)
    return errprn(NO_WRITE);
  else if (dcf->wrt_items>=dcf->no_items)
    return errprn(DCF_FULL);
  else if (!data || dim==0)
    return errprn(DATA_WRITE);

  if (fwrite(&t, sizeof(DTYPES), 1, dcf->f)!=1 || 
        fwrite(&dim,sizeof(unsigned),1,dcf->f)!=1)
    return errprn(UNSUC_DATA_ENTRY);

  // Note: switch() cannot be used, because for example 'int' and 'long' could have the same size
  if (size==sizeof(char)) {
    for (i=0; i<dim; i++) {
      itmp = *((char*)data + i);
      if (fwrite(&itmp,sizeof(int),1,dcf->f)!=1)
        return errprn(UNSUC_DATA_ENTRY);
    }
  } else if (size==sizeof(short)) {
    for (i=0; i<dim; i++) {
      itmp = *((short*)data + i);
      if (fwrite(&itmp,sizeof(int),1,dcf->f)!=1)
        return errprn(UNSUC_DATA_ENTRY);
    }
  } else if (size==sizeof(int)) {
    for (i=0; i<dim; i++) {
      itmp = *((int*)data + i);
      if (fwrite(&itmp,sizeof(int),1,dcf->f)!=1)
        return errprn(UNSUC_DATA_ENTRY);
    }
  } else if (size==sizeof(long)) {
    for (i=0; i<dim; i++) {
      itmp = *((long*)data + i);
      if (fwrite(&itmp,sizeof(int),1,dcf->f)!=1)
        return errprn(UNSUC_DATA_ENTRY);
    }
  } else {
    return errprn(INCOMPATIBLE_TYPE);
  }
  dcf->offset[++dcf->wrt_items] = ftell(dcf->f);
  return 1;
}

/* Save sparse vector - dim, nnz; ind and data are arrays of length nnz with 0-based indices
   and real data */
int DCF_save_SVEC(DCFILE *dcf, unsigned dim, unsigned nnz, unsigned *ind, double *data) {
  DTYPES t=SVEC;

  if (!dcf || dcf->mode!=1) {
    errprn(NO_WRITE);
    return 0;
  } else if (dcf->wrt_items>=dcf->no_items) {
    errprn(DCF_FULL);
    return 0;
  } else if (!data || !ind || dim==0 || nnz==0 || nnz>dim) {
    errprn(DATA_WRITE);
    return 0;
  }

  if (fwrite(&t, sizeof(DTYPES), 1, dcf->f)!=1 || 
        fwrite(&dim,sizeof(unsigned),1,dcf->f)!=1 || 
        fwrite(&nnz,sizeof(unsigned),1,dcf->f)!=1 ||
        fwrite(ind,sizeof(unsigned),nnz,dcf->f)!=nnz || 
	fwrite(data,sizeof(double),nnz,dcf->f)!=nnz) {
    errprn(UNSUC_DATA_ENTRY);
    return 0;
  }
  dcf->offset[++dcf->wrt_items] = ftell(dcf->f);
  return 1;
}

/* Save sparse matrix in column compressed format - m rows, n columns, nnz nonzeroes,
   colstarts column indices of starts columns (dim n+1), rowind row indices (dim nnz),
   data real values of elements (dim nnz) */
int DCF_save_SMAT(DCFILE *dcf, unsigned m, unsigned n, unsigned nnz, unsigned *colstarts, unsigned *rowind, double *data) {
  unsigned tmp[3]={m,n,nnz};
  DTYPES t=SMAT;

  if (!dcf || dcf->mode!=1) {
    errprn(NO_WRITE);
    return 0;
  } else if (dcf->wrt_items>=dcf->no_items) {
    errprn(DCF_FULL);
    return 0;
  } else if (!data || !rowind || !colstarts || n*m*nnz==0 || nnz>m*n) {
    errprn(DATA_WRITE);
    return 0;
  }

  if (fwrite(&t, sizeof(DTYPES), 1, dcf->f)!=1 || 
        fwrite(tmp,sizeof(unsigned),3,dcf->f)!=3 || 
	fwrite(colstarts,sizeof(unsigned),n+1,dcf->f)!=n+1 ||
        fwrite(rowind,sizeof(unsigned),nnz,dcf->f)!=nnz || 
	fwrite(data,sizeof(double),nnz,dcf->f)!=nnz) {
    errprn(UNSUC_DATA_ENTRY);
    return 0;
  }
  dcf->offset[++dcf->wrt_items] = ftell(dcf->f);
  return 1;
}

/* Save (special) sparse matrix in column compressed format - m rows, n columns, nnz nonzeroes,
   colstarts column indices of starts columns (dim n+1), rowind row indices (dim nnz),
   data real values of elements (dim nnz)
   Possible symmetrization if square matrix (& if (symmetrization),
   possible 1-based indices correction (in colstarts, rowind) if (one_based) */
int DCF_save_SMATs(DCFILE *dcf, unsigned m, unsigned n, unsigned nnz, unsigned *colstarts, unsigned *rowind, double *data,int symmetrization, int one_based) {
  int add, sym;
  unsigned *col, *row;
  double *dat;
  unsigned i,j,flag,entry;

  if (!data || !rowind || !colstarts || n*m*nnz==0 || nnz>m*n) {
    errprn(DATA_WRITE);
    return 0;
  }

  add = one_based ? 1 : 0;
  sym = (m==n && symmetrization) ? 1 : 0;

  if (!add && !sym)   // nothing extra to do
    return DCF_save_SMAT(dcf,m,n,nnz,colstarts,rowind,data);

  if ((col=(unsigned *) calloc(n+1,sizeof(unsigned)))==0) {
    errprn(MEMORY);
    return 0;
  }
  if (!sym) {
    for (i=0;i<n+1;i++)
      col[i]=colstarts[i]-add;
  } else {  // symmetrize that, n==m
    for (j=0;j<n;j++)
      for (entry=colstarts[j]-add; entry<colstarts[j+1]-add; entry++) {
        i=rowind[entry]-add;
	col[j+1]++;
	if (i!=j)
          col[i+1]++;
      }
    for (j=1;j<n+1;j++)
      col[j]+=col[j-1];
  }
  nnz=col[n];

  if ((row=(unsigned *) malloc(nnz*sizeof(unsigned)))==0) {
    errprn(MEMORY);
    return 0;
  }
  if (!sym) {
    for (entry=0;entry<nnz;entry++)
       row[entry]=rowind[entry]-add;
    dat=data;
  } else {
    if ((dat = (double *) malloc(nnz*sizeof(double)))==0) {
      errprn(MEMORY);
      return 0;
    }
    for (j=0;j<n;j++)
      for (entry=colstarts[j]-add; entry<colstarts[j+1]-add; entry++) {
        i=rowind[entry]-add;
	row[col[j]]=i;
	dat[col[j]]=data[entry];
	col[j]++;
	if (i!=j) {
	  row[col[i]]=j;
	  dat[col[i]]=data[entry];
          col[i]++;
	}
      }
      for (j=n;j>0;j--)
	col[j]=col[j-1];
      col[0]=0;
  }

  flag = DCF_save_SMAT(dcf,m,n,nnz,col,row,dat);

  free(col);
  free(row);
  if (sym)
    free(dat);

  return flag;
}

/* Read up to 3 unsigned int parameters (usually dimension/number of nnz etc.),
   a parameter is read only if its destination is not null --> use (f,&u1,&u2,&u3)
   or (f,&u1,&u2,0) or (f,&u1,0,0) to read 3/2/1 parameters,
   return number of read parameters, 0 on error */
int DCF_read_dim(DCFILE *dcf, unsigned *one, unsigned *two, unsigned *three) {
  unsigned tmp[3];
  int no=0,act=0;

  if (!dcf || dcf->mode!=0)
    return 0;
  if (one) {*one=0; no++;}
  if (two) {*two=0; no++;}
  if (three) {*three=0; no++;}
  if (!no)
    return 0;

  if (fread(tmp,sizeof(unsigned),no,dcf->f)!=no)
    return 0;
  if (one) *one=tmp[act++];
  if (two) *two=tmp[act++];
  if (three) *three=tmp[act++];
  return 1;
}

/* Read data block of DARR */
int DCF_read_DARR(DCFILE *dcf, unsigned m, unsigned n, double *data) {
  if (!dcf || dcf->mode!=0 || !data || m*n==0) {
    errprn(INVALID_INPUT);
    return 0;
  }
  if (fread(data,sizeof(double),m*n,dcf->f)!=m*n) {
    errprn(READ_ERROR);
    return 0;
  }
  return 1;
}

/* Read IRR */
int DCF_read_IARR(DCFILE *dcf, unsigned dim, int *data) {
  if (!dcf || dcf->mode!=0 || !data || dim==0) {
    errprn(INVALID_INPUT);
    return 0;
  }
  if (fread(data,sizeof(int),dim,dcf->f)!=dim) {
    errprn(READ_ERROR);
    return 0;
  }
  return 1;

}

/* Read SVEC */
int DCF_read_SVEC(DCFILE *dcf, unsigned nnz, unsigned *ind, double *data) {
  if (!dcf || dcf->mode!=0 || !data || !ind || !nnz) {
    errprn(INVALID_INPUT);
    return 0;
  }
  if (fread(ind,sizeof(unsigned),nnz,dcf->f)!=nnz ||
	fread(data,sizeof(double),nnz,dcf->f)!=nnz) {
    errprn(READ_ERROR);
    return 0;
  }
  return 1;
}

/* Read SMAT, n==no of columns */
int DCF_read_SMAT(DCFILE *dcf, unsigned n, unsigned nnz, unsigned *colstarts, unsigned *rowind, double *data) {
  if (!dcf || dcf->mode!=0 || !colstarts || !rowind || !data || nnz*n==0) {
    errprn(INVALID_INPUT);
    return 0;
  }
  if (fread(colstarts, sizeof(unsigned), n+1, dcf->f)!=n+1 ||
	fread(rowind, sizeof(unsigned), nnz, dcf->f)!=nnz ||
	fread(data,sizeof(double),nnz,dcf->f)!=nnz) {
    errprn(READ_ERROR);
    return 0;
  }
  return 1;
}


///////////////// MEX PART ///////////////////////

#ifdef  MATLAB
#include "mex.h"

// 0..print all text when reading file by default / 1..do not print unless you are asked to do it
#define DEFAULT_SILENT   0

/* Returns a pointer to the global scalar variable with 'name', 0 on error (doesn't exist,etc)
   Assume that the variable is global, real matrix with dimensions 1x1*/
static double *get_global_scalar(const char *name) {
  mxArray *var;
  
  if (name && (var=mexGetVariable("global",name))!=0 && mxIsNumeric(var) && !mxIsComplex(var) && mxGetM(var)==1 && mxGetN(var)==1)
    return mxGetPr(var);
  else
    return 0;
}

/* Print error message & exit with 0 (just for convinience of repeated error message */
static int mexErrPrn(ERRMESSAGE id) {
  switch (id) {
    case READ_DIM:
      mexPrintf("Reading Error: Data item corrupted (in dim/nnz/...).\n");
      break;
    case READ_DATA:
      mexPrintf("Reading Error: Data item corrupted (data itself).\n");
      break;
    case WRITE:
      mexPrintf("Error: Couldn't write to file.\n");
      break;
    default:
      mexPrintf("Error: Data item corrupted (unspecified).\n");
      break;
  }
  return 0;
}

/* Read from an opened Data Container File data item no 'no'; in detail:
   seek, check, print info, and read if mx!=0, returns 0 on error, 1 if OK
   if (be_silent && mx) then suppress information messages while reading items */
int read_item(DCFILE *dcf, unsigned no, mxArray **mx, int be_silent) {
  unsigned dim, n, m, nnz, i, j;
  double *dmex, *dtmp;
  int *imex;

  switch (DCF_seek(dcf,no)) {
    case DARR:
      if (!DCF_read_dim(dcf,&m,&n,0) || m*n==0)
	return mexErrPrn(READ_DIM);
      if (!mx || !be_silent)
        mexPrintf("Double array, dimensions %ux%u\n",m,n);
      if (mx) {
        dmex=mxGetPr(*mx=mxCreateDoubleMatrix(m,n,mxREAL));
	if ((dtmp=(double *)malloc(m*n*sizeof(double)))==0) {
	  mexPrintf("Error @ read_item(), cannot allocate temporary memory.\n");
	  return 0;
	}
	if (!DCF_read_DARR(dcf,m,n,dtmp))
	  return mexErrPrn(READ_DATA);
	/* 'transposition' needed for Matlab (row-wise to column-wise organization) */
	for (i=0;i<m;i++)
	  for (j=0; j<n; j++)
	    dmex[i + j*m]=dtmp[i*n + j];
	free(dtmp);
      }
      return 1;
    case IARR:
      if (!DCF_read_dim(dcf,&dim,0,0) || dim==0)
	return mexErrPrn(READ_DIM);
      if (!mx || !be_silent)
        mexPrintf("Integer vector, dimension %u\n",dim);
      if (mx) {
	switch (sizeof(int)) {
	  case 1:
            *mx=mxCreateNumericMatrix(dim,1,mxINT8_CLASS,mxREAL);
	    break;
	  case 2:
            *mx=mxCreateNumericMatrix(dim,1,mxINT16_CLASS,mxREAL);
	    break;
	  case 8:
            *mx=mxCreateNumericMatrix(dim,1,mxINT64_CLASS,mxREAL);
	    break;
	  case 4:
	  default:
            *mx=mxCreateNumericMatrix(dim,1,mxINT32_CLASS,mxREAL);
	    break;
	}
	imex=(int *) mxGetData(*mx);
	if (sizeof(int)!=mxGetElementSize(*mx)) {
	  mexPrintf("Error! Different size of integer types (int in C %i, int in Matlab %i).\nCannot find a compatibile integer type.\n",sizeof(int),mxGetElementSize(*mx));
	  return 0;
	}
	if (!DCF_read_IARR(dcf,dim,imex))
	  return mexErrPrn(READ_DATA);
      }
      return 1;
    case SMAT:
      if (!DCF_read_dim(dcf,&m,&n,&nnz) || m*n*nnz==0 || nnz>m*n)
	return mexErrPrn(READ_DIM);
      if (!mx || !be_silent)
        mexPrintf("Sparse matrix, dimensions %ux%u, nonzeroes %u\n",m,n,nnz);
      if (mx) {
	*mx=mxCreateSparse(m,n,nnz,mxREAL);
	if (!DCF_read_SMAT(dcf,n,nnz,mxGetJc(*mx),mxGetIr(*mx),mxGetPr(*mx)))
	  return mexErrPrn(READ_DATA);
      }
      return 1;
    case SVEC:
      if (!DCF_read_dim(dcf,&dim,&nnz,0) || dim*nnz==0 || nnz>dim)
	return mexErrPrn(READ_DIM);
      if (!mx || !be_silent)
        mexPrintf("Sparse vector, dimension %u, nonzeroes %u\n",dim,nnz);
      if (mx) {
	*mx=mxCreateSparse(dim,1,nnz,mxREAL);
	if (!DCF_read_SVEC(dcf,nnz,mxGetIr(*mx),mxGetPr(*mx)))
	  return mexErrPrn(READ_DATA);
	imex=mxGetJc(*mx);
	imex[0]=0;
	imex[1]=nnz;
      }
      return 1;
    case EMPT:
      if (!mx || !be_silent)
        mexPrintf("Empty item\n");
      if (mx)
	*mx=mxCreateDoubleMatrix(0,0,mxREAL);
      return 1;
    default:
      mexPrintf("Error: Nonexisting or corrupted data item in the file.\n");
      if (mx)
	*mx=mxCreateDoubleMatrix(0,0,mxREAL);
      return 0;
  }
}

/* Tests if mxArray can be stored in Data Container File, if so, returns 'id' how to store it (>=1),
   otherwise 0 and print an error message (if prn) */
int check_item(const mxArray *mx, int prn) {
  int ret=0, no_dim,isint=0,isunsigned=0;

  if (!mx)
    return 0;

  switch (mxGetClassID(mx)) {
    case mxSINGLE_CLASS:
      if (prn) 
	mexPrintf("Transform single precision to double at first. Single precision cannot be stored.\n");
	//mexPrintf("Info: Single precision will be stored as double.\n");
	// perhaps it should be OK since mxGetPr() redturns double * ... :)
	break;
    case mxDOUBLE_CLASS:
      ret=2;
      break;
    case mxUINT8_CLASS:
    case mxUINT16_CLASS:
    case mxUINT32_CLASS:
    case mxUINT64_CLASS:
      isunsigned=1;
    case mxINT8_CLASS:
    case mxINT16_CLASS:
    case mxINT32_CLASS:
    case mxINT64_CLASS:
      ret=5;
      isint=1;
      break;
    default:
      if (prn) 
	mexPrintf("Warning: class '%s' cannot be stored in Data Container File.\n",mxGetClassName(mx));
      break;
  }

  if (ret) {
    if (mxIsComplex(mx))
      if (prn) 
	mexPrintf("Warning: Only real part of complex number will be stored.\n");
    no_dim=mxGetNumberOfDimensions(mx);
    if (no_dim>2) {
      if (prn) 
	mexPrintf("%i-dimensional array cannot be stored.\n",no_dim);
      return 0;
    } 
    if(mxGetM(mx)<1 || mxGetN(mx)<1) {
      //if (prn) 
      //  mexPrintf("Error: Invalid dimensions, item not saved.\n");
      return 6;
    }
    if (isint && (mxGetN(mx)!=1 || mxIsSparse(mx))) {
      if (prn) 
	mexPrintf("Error: Only full integer vectors can be stored.\n");
      return 0;
    }
    if (prn && isint) {
      if (mxGetElementSize(mx)<sizeof(int))
	mexPrintf("Info: Integers will be stored as %i-bit.\n",8*sizeof(int));
      else if (mxGetElementSize(mx)>sizeof(int))
	mexPrintf("Serious warning: Integers %i bit will be stored as %i-bit, possible overflow!!!\n",mxGetElementSize(mx)*8,8*sizeof(int));
      else if (isunsigned /*&& sizes are equal*/)
	mexPrintf("Serious warning: unsigned integer is stored as signed, possible misinterpretation.\n");
    }
    if (ret==2) {
      if (mxIsSparse(mx))
	ret = mxGetN(mx)==1 ? 3 : 4;
    }
  }
  return ret;
}

/* Print usage & exit Mex File */
void usage(void) {
  mexErrMsgTxt("Help for data transfer by Data Container File\n\
  Reading:\n\
    dt('file')  .............. listing of the content of the file\n\
    x=dt('file',no) .......... read to 'x' data item number 'no' from the file\n\
    [x,y,z,...]=dt('file') ... read as many data items from the file as possible,\n\
       the first one to 'x', second to 'y' etc., it is not a problem if the numbers of\n\
       variables and data items in the file don't match\n\
    Note: if global scalar DCF_SILENT is defined (and nonzero) info messages while\n\
       reading items are suppressed\n\n\
  Writing:\n\
    dt('file','comment in file',x,y,z,...) ... store in the file items x,y,z,...\n\n\
  Possible to use it for real scalar/vector/matrices full and sparse and full\n\
  integer vectors.\n");
}

/* MeXfunction - matlab interface to Data Container File, basic usage - see usage()
 * Notes:
 *   The file is binary and should be read by DCF functions, its format - see datatransfer.h.
 *   All errors should be checked so it shouldn't mind if 'no' is out of range etc.
 *   dt('file') returns number of items in the file, a listing in detail is printed on the screen
 *   [x,y,z,...]=dt('file') unused variables (e.i., insuficient number of items in the file) are
 *     filled with 0; there is just a warning of unused data items and only first n are loaded if
 *     there is insufficient numebr of variables
 *   dt('file','comment',x,y,z,...) it is possible to save as many parameters as you intend
 *     invalid data types are ignored, unsuitable types are converted if possible (+warning/info)
 *     the comment is there for reminding what is in the file, can be anything even empty ('')
 *   Data items in file are stored in double/int (==INT32??) and sparsity is preserve.
 *
 *   File 'matlab_error_log.txt' will be created for 'internal' (=errprn) error messages
 */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  DCFILE *dcf;
  char filename[512], comment[4192];
  unsigned no, read_max, id;
  const mxArray *mx;
  FILE *myerrstream=0;
  int be_silent=DEFAULT_SILENT;
  double *pdtmp;

  /* Specify error stream (stderr is not printed out in Matlab) */
  if ((myerrstream=fopen("dcf_matlab_error_log.txt","w"))!=0)
    set_errstream(myerrstream);

  /* Open for reading */
  if ((nrhs==1 || nrhs==2) && mxIsChar(prhs[0]) && 
	(nrhs!=2 || mxIsNumeric(prhs[1]) && !mxIsComplex(prhs[1]) && 
	                     mxGetM(prhs[1])==1 && mxGetN(prhs[1])==1)) {
    if (mxGetString(prhs[0], filename, sizeof(filename)-1))
      mexErrMsgTxt("Error occured while reading the filename.\n");
    if ((dcf=DCF_ropen(filename))==0)
      mexErrMsgTxt("Couldn't open the file.\n");
    if ((pdtmp = get_global_scalar("DCF_SILENT"))!=0)
      be_silent= *pdtmp ? 1 : 0;
    if (!be_silent || (nrhs==1 && nlhs==0))
      mexPrintf("File: '%s'\nNote: %s\nNo items: %u\n-----------------\n",dcf->filename,dcf->comment,dcf->no_items);
    
    if (nrhs==1 && nlhs==0) {      // listing only
      mexPrintf("Listing:\n");
      for (no=0; no<dcf->no_items; no++)
	read_item(dcf,no,0,0);
      plhs[0] = mxCreateDoubleScalar(dcf->no_items);

    } else if (nrhs==2 && nlhs==1) {  // read just one
      no = (unsigned) *mxGetPr(prhs[1]);
      if (!be_silent)
        mexPrintf("Reading item no %i (0-based):\n",no);
      read_item(dcf,no,&plhs[0],be_silent);

    } else if (nrhs==1 && nlhs>=1) {  // read all possible items
      if (!be_silent)
        mexPrintf("Reading all:\n");
      read_max = nlhs > dcf->no_items ? dcf->no_items : nlhs;
      for (no=0; no<read_max; no++)
        read_item(dcf,no,&plhs[no],be_silent);

      if (read_max < dcf->no_items) {
        if (!be_silent)
          mexPrintf("Warning: %i saved data items in the file were not used.\n",dcf->no_items-read_max);
      } else if (nlhs > dcf->no_items) {  // fill the rest with 0
	mexPrintf("Warning: %i last parameters were not assigned (no data in the file).\n",nlhs-dcf->no_items);
        for (;no<nlhs;no++) 
          plhs[no]=mxCreateDoubleMatrix(0,0,mxREAL);
      }
    } else {    // error
      DCF_close(dcf);
      usage();
    }
    DCF_close(dcf);
  } else if (nlhs==0 && nrhs>=3 && mxIsChar(prhs[0]) && mxIsChar(prhs[1])) { 
    /* open for writing */
    if (mxGetString(prhs[0], filename, sizeof(filename)-1) || 
	   mxGetString(prhs[1],comment,sizeof(comment)-1))
      mexErrMsgTxt("Error occured in first two parameters (filename, comment).\n");
    /* count usable parameters */
    for (no=2, read_max=0; no<nrhs; no++)
      if (check_item(prhs[no],1))
	read_max++;
    if (read_max==0) {
      mexPrintf("Error: No valid inputs (nothing which could be saved to Data Container File)\n");
      return;
    }
    if ((dcf=DCF_wopen(filename,comment,read_max))==0)
      mexErrMsgTxt("Couldn't open the file.\n");

    for (no=2; no<nrhs; no++) {
      if ((id=check_item(prhs[no],0))==0)
	continue;
      mx=prhs[no];
      switch (id) {
        /* // NOT USED, use case 2 instead
	 case 1:  // double full vector --> can be saved without any problems
	  if (!DCF_save_DARR(dcf,mxGetM(mx),1,mxGetPr(mx)))
            mexErrPrn(WRITE);
	  break; */
	case 2:  // double matrix --> has to be 'transposed'
	  if (!DCF_save_DARRt(dcf,mxGetM(mx),mxGetN(mx),mxGetPr(mx)))
	    mexErrPrn(WRITE);
	  else mexPrintf("Item saved.\n");
	  break;
	case 3:  // sparse double vector (i.e., in Matlab stored as sparse matrix mx1)
	  if (!DCF_save_SVEC(dcf,mxGetM(mx),mxGetNzmax(mx),mxGetIr(mx),mxGetPr(mx)))
	    mexErrPrn(WRITE);
	  else mexPrintf("Item saved.\n");
	  break;
	case 4:  // sparse double matrix --> no problems
	  if (!DCF_save_SMAT(dcf,mxGetM(mx),mxGetN(mx),mxGetNzmax(mx),mxGetJc(mx),mxGetIr(mx),mxGetPr(mx)))
	    mexErrPrn(WRITE);
	  else mexPrintf("Item saved.\n");
	  break;
	case 5:  // integer vector
	  if (!DCF_save_IARRds(dcf,mxGetM(mx),mxGetData(mx),mxGetElementSize(mx)))
	    mexErrPrn(WRITE);
	  else mexPrintf("Item saved.\n");
	  break;
	case 6:  // empty item
	  if (!DCF_save_EMPT(dcf))
	    mexErrPrn(WRITE);
	  else mexPrintf("Item saved.\n");
	  break;
	default:
	  mexPrintf("Sorry, internal error?, unsupported class ID, cannot be saved,\nDCF all items not used?");
	  break;
      }
    }
    DCF_close(dcf);
  } else {
    usage();	  
  }

  /* Close error stream if opened */
  if (myerrstream)
    fclose(myerrstream);

}  // end of mexFunction()

#endif  // of MATLAB

////////////////////////////////// MATLIB INTERFACE ////////////////////////////

/* Simple interface for Matlib. Sparse vectors are not supported, therefore are
   saved as normal ones. Integer vectors don't exist in Matlib context, needed to
   use 'direct' interface, e.i., allocate its memory in the program... */

#ifdef MATLIB

#include "matrix.h"
#include "spmat.h"

/* Saves VEC into an opened DCF, returns 0..error, 1..OK */
int DCF_save_VEC(DCFILE *dcf, VEC *v) {
  if (!dcf || dcf->mode!=1 || !v)
    return 0;
  return DCF_save_DARR(dcf,v->dim,1,v->ve);
}

/* Saves MAT, same return flag */
int DCF_save_MAT(DCFILE *dcf, MAT *m) {
  if (!dcf || dcf->mode!=1 || !m)
    return 0;
  return DCF_save_DARR(dcf,m->m,m->n,m->base);
}

/* Save SPMAT */
int DCF_save_SPMAT(DCFILE *dcf, SPMAT *spm) {
  if (!dcf || dcf->mode!=1 || !spm)
    return 0;
  return DCF_save_SMAT(dcf,spm->m,spm->n,spm->nnz,spm->Acol,spm->Aiind,spm->Annz);
}

/* Save double (scalar) as DARR 1x1 */
int DCF_save_double(DCFILE *dcf, double d) {
  if (!dcf || dcf->mode!=1)
    return 0;
  return DCF_save_DARR(dcf,1,1,&d);
}

/* Save int (scalar) as IARR dim 1 */
int DCF_save_int(DCFILE *dcf, int i) {
  if (!dcf || dcf->mode!=1)
    return 0;
  return DCF_save_IARR(dcf,1,&i);
}

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
double DCF_read_double(DCFILE *dcf, unsigned no) {
  unsigned m,n;
  double d;
  int itmp;
  
  switch (DCF_seek(dcf,no)) {
    case DARR:
      if (!DCF_read_dim(dcf,&m,&n,0) || m*n==0) {
	errprn(READ_DIM);
        break;
      }
      if (m==1 && n==1 && DCF_read_DARR(dcf,m,n,&d))
	return d;
      break;
    case IARR:
      if (!DCF_read_dim(dcf,&m,0,0) || m==0) {
	errprn(READ_DIM);
	break;
      }
      if (m==1 && DCF_read_IARR(dcf,m,&itmp))
	return itmp;
      break;
    default:  // not possible to recover double from that item
      break;
  }
  return 0.;
}

/* Read one integer from IARR of dim 1 ... could be useful */
int DCF_read_int(DCFILE *dcf, unsigned no) {
  unsigned dim;
  int i;

  if (DCF_seek(dcf,no)!=IARR)
    return 0;
  if (!DCF_read_dim(dcf,&dim,0,0) || dim==0) {
    errprn(READ_DIM);
    return 0;
  }
  if (dim==1 && DCF_read_IARR(dcf,dim,&i))
    return i;
  return 0;
}

/* Read a vector to VEC from MAT dimx1 or SVEC */
VEC *DCF_read_VEC(DCFILE *dcf, unsigned no) {
  unsigned m,n,nnz,dim;
  unsigned *indices;
  int i,innz;
  VEC *v=0;
  DTYPES t;

  switch (t=DCF_seek(dcf,no)) {
    case DARR:
      if (!DCF_read_dim(dcf,&m,&n,0) || m*n==0) {
	errprn(READ_DIM);
        return 0;
      }
      if (m!=1 && n!=1)
	return 0;
      dim = m==1 ? n : m;
      break;
    case SVEC:
      if (!DCF_read_dim(dcf,&dim,&nnz,0) || dim*nnz==0 || nnz>dim) {
	errprn(READ_DIM);
        return 0; 
      }
      if ((indices=(unsigned *) malloc(nnz*sizeof(unsigned)))==0) {
	errprn(MEMORY);
	return 0;
      }
      break;
    default:
      return 0;
  }

  if ((v=v_get(dim))==0) {
    errprn(MEMORY);
    return 0;
  }

  switch (t) {
    case DARR:
      if (!DCF_read_DARR(dcf,m,n,v->ve)) {
        errprn(READ_DATA);
	v_free(v);
	return 0;
      }
      break;
    case SVEC:
      if (!DCF_read_SVEC(dcf,nnz,indices,v->ve)) {
        errprn(READ_DATA);
	free(indices);
	v_free(v);
	return 0;
      }
      // 'sort' elements
      for (i=nnz-1;i>=0;i--)
	v->ve[indices[i]]=v->ve[i];
      for (i=0,innz=0;i<dim;i++) {
	if (indices[innz]==i) {
	  innz = innz>=nnz-1 ? innz : innz+1;
	} else
	  v->ve[i]=0.;
      }
      break;
  }
  return v;
}

/* Read a matrix to MAT from DARR mxn (no other possibility, SMAT is stored to SPMAT) */
MAT *DCF_read_MAT(DCFILE *dcf, unsigned no) {
  MAT *matr;
  unsigned m,n;

  if (DCF_seek(dcf,no)!=DARR || !DCF_read_dim(dcf,&m,&n,0) || m*n==0) {
    errprn(READ_DIM);
    return 0;
  }
  if ((matr=m_get(m,n))==0) {
    errprn(MEMORY);
    return 0;
  }
  if (!DCF_read_DARR(dcf,m,n,matr->base)) {
    errprn(READ_DATA);
    m_free(matr);
    return 0;
  }
  return matr;
}

/* Read a sparse matrix to SPMAT from SMAT in DCF */
SPMAT *DCF_read_SPMAT(DCFILE *dcf, unsigned no) {
  SPMAT *sm;
  unsigned m,n,nnz;

  if (DCF_seek(dcf,no)!=SMAT || !DCF_read_dim(dcf,&m,&n,&nnz) || m*n*nnz==0) {
    errprn(READ_DIM);
    return 0;
  }
  if ((sm=spm_get(m,n,nnz))==0) {
    errprn(MEMORY);
    return 0;
  }
  if (!DCF_read_SMAT(dcf,n,nnz,sm->Acol,sm->Aiind,sm->Annz)) {
    errprn(READ_DATA);
    spm_free(sm);
    return 0;
  }
  return sm;
}

/* Just for debugging purposes - read IARR and print it on the screen (nothing else) */
void DCF_print_IARR(DCFILE *dcf, unsigned no) {
  unsigned dim,i;
  int *idata;

  if (DCF_seek(dcf,no)!=IARR || !DCF_read_dim(dcf,&dim,0,0) || dim==0) {
    errprn(READ_DIM);
    return;
  }
  if ((idata = (int *) malloc(dim*sizeof(int)))==0) {
    errprn(MEMORY);
    return;
  }
  if (!DCF_read_IARR(dcf,dim,idata)) {
    errprn(READ_DATA);
  } else {   // print it
    printf("IARR vector:\n");
    for (i=0;i<dim;i++)
      printf("%3i element: %i\n",i,idata[i]);
  }
  free(idata);
}

/* List the content of a DCF file, returns number of items in file
   if(content) then content of each data item is read and printed as well */
int DCF_print(DCFILE *dcf,int content) {
  unsigned no,m,n,dim,nnz;
  VEC *vc;
  MAT *mt;
  SPMAT *sm;

  if (!dcf || dcf->mode!=0) {
    printf("No valid DCF to read.\n");
    return 0;
  }
  printf("Data Container File '%s'\nNote: %s\nNumber of data items: %u\n---------\n",dcf->filename,dcf->comment,dcf->no_items);
  for (no=0;no<dcf->no_items;no++) {
    switch (DCF_seek(dcf,no)) {
    case DARR:
      if (!DCF_read_dim(dcf,&m,&n,0) || m*n==0) {
	errprn(READ_DIM);
	break;
      }
      printf("Double array, dimensions %ux%u\n",m,n);
      if (content) {
	mt = DCF_read_MAT(dcf,no);
	if (mt) {
          m_print(mt);
	  m_free(mt);
	}
      }
      break;
    case IARR:
      if (!DCF_read_dim(dcf,&dim,0,0) || dim==0) {
	errprn(READ_DIM);
	break;
      }
      printf("Integer vector, dimension %u\n",dim);
      if (content)
	DCF_print_IARR(dcf,no); 
      break;
    case SMAT:
      if (!DCF_read_dim(dcf,&m,&n,&nnz) || m*n*nnz==0 || nnz>m*n) {
	errprn(READ_DIM);
	break;
      }
      printf("Sparse matrix, dimensions %ux%u, nonzeroes %u\n",m,n,nnz);
      if (content) {
	sm = DCF_read_SPMAT(dcf,no);
	if (sm) {
	  spm_print(sm,15);
	  spm_free(sm);
	}
      }
      break;
    case SVEC:
      if (!DCF_read_dim(dcf,&dim,&nnz,0) || dim*nnz==0 || nnz>dim) {
	errprn(READ_DIM);
	break;
      }
      printf("Sparse vector, dimension %u, nonzeroes %u\n",dim,nnz);
      if (content) {
	vc = DCF_read_VEC(dcf,no);
	if (vc) {
	  v_print(vc);
	  v_free(vc);
	}
      }
      break;
    case EMPT:
      printf("Empty item\n");
      break;
    default:
      printf("Error: Nonexisting or corrupted data item in the file.\n");
      break;
    }
  }  // end of for
  return dcf->no_items;
}


#endif  // of MATLIB





