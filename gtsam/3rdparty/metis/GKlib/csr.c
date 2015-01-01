/*!
 * \file 
 *
 * \brief Various routines with dealing with CSR matrices
 *
 * \author George Karypis
 * \version\verbatim $Id: csr.c 13437 2013-01-11 21:54:10Z karypis $ \endverbatim
 */

#include <GKlib.h>

#define OMPMINOPS       50000

/*************************************************************************/
/*! Allocate memory for a CSR matrix and initializes it 
    \returns the allocated matrix. The various fields are set to NULL.
*/
/**************************************************************************/
gk_csr_t *gk_csr_Create()
{
  gk_csr_t *mat;

  mat = (gk_csr_t *)gk_malloc(sizeof(gk_csr_t), "gk_csr_Create: mat");

  gk_csr_Init(mat);

  return mat;
}


/*************************************************************************/
/*! Initializes the matrix 
    \param mat is the matrix to be initialized.
*/
/*************************************************************************/
void gk_csr_Init(gk_csr_t *mat)
{
  memset(mat, 0, sizeof(gk_csr_t));
  mat->nrows = mat->ncols = -1;
}


/*************************************************************************/
/*! Frees all the memory allocated for matrix.
    \param mat is the matrix to be freed.
*/
/*************************************************************************/
void gk_csr_Free(gk_csr_t **mat)
{
  if (*mat == NULL)
    return;
  gk_csr_FreeContents(*mat);
  gk_free((void **)mat, LTERM);
}


/*************************************************************************/
/*! Frees only the memory allocated for the matrix's different fields and
    sets them to NULL.
    \param mat is the matrix whose contents will be freed.
*/    
/*************************************************************************/
void gk_csr_FreeContents(gk_csr_t *mat)
{
  gk_free((void *)&mat->rowptr, &mat->rowind, &mat->rowval, &mat->rowids,
          &mat->colptr, &mat->colind, &mat->colval, &mat->colids, 
          &mat->rnorms, &mat->cnorms, &mat->rsums, &mat->csums, 
          &mat->rsizes, &mat->csizes, &mat->rvols, &mat->cvols, 
          &mat->rwgts, &mat->cwgts, 
          LTERM);
}


/*************************************************************************/
/*! Returns a copy of a matrix.
    \param mat is the matrix to be duplicated.
    \returns the newly created copy of the matrix.
*/
/**************************************************************************/
gk_csr_t *gk_csr_Dup(gk_csr_t *mat)
{
  gk_csr_t *nmat;

  nmat = gk_csr_Create();

  nmat->nrows  = mat->nrows;
  nmat->ncols  = mat->ncols;

  /* copy the row structure */
  if (mat->rowptr)
    nmat->rowptr = gk_zcopy(mat->nrows+1, mat->rowptr, 
                            gk_zmalloc(mat->nrows+1, "gk_csr_Dup: rowptr"));
  if (mat->rowids)
    nmat->rowids = gk_icopy(mat->nrows, mat->rowids, 
                            gk_imalloc(mat->nrows, "gk_csr_Dup: rowids"));
  if (mat->rnorms)
    nmat->rnorms = gk_fcopy(mat->nrows, mat->rnorms, 
                            gk_fmalloc(mat->nrows, "gk_csr_Dup: rnorms"));
  if (mat->rowind)
    nmat->rowind = gk_icopy(mat->rowptr[mat->nrows], mat->rowind, 
                            gk_imalloc(mat->rowptr[mat->nrows], "gk_csr_Dup: rowind"));
  if (mat->rowval)
    nmat->rowval = gk_fcopy(mat->rowptr[mat->nrows], mat->rowval, 
                            gk_fmalloc(mat->rowptr[mat->nrows], "gk_csr_Dup: rowval"));

  /* copy the col structure */
  if (mat->colptr)
    nmat->colptr = gk_zcopy(mat->ncols+1, mat->colptr, 
                            gk_zmalloc(mat->ncols+1, "gk_csr_Dup: colptr"));
  if (mat->colids)
    nmat->colids = gk_icopy(mat->ncols, mat->colids, 
                            gk_imalloc(mat->ncols, "gk_csr_Dup: colids"));
  if (mat->cnorms)
    nmat->cnorms = gk_fcopy(mat->ncols, mat->cnorms, 
                            gk_fmalloc(mat->ncols, "gk_csr_Dup: cnorms"));
  if (mat->colind)
    nmat->colind = gk_icopy(mat->colptr[mat->ncols], mat->colind, 
                            gk_imalloc(mat->colptr[mat->ncols], "gk_csr_Dup: colind"));
  if (mat->colval)
    nmat->colval = gk_fcopy(mat->colptr[mat->ncols], mat->colval, 
                            gk_fmalloc(mat->colptr[mat->ncols], "gk_csr_Dup: colval"));

  return nmat;
}


/*************************************************************************/
/*! Returns a submatrix containint a set of consecutive rows.
    \param mat is the original matrix.
    \param rstart is the starting row.
    \param nrows is the number of rows from rstart to extract.
    \returns the row structure of the newly created submatrix.
*/
/**************************************************************************/
gk_csr_t *gk_csr_ExtractSubmatrix(gk_csr_t *mat, int rstart, int nrows)
{
  ssize_t i;
  gk_csr_t *nmat;

  if (rstart+nrows > mat->nrows)
    return NULL;

  nmat = gk_csr_Create();

  nmat->nrows  = nrows;
  nmat->ncols  = mat->ncols;

  /* copy the row structure */
  if (mat->rowptr)
    nmat->rowptr = gk_zcopy(nrows+1, mat->rowptr+rstart, 
                              gk_zmalloc(nrows+1, "gk_csr_ExtractSubmatrix: rowptr"));
  for (i=nrows; i>=0; i--)
    nmat->rowptr[i] -= nmat->rowptr[0];
  ASSERT(nmat->rowptr[0] == 0);

  if (mat->rowids)
    nmat->rowids = gk_icopy(nrows, mat->rowids+rstart, 
                            gk_imalloc(nrows, "gk_csr_ExtractSubmatrix: rowids"));
  if (mat->rnorms)
    nmat->rnorms = gk_fcopy(nrows, mat->rnorms+rstart, 
                            gk_fmalloc(nrows, "gk_csr_ExtractSubmatrix: rnorms"));

  if (mat->rsums)
    nmat->rsums = gk_fcopy(nrows, mat->rsums+rstart, 
                            gk_fmalloc(nrows, "gk_csr_ExtractSubmatrix: rsums"));

  ASSERT(nmat->rowptr[nrows] == mat->rowptr[rstart+nrows]-mat->rowptr[rstart]);
  if (mat->rowind)
    nmat->rowind = gk_icopy(mat->rowptr[rstart+nrows]-mat->rowptr[rstart], 
                            mat->rowind+mat->rowptr[rstart], 
                            gk_imalloc(mat->rowptr[rstart+nrows]-mat->rowptr[rstart],
                                       "gk_csr_ExtractSubmatrix: rowind"));
  if (mat->rowval)
    nmat->rowval = gk_fcopy(mat->rowptr[rstart+nrows]-mat->rowptr[rstart], 
                            mat->rowval+mat->rowptr[rstart], 
                            gk_fmalloc(mat->rowptr[rstart+nrows]-mat->rowptr[rstart],
                                       "gk_csr_ExtractSubmatrix: rowval"));

  return nmat;
}


/*************************************************************************/
/*! Returns a submatrix containing a certain set of rows.
    \param mat is the original matrix.
    \param nrows is the number of rows to extract.
    \param rind is the set of row numbers to extract.
    \returns the row structure of the newly created submatrix.
*/
/**************************************************************************/
gk_csr_t *gk_csr_ExtractRows(gk_csr_t *mat, int nrows, int *rind)
{
  ssize_t i, ii, j, nnz;
  gk_csr_t *nmat;

  nmat = gk_csr_Create();

  nmat->nrows = nrows;
  nmat->ncols = mat->ncols;

  for (nnz=0, i=0; i<nrows; i++)  
    nnz += mat->rowptr[rind[i]+1]-mat->rowptr[rind[i]];

  nmat->rowptr = gk_zmalloc(nmat->nrows+1, "gk_csr_ExtractPartition: rowptr");
  nmat->rowind = gk_imalloc(nnz, "gk_csr_ExtractPartition: rowind");
  nmat->rowval = gk_fmalloc(nnz, "gk_csr_ExtractPartition: rowval");

  nmat->rowptr[0] = 0;
  for (nnz=0, j=0, ii=0; ii<nrows; ii++) {
    i = rind[ii];
    gk_icopy(mat->rowptr[i+1]-mat->rowptr[i], mat->rowind+mat->rowptr[i], nmat->rowind+nnz);
    gk_fcopy(mat->rowptr[i+1]-mat->rowptr[i], mat->rowval+mat->rowptr[i], nmat->rowval+nnz);
    nnz += mat->rowptr[i+1]-mat->rowptr[i];
    nmat->rowptr[++j] = nnz;
  }
  ASSERT(j == nmat->nrows);

  return nmat;
}


/*************************************************************************/
/*! Returns a submatrix corresponding to a specified partitioning of rows.
    \param mat is the original matrix.
    \param part is the partitioning vector of the rows.
    \param pid is the partition ID that will be extracted.
    \returns the row structure of the newly created submatrix.
*/
/**************************************************************************/
gk_csr_t *gk_csr_ExtractPartition(gk_csr_t *mat, int *part, int pid)
{
  ssize_t i, j, nnz;
  gk_csr_t *nmat;

  nmat = gk_csr_Create();

  nmat->nrows = 0;
  nmat->ncols = mat->ncols;

  for (nnz=0, i=0; i<mat->nrows; i++) {
    if (part[i] == pid) {
      nmat->nrows++;
      nnz += mat->rowptr[i+1]-mat->rowptr[i];
    }
  }

  nmat->rowptr = gk_zmalloc(nmat->nrows+1, "gk_csr_ExtractPartition: rowptr");
  nmat->rowind = gk_imalloc(nnz, "gk_csr_ExtractPartition: rowind");
  nmat->rowval = gk_fmalloc(nnz, "gk_csr_ExtractPartition: rowval");

  nmat->rowptr[0] = 0;
  for (nnz=0, j=0, i=0; i<mat->nrows; i++) {
    if (part[i] == pid) {
      gk_icopy(mat->rowptr[i+1]-mat->rowptr[i], mat->rowind+mat->rowptr[i], nmat->rowind+nnz);
      gk_fcopy(mat->rowptr[i+1]-mat->rowptr[i], mat->rowval+mat->rowptr[i], nmat->rowval+nnz);
      nnz += mat->rowptr[i+1]-mat->rowptr[i];
      nmat->rowptr[++j] = nnz;
    }
  }
  ASSERT(j == nmat->nrows);

  return nmat;
}


/*************************************************************************/
/*! Splits the matrix into multiple sub-matrices based on the provided
    color array.
    \param mat is the original matrix.
    \param color is an array of size equal to the number of non-zeros
           in the matrix (row-wise structure). The matrix is split into
           as many parts as the number of colors. For meaningfull results,
           the colors should be numbered consecutively starting from 0.
    \returns an array of matrices for each supplied color number.
*/
/**************************************************************************/
gk_csr_t **gk_csr_Split(gk_csr_t *mat, int *color)
{
  ssize_t i, j;
  int nrows, ncolors;
  ssize_t *rowptr;
  int *rowind;
  float *rowval;
  gk_csr_t **smats;

  nrows  = mat->nrows;
  rowptr = mat->rowptr;
  rowind = mat->rowind;
  rowval = mat->rowval;

  ncolors = gk_imax(rowptr[nrows], color)+1;

  smats = (gk_csr_t **)gk_malloc(sizeof(gk_csr_t *)*ncolors, "gk_csr_Split: smats");
  for (i=0; i<ncolors; i++) {
    smats[i] = gk_csr_Create();
    smats[i]->nrows  = mat->nrows;
    smats[i]->ncols  = mat->ncols;
    smats[i]->rowptr = gk_zsmalloc(nrows+1, 0, "gk_csr_Split: smats[i]->rowptr"); 
  }

  for (i=0; i<nrows; i++) {
    for (j=rowptr[i]; j<rowptr[i+1]; j++) 
      smats[color[j]]->rowptr[i]++;
  }
  for (i=0; i<ncolors; i++) 
    MAKECSR(j, nrows, smats[i]->rowptr);

  for (i=0; i<ncolors; i++) {
    smats[i]->rowind = gk_imalloc(smats[i]->rowptr[nrows], "gk_csr_Split: smats[i]->rowind"); 
    smats[i]->rowval = gk_fmalloc(smats[i]->rowptr[nrows], "gk_csr_Split: smats[i]->rowval"); 
  }

  for (i=0; i<nrows; i++) {
    for (j=rowptr[i]; j<rowptr[i+1]; j++) {
      smats[color[j]]->rowind[smats[color[j]]->rowptr[i]] = rowind[j];
      smats[color[j]]->rowval[smats[color[j]]->rowptr[i]] = rowval[j];
      smats[color[j]]->rowptr[i]++;
    }
  }

  for (i=0; i<ncolors; i++) 
    SHIFTCSR(j, nrows, smats[i]->rowptr);

  return smats;
}


/**************************************************************************/
/*! Reads a CSR matrix from the supplied file and stores it the matrix's 
    forward structure.
    \param filename is the file that stores the data.
    \param format is either GK_CSR_FMT_METIS, GK_CSR_FMT_CLUTO, 
           GK_CSR_FMT_CSR, GK_CSR_FMT_BINROW, GK_CSR_FMT_BINCOL 
           specifying the type of the input format. 
           The GK_CSR_FMT_CSR does not contain a header
           line, whereas the GK_CSR_FMT_BINROW is a binary format written 
           by gk_csr_Write() using the same format specifier.
    \param readvals is either 1 or 0, indicating if the CSR file contains
           values or it does not. It only applies when GK_CSR_FMT_CSR is
           used.
    \param numbering is either 1 or 0, indicating if the numbering of the 
           indices start from 1 or 0, respectively. If they start from 1, 
           they are automatically decreamented during input so that they
           will start from 0. It only applies when GK_CSR_FMT_CSR is
           used.
    \returns the matrix that was read.
*/
/**************************************************************************/
gk_csr_t *gk_csr_Read(char *filename, int format, int readvals, int numbering)
{
  ssize_t i, k, l;
  size_t nfields, nrows, ncols, nnz, fmt, ncon;
  size_t lnlen;
  ssize_t *rowptr;
  int *rowind, ival;
  float *rowval=NULL, fval;
  int readsizes, readwgts;
  char *line=NULL, *head, *tail, fmtstr[256];
  FILE *fpin;
  gk_csr_t *mat=NULL;


  if (!gk_fexists(filename)) 
    gk_errexit(SIGERR, "File %s does not exist!\n", filename);

  if (format == GK_CSR_FMT_BINROW) {
    mat = gk_csr_Create();

    fpin = gk_fopen(filename, "rb", "gk_csr_Read: fpin");
    if (fread(&(mat->nrows), sizeof(int32_t), 1, fpin) != 1)
      gk_errexit(SIGERR, "Failed to read the nrows from file %s!\n", filename);
    if (fread(&(mat->ncols), sizeof(int32_t), 1, fpin) != 1)
      gk_errexit(SIGERR, "Failed to read the ncols from file %s!\n", filename);
    mat->rowptr = gk_zmalloc(mat->nrows+1, "gk_csr_Read: rowptr");
    if (fread(mat->rowptr, sizeof(ssize_t), mat->nrows+1, fpin) != mat->nrows+1)
      gk_errexit(SIGERR, "Failed to read the rowptr from file %s!\n", filename);
    mat->rowind = gk_imalloc(mat->rowptr[mat->nrows], "gk_csr_Read: rowind");
    if (fread(mat->rowind, sizeof(int32_t), mat->rowptr[mat->nrows], fpin) != mat->rowptr[mat->nrows])
      gk_errexit(SIGERR, "Failed to read the rowind from file %s!\n", filename);
    if (readvals == 1) {
      mat->rowval = gk_fmalloc(mat->rowptr[mat->nrows], "gk_csr_Read: rowval");
      if (fread(mat->rowval, sizeof(float), mat->rowptr[mat->nrows], fpin) != mat->rowptr[mat->nrows])
        gk_errexit(SIGERR, "Failed to read the rowval from file %s!\n", filename);
    }

    gk_fclose(fpin);
    return mat;
  }

  if (format == GK_CSR_FMT_BINCOL) {
    mat = gk_csr_Create();

    fpin = gk_fopen(filename, "rb", "gk_csr_Read: fpin");
    if (fread(&(mat->nrows), sizeof(int32_t), 1, fpin) != 1)
      gk_errexit(SIGERR, "Failed to read the nrows from file %s!\n", filename);
    if (fread(&(mat->ncols), sizeof(int32_t), 1, fpin) != 1)
      gk_errexit(SIGERR, "Failed to read the ncols from file %s!\n", filename);
    mat->colptr = gk_zmalloc(mat->ncols+1, "gk_csr_Read: colptr");
    if (fread(mat->colptr, sizeof(ssize_t), mat->ncols+1, fpin) != mat->ncols+1)
      gk_errexit(SIGERR, "Failed to read the colptr from file %s!\n", filename);
    mat->colind = gk_imalloc(mat->colptr[mat->ncols], "gk_csr_Read: colind");
    if (fread(mat->colind, sizeof(int32_t), mat->colptr[mat->ncols], fpin) != mat->colptr[mat->ncols])
      gk_errexit(SIGERR, "Failed to read the colind from file %s!\n", filename);
    if (readvals) {
      mat->colval = gk_fmalloc(mat->colptr[mat->ncols], "gk_csr_Read: colval");
      if (fread(mat->colval, sizeof(float), mat->colptr[mat->ncols], fpin) != mat->colptr[mat->ncols])
        gk_errexit(SIGERR, "Failed to read the colval from file %s!\n", filename);
    }

    gk_fclose(fpin);
    return mat;
  }


  if (format == GK_CSR_FMT_CLUTO) {
    fpin = gk_fopen(filename, "r", "gk_csr_Read: fpin");
    do {
      if (gk_getline(&line, &lnlen, fpin) <= 0)
        gk_errexit(SIGERR, "Premature end of input file: file:%s\n", filename);
    } while (line[0] == '%');

    if (sscanf(line, "%zu %zu %zu", &nrows, &ncols, &nnz) != 3)
      gk_errexit(SIGERR, "Header line must contain 3 integers.\n");

    readsizes = 0;
    readwgts  = 0;
    readvals  = 1;
    numbering = 1;
  }
  else if (format == GK_CSR_FMT_METIS) {
    fpin = gk_fopen(filename, "r", "gk_csr_Read: fpin");
    do {
      if (gk_getline(&line, &lnlen, fpin) <= 0)
        gk_errexit(SIGERR, "Premature end of input file: file:%s\n", filename);
    } while (line[0] == '%');

    fmt = ncon = 0;
    nfields = sscanf(line, "%zu %zu %zu %zu", &nrows, &nnz, &fmt, &ncon);
    if (nfields < 2)
      gk_errexit(SIGERR, "Header line must contain at least 2 integers (#vtxs and #edges).\n");

    ncols = nrows;
    nnz *= 2;

    if (fmt > 111)
      gk_errexit(SIGERR, "Cannot read this type of file format [fmt=%zu]!\n", fmt);

    sprintf(fmtstr, "%03zu", fmt%1000);
    readsizes = (fmtstr[0] == '1');
    readwgts  = (fmtstr[1] == '1');
    readvals  = (fmtstr[2] == '1');
    numbering = 1;
    ncon      = (ncon == 0 ? 1 : ncon);
  }
  else {
    readsizes = 0;
    readwgts  = 0;

    gk_getfilestats(filename, &nrows, &nnz, NULL, NULL);

    if (readvals == 1 && nnz%2 == 1)
      gk_errexit(SIGERR, "Error: The number of numbers (%zd %d) in the input file is not even.\n", nnz, readvals);
    if (readvals == 1)
      nnz = nnz/2;
    fpin = gk_fopen(filename, "r", "gk_csr_Read: fpin");
  }

  mat = gk_csr_Create();

  mat->nrows = nrows;

  rowptr = mat->rowptr = gk_zmalloc(nrows+1, "gk_csr_Read: rowptr");
  rowind = mat->rowind = gk_imalloc(nnz, "gk_csr_Read: rowind");
  if (readvals != 2)
    rowval = mat->rowval = gk_fsmalloc(nnz, 1.0, "gk_csr_Read: rowval");

  if (readsizes)
    mat->rsizes = gk_fsmalloc(nrows, 0.0, "gk_csr_Read: rsizes");

  if (readwgts)
    mat->rwgts = gk_fsmalloc(nrows*ncon, 0.0, "gk_csr_Read: rwgts");

  /*----------------------------------------------------------------------
   * Read the sparse matrix file
   *---------------------------------------------------------------------*/
  numbering = (numbering ? - 1 : 0);
  for (ncols=0, rowptr[0]=0, k=0, i=0; i<nrows; i++) {
    do {
      if (gk_getline(&line, &lnlen, fpin) == -1)
        gk_errexit(SIGERR, "Premature end of input file: file while reading row %d\n", i);
    } while (line[0] == '%');

    head = line;
    tail = NULL;

    /* Read vertex sizes */
    if (readsizes) {
#ifdef __MSC__
      mat->rsizes[i] = (float)strtod(head, &tail);
#else
      mat->rsizes[i] = strtof(head, &tail);
#endif
      if (tail == head)
        gk_errexit(SIGERR, "The line for vertex %zd does not have size information\n", i+1);
      if (mat->rsizes[i] < 0)
        errexit("The size for vertex %zd must be >= 0\n", i+1);
      head = tail;
    }

    /* Read vertex weights */
    if (readwgts) {
      for (l=0; l<ncon; l++) {
#ifdef __MSC__
        mat->rwgts[i*ncon+l] = (float)strtod(head, &tail);
#else
        mat->rwgts[i*ncon+l] = strtof(head, &tail);
#endif
        if (tail == head)
          errexit("The line for vertex %zd does not have enough weights "
                  "for the %d constraints.\n", i+1, ncon);
        if (mat->rwgts[i*ncon+l] < 0)
          errexit("The weight vertex %zd and constraint %zd must be >= 0\n", i+1, l);
        head = tail;
      }
    }

   
    /* Read the rest of the row */
    while (1) {
      ival = (int)strtol(head, &tail, 0);
      if (tail == head) 
        break;
      head = tail;
      
      if ((rowind[k] = ival + numbering) < 0)
        gk_errexit(SIGERR, "Error: Invalid column number %d at row %zd.\n", ival, i);

      ncols = gk_max(rowind[k], ncols);

      if (readvals == 1) {
#ifdef __MSC__
        fval = (float)strtod(head, &tail);
#else
	fval = strtof(head, &tail);
#endif
        if (tail == head)
          gk_errexit(SIGERR, "Value could not be found for column! Row:%zd, NNZ:%zd\n", i, k);
        head = tail;

        rowval[k] = fval;
      }
      k++;
    }
    rowptr[i+1] = k;
  }

  if (format == GK_CSR_FMT_METIS) {
    ASSERT(ncols+1 == mat->nrows);
    mat->ncols = mat->nrows;
  }
  else {
    mat->ncols = ncols+1;
  }

  if (k != nnz)
    gk_errexit(SIGERR, "gk_csr_Read: Something wrong with the number of nonzeros in "
                       "the input file. NNZ=%zd, ActualNNZ=%zd.\n", nnz, k);

  gk_fclose(fpin);

  gk_free((void **)&line, LTERM);

  return mat;
}


/**************************************************************************/
/*! Writes the row-based structure of a matrix into a file.
    \param mat is the matrix to be written,
    \param filename is the name of the output file.
    \param format is one of: GK_CSR_FMT_CLUTO, GK_CSR_FMT_CSR, 
           GK_CSR_FMT_BINROW, GK_CSR_FMT_BINCOL.
    \param writevals is either 1 or 0 indicating if the values will be 
           written or not. This is only applicable when GK_CSR_FMT_CSR
           is used.
    \param numbering is either 1 or 0 indicating if the internal 0-based 
           numbering will be shifted by one or not during output. This 
           is only applicable when GK_CSR_FMT_CSR is used.
*/
/**************************************************************************/
void gk_csr_Write(gk_csr_t *mat, char *filename, int format, int writevals, int numbering)
{
  ssize_t i, j;
  FILE *fpout;

  if (format == GK_CSR_FMT_BINROW) {
    if (filename == NULL)
      gk_errexit(SIGERR, "The filename parameter cannot be NULL.\n");
    fpout = gk_fopen(filename, "wb", "gk_csr_Write: fpout");

    fwrite(&(mat->nrows), sizeof(int32_t), 1, fpout); 
    fwrite(&(mat->ncols), sizeof(int32_t), 1, fpout); 
    fwrite(mat->rowptr, sizeof(ssize_t), mat->nrows+1, fpout); 
    fwrite(mat->rowind, sizeof(int32_t), mat->rowptr[mat->nrows], fpout); 
    if (writevals)
      fwrite(mat->rowval, sizeof(float), mat->rowptr[mat->nrows], fpout); 

    gk_fclose(fpout);
    return;
  }

  if (format == GK_CSR_FMT_BINCOL) {
    if (filename == NULL)
      gk_errexit(SIGERR, "The filename parameter cannot be NULL.\n");
    fpout = gk_fopen(filename, "wb", "gk_csr_Write: fpout");

    fwrite(&(mat->nrows), sizeof(int32_t), 1, fpout); 
    fwrite(&(mat->ncols), sizeof(int32_t), 1, fpout); 
    fwrite(mat->colptr, sizeof(ssize_t), mat->ncols+1, fpout); 
    fwrite(mat->colind, sizeof(int32_t), mat->colptr[mat->ncols], fpout); 
    if (writevals) 
      fwrite(mat->colval, sizeof(float), mat->colptr[mat->ncols], fpout); 

    gk_fclose(fpout);
    return;
  }

  if (filename)
    fpout = gk_fopen(filename, "w", "gk_csr_Write: fpout");
  else
    fpout = stdout; 

  if (format == GK_CSR_FMT_CLUTO) {
    fprintf(fpout, "%d %d %zd\n", mat->nrows, mat->ncols, mat->rowptr[mat->nrows]);
    writevals = 1;
    numbering = 1;
  }

  for (i=0; i<mat->nrows; i++) {
    for (j=mat->rowptr[i]; j<mat->rowptr[i+1]; j++) {
      fprintf(fpout, " %d", mat->rowind[j]+(numbering ? 1 : 0));
      if (writevals) 
        fprintf(fpout, " %f", mat->rowval[j]);
    }
    fprintf(fpout, "\n");
  }
  if (filename)
    gk_fclose(fpout);
}


/*************************************************************************/
/*! Prunes certain rows/columns of the matrix. The prunning takes place 
    by analyzing the row structure of the matrix. The prunning takes place
    by removing rows/columns but it does not affect the numbering of the
    remaining rows/columns.
   
    \param mat the matrix to be prunned,
    \param what indicates if the rows (GK_CSR_ROW) or the columns (GK_CSR_COL)
           of the matrix will be prunned,
    \param minf is the minimum number of rows (columns) that a column (row) must
           be present in order to be kept,
    \param maxf is the maximum number of rows (columns) that a column (row) must
          be present at in order to be kept.
    \returns the prunned matrix consisting only of its row-based structure. 
          The input matrix is not modified. 
*/
/**************************************************************************/
gk_csr_t *gk_csr_Prune(gk_csr_t *mat, int what, int minf, int maxf)
{
  ssize_t i, j, nnz;
  int nrows, ncols;
  ssize_t *rowptr, *nrowptr;
  int *rowind, *nrowind, *collen;
  float *rowval, *nrowval;
  gk_csr_t *nmat;

  nmat = gk_csr_Create();
  
  nrows = nmat->nrows = mat->nrows;
  ncols = nmat->ncols = mat->ncols;

  rowptr = mat->rowptr;
  rowind = mat->rowind;
  rowval = mat->rowval;

  nrowptr = nmat->rowptr = gk_zmalloc(nrows+1, "gk_csr_Prune: nrowptr");
  nrowind = nmat->rowind = gk_imalloc(rowptr[nrows], "gk_csr_Prune: nrowind");
  nrowval = nmat->rowval = gk_fmalloc(rowptr[nrows], "gk_csr_Prune: nrowval");


  switch (what) {
    case GK_CSR_COL:
      collen = gk_ismalloc(ncols, 0, "gk_csr_Prune: collen");

      for (i=0; i<nrows; i++) {
        for (j=rowptr[i]; j<rowptr[i+1]; j++) {
          ASSERT(rowind[j] < ncols);
          collen[rowind[j]]++;
        }
      }
      for (i=0; i<ncols; i++)
        collen[i] = (collen[i] >= minf && collen[i] <= maxf ? 1 : 0);

      nrowptr[0] = 0;
      for (nnz=0, i=0; i<nrows; i++) {
        for (j=rowptr[i]; j<rowptr[i+1]; j++) {
          if (collen[rowind[j]]) {
            nrowind[nnz] = rowind[j];
            nrowval[nnz] = rowval[j];
            nnz++;
          }
        }
        nrowptr[i+1] = nnz;
      }
      gk_free((void **)&collen, LTERM);
      break;

    case GK_CSR_ROW:
      nrowptr[0] = 0;
      for (nnz=0, i=0; i<nrows; i++) {
        if (rowptr[i+1]-rowptr[i] >= minf && rowptr[i+1]-rowptr[i] <= maxf) {
          for (j=rowptr[i]; j<rowptr[i+1]; j++, nnz++) {
            nrowind[nnz] = rowind[j];
            nrowval[nnz] = rowval[j];
          }
        }
        nrowptr[i+1] = nnz;
      }
      break;

    default:
      gk_csr_Free(&nmat);
      gk_errexit(SIGERR, "Unknown prunning type of %d\n", what);
      return NULL;
  }

  return nmat;
}


/*************************************************************************/
/*! Eliminates certain entries from the rows/columns of the matrix. The 
    filtering takes place by keeping only the highest weight entries whose
    sum accounts for a certain fraction of the overall weight of the 
    row/column.
   
    \param mat the matrix to be prunned,
    \param what indicates if the rows (GK_CSR_ROW) or the columns (GK_CSR_COL)
           of the matrix will be prunned,
    \param norm indicates the norm that will be used to aggregate the weights
           and possible values are 1 or 2,
    \param fraction is the fraction of the overall norm that will be retained
           by the kept entries.
    \returns the filtered matrix consisting only of its row-based structure. 
           The input matrix is not modified. 
*/
/**************************************************************************/
gk_csr_t *gk_csr_LowFilter(gk_csr_t *mat, int what, int norm, float fraction)
{
  ssize_t i, j, nnz;
  int nrows, ncols, ncand, maxlen=0;
  ssize_t *rowptr, *colptr, *nrowptr;
  int *rowind, *colind, *nrowind;
  float *rowval, *colval, *nrowval, rsum, tsum;
  gk_csr_t *nmat;
  gk_fkv_t *cand;

  nmat = gk_csr_Create();
  
  nrows = nmat->nrows = mat->nrows;
  ncols = nmat->ncols = mat->ncols;

  rowptr = mat->rowptr;
  rowind = mat->rowind;
  rowval = mat->rowval;
  colptr = mat->colptr;
  colind = mat->colind;
  colval = mat->colval;

  nrowptr = nmat->rowptr = gk_zmalloc(nrows+1, "gk_csr_LowFilter: nrowptr");
  nrowind = nmat->rowind = gk_imalloc(rowptr[nrows], "gk_csr_LowFilter: nrowind");
  nrowval = nmat->rowval = gk_fmalloc(rowptr[nrows], "gk_csr_LowFilter: nrowval");


  switch (what) {
    case GK_CSR_COL:
      if (mat->colptr == NULL) 
        gk_errexit(SIGERR, "Cannot filter columns when column-based structure has not been created.\n");

      gk_zcopy(nrows+1, rowptr, nrowptr);

      for (i=0; i<ncols; i++) 
        maxlen = gk_max(maxlen, colptr[i+1]-colptr[i]);

      #pragma omp parallel private(i, j, ncand, rsum, tsum, cand)
      {
        cand = gk_fkvmalloc(maxlen, "gk_csr_LowFilter: cand");

        #pragma omp for schedule(static)
        for (i=0; i<ncols; i++) {
          for (tsum=0.0, ncand=0, j=colptr[i]; j<colptr[i+1]; j++, ncand++) {
            cand[ncand].val = colind[j];
            cand[ncand].key = colval[j];
            tsum += (norm == 1 ? colval[j] : colval[j]*colval[j]);
          }
          gk_fkvsortd(ncand, cand);

          for (rsum=0.0, j=0; j<ncand && rsum<=fraction*tsum; j++) {
            rsum += (norm == 1 ? cand[j].key : cand[j].key*cand[j].key);
            nrowind[nrowptr[cand[j].val]] = i;
            nrowval[nrowptr[cand[j].val]] = cand[j].key;
            nrowptr[cand[j].val]++;
          }
        }

        gk_free((void **)&cand, LTERM);
      }

      /* compact the nrowind/nrowval */
      for (nnz=0, i=0; i<nrows; i++) {
        for (j=rowptr[i]; j<nrowptr[i]; j++, nnz++) {
          nrowind[nnz] = nrowind[j];
          nrowval[nnz] = nrowval[j];
        }
        nrowptr[i] = nnz;
      }
      SHIFTCSR(i, nrows, nrowptr);

      break;

    case GK_CSR_ROW:
      if (mat->rowptr == NULL) 
        gk_errexit(SIGERR, "Cannot filter rows when row-based structure has not been created.\n");

      for (i=0; i<nrows; i++) 
        maxlen = gk_max(maxlen, rowptr[i+1]-rowptr[i]);

      #pragma omp parallel private(i, j, ncand, rsum, tsum, cand)
      {
        cand = gk_fkvmalloc(maxlen, "gk_csr_LowFilter: cand");

        #pragma omp for schedule(static)
        for (i=0; i<nrows; i++) {
          for (tsum=0.0, ncand=0, j=rowptr[i]; j<rowptr[i+1]; j++, ncand++) {
            cand[ncand].val = rowind[j];
            cand[ncand].key = rowval[j];
            tsum += (norm == 1 ? rowval[j] : rowval[j]*rowval[j]);
          }
          gk_fkvsortd(ncand, cand);

          for (rsum=0.0, j=0; j<ncand && rsum<=fraction*tsum; j++) {
            rsum += (norm == 1 ? cand[j].key : cand[j].key*cand[j].key);
            nrowind[rowptr[i]+j] = cand[j].val;
            nrowval[rowptr[i]+j] = cand[j].key;
          }
          nrowptr[i+1] = rowptr[i]+j;
        }

        gk_free((void **)&cand, LTERM);
      }

      /* compact nrowind/nrowval */
      nrowptr[0] = nnz = 0;
      for (i=0; i<nrows; i++) {
        for (j=rowptr[i]; j<nrowptr[i+1]; j++, nnz++) {
          nrowind[nnz] = nrowind[j];
          nrowval[nnz] = nrowval[j];
        }
        nrowptr[i+1] = nnz;
      }

      break;

    default:
      gk_csr_Free(&nmat);
      gk_errexit(SIGERR, "Unknown prunning type of %d\n", what);
      return NULL;
  }

  return nmat;
}


/*************************************************************************/
/*! Eliminates certain entries from the rows/columns of the matrix. The 
    filtering takes place by keeping only the highest weight top-K entries 
    along each row/column and those entries whose weight is greater than
    a specified value.
   
    \param mat the matrix to be prunned,
    \param what indicates if the rows (GK_CSR_ROW) or the columns (GK_CSR_COL)
           of the matrix will be prunned,
    \param topk is the number of the highest weight entries to keep.
    \param keepval is the weight of a term above which will be kept. This
           is used to select additional terms past the first topk.
    \returns the filtered matrix consisting only of its row-based structure. 
           The input matrix is not modified. 
*/
/**************************************************************************/
gk_csr_t *gk_csr_TopKPlusFilter(gk_csr_t *mat, int what, int topk, float keepval)
{
  ssize_t i, j, k, nnz;
  int nrows, ncols, ncand;
  ssize_t *rowptr, *colptr, *nrowptr;
  int *rowind, *colind, *nrowind;
  float *rowval, *colval, *nrowval;
  gk_csr_t *nmat;
  gk_fkv_t *cand;

  nmat = gk_csr_Create();
  
  nrows = nmat->nrows = mat->nrows;
  ncols = nmat->ncols = mat->ncols;

  rowptr = mat->rowptr;
  rowind = mat->rowind;
  rowval = mat->rowval;
  colptr = mat->colptr;
  colind = mat->colind;
  colval = mat->colval;

  nrowptr = nmat->rowptr = gk_zmalloc(nrows+1, "gk_csr_LowFilter: nrowptr");
  nrowind = nmat->rowind = gk_imalloc(rowptr[nrows], "gk_csr_LowFilter: nrowind");
  nrowval = nmat->rowval = gk_fmalloc(rowptr[nrows], "gk_csr_LowFilter: nrowval");


  switch (what) {
    case GK_CSR_COL:
      if (mat->colptr == NULL) 
        gk_errexit(SIGERR, "Cannot filter columns when column-based structure has not been created.\n");

      cand = gk_fkvmalloc(nrows, "gk_csr_LowFilter: cand");

      gk_zcopy(nrows+1, rowptr, nrowptr);
      for (i=0; i<ncols; i++) {
        for (ncand=0, j=colptr[i]; j<colptr[i+1]; j++, ncand++) {
          cand[ncand].val = colind[j];
          cand[ncand].key = colval[j];
        }
        gk_fkvsortd(ncand, cand);

        k = gk_min(topk, ncand);
        for (j=0; j<k; j++) {
          nrowind[nrowptr[cand[j].val]] = i;
          nrowval[nrowptr[cand[j].val]] = cand[j].key;
          nrowptr[cand[j].val]++;
        }
        for (; j<ncand; j++) {
          if (cand[j].key < keepval) 
            break;

          nrowind[nrowptr[cand[j].val]] = i;
          nrowval[nrowptr[cand[j].val]] = cand[j].key;
          nrowptr[cand[j].val]++;
        }
      }

      /* compact the nrowind/nrowval */
      for (nnz=0, i=0; i<nrows; i++) {
        for (j=rowptr[i]; j<nrowptr[i]; j++, nnz++) {
          nrowind[nnz] = nrowind[j];
          nrowval[nnz] = nrowval[j];
        }
        nrowptr[i] = nnz;
      }
      SHIFTCSR(i, nrows, nrowptr);

      gk_free((void **)&cand, LTERM);
      break;

    case GK_CSR_ROW:
      if (mat->rowptr == NULL) 
        gk_errexit(SIGERR, "Cannot filter rows when row-based structure has not been created.\n");

      cand = gk_fkvmalloc(ncols, "gk_csr_LowFilter: cand");

      nrowptr[0] = 0;
      for (nnz=0, i=0; i<nrows; i++) {
        for (ncand=0, j=rowptr[i]; j<rowptr[i+1]; j++, ncand++) {
          cand[ncand].val = rowind[j];
          cand[ncand].key = rowval[j];
        }
        gk_fkvsortd(ncand, cand);

        k = gk_min(topk, ncand);
        for (j=0; j<k; j++, nnz++) {
          nrowind[nnz] = cand[j].val;
          nrowval[nnz] = cand[j].key;
        }
        for (; j<ncand; j++, nnz++) {
          if (cand[j].key < keepval) 
            break;

          nrowind[nnz] = cand[j].val;
          nrowval[nnz] = cand[j].key;
        }
        nrowptr[i+1] = nnz;
      }

      gk_free((void **)&cand, LTERM);
      break;

    default:
      gk_csr_Free(&nmat);
      gk_errexit(SIGERR, "Unknown prunning type of %d\n", what);
      return NULL;
  }

  return nmat;
}


/*************************************************************************/
/*! Eliminates certain entries from the rows/columns of the matrix. The 
    filtering takes place by keeping only the terms whose contribution to
    the total length of the document is greater than a user-splied multiple
    over the average.

    This routine assumes that the vectors are normalized to be unit length.
   
    \param mat the matrix to be prunned,
    \param what indicates if the rows (GK_CSR_ROW) or the columns (GK_CSR_COL)
           of the matrix will be prunned,
    \param zscore is the multiplicative factor over the average contribution 
           to the length of the document.
    \returns the filtered matrix consisting only of its row-based structure. 
           The input matrix is not modified. 
*/
/**************************************************************************/
gk_csr_t *gk_csr_ZScoreFilter(gk_csr_t *mat, int what, float zscore)
{
  ssize_t i, j, nnz;
  int nrows;
  ssize_t *rowptr, *nrowptr;
  int *rowind, *nrowind;
  float *rowval, *nrowval, avgwgt;
  gk_csr_t *nmat;

  nmat = gk_csr_Create();
  
  nmat->nrows = mat->nrows;
  nmat->ncols = mat->ncols;

  nrows  = mat->nrows; 
  rowptr = mat->rowptr;
  rowind = mat->rowind;
  rowval = mat->rowval;

  nrowptr = nmat->rowptr = gk_zmalloc(nrows+1, "gk_csr_ZScoreFilter: nrowptr");
  nrowind = nmat->rowind = gk_imalloc(rowptr[nrows], "gk_csr_ZScoreFilter: nrowind");
  nrowval = nmat->rowval = gk_fmalloc(rowptr[nrows], "gk_csr_ZScoreFilter: nrowval");


  switch (what) {
    case GK_CSR_COL:
      gk_errexit(SIGERR, "This has not been implemented yet.\n");
      break;

    case GK_CSR_ROW:
      if (mat->rowptr == NULL) 
        gk_errexit(SIGERR, "Cannot filter rows when row-based structure has not been created.\n");

      nrowptr[0] = 0;
      for (nnz=0, i=0; i<nrows; i++) {
        avgwgt = zscore/(rowptr[i+1]-rowptr[i]);
        for (j=rowptr[i]; j<rowptr[i+1]; j++) {
          if (rowval[j] > avgwgt) {
            nrowind[nnz] = rowind[j];
            nrowval[nnz] = rowval[j];
            nnz++;
          }
        }
        nrowptr[i+1] = nnz;
      }
      break;

    default:
      gk_csr_Free(&nmat);
      gk_errexit(SIGERR, "Unknown prunning type of %d\n", what);
      return NULL;
  }

  return nmat;
}


/*************************************************************************/
/*! Compacts the column-space of the matrix by removing empty columns.
    As a result of the compaction, the column numbers are renumbered. 
    The compaction operation is done in place and only affects the row-based
    representation of the matrix.
    The new columns are ordered in decreasing frequency.
   
    \param mat the matrix whose empty columns will be removed.
*/
/**************************************************************************/
void gk_csr_CompactColumns(gk_csr_t *mat)
{
  ssize_t i;
  int nrows, ncols, nncols;
  ssize_t *rowptr;
  int *rowind, *colmap;
  gk_ikv_t *clens;

  nrows  = mat->nrows;
  ncols  = mat->ncols;
  rowptr = mat->rowptr;
  rowind = mat->rowind;

  colmap = gk_imalloc(ncols, "gk_csr_CompactColumns: colmap");

  clens = gk_ikvmalloc(ncols, "gk_csr_CompactColumns: clens");
  for (i=0; i<ncols; i++) {
    clens[i].key = 0;
    clens[i].val = i;
  }

  for (i=0; i<rowptr[nrows]; i++) 
    clens[rowind[i]].key++;
  gk_ikvsortd(ncols, clens);

  for (nncols=0, i=0; i<ncols; i++) {
    if (clens[i].key > 0) 
      colmap[clens[i].val] = nncols++;
    else
      break;
  }

  for (i=0; i<rowptr[nrows]; i++) 
    rowind[i] = colmap[rowind[i]];

  mat->ncols = nncols;

  gk_free((void **)&colmap, &clens, LTERM);
}


/*************************************************************************/
/*! Sorts the indices in increasing order
    \param mat the matrix itself,
    \param what is either GK_CSR_ROW or GK_CSR_COL indicating which set of
           indices to sort.
*/
/**************************************************************************/
void gk_csr_SortIndices(gk_csr_t *mat, int what)
{
  int n, nn=0;
  ssize_t *ptr;
  int *ind;
  float *val;

  switch (what) {
    case GK_CSR_ROW:
      if (!mat->rowptr)
        gk_errexit(SIGERR, "Row-based view of the matrix does not exists.\n");

      n   = mat->nrows;
      ptr = mat->rowptr;
      ind = mat->rowind;
      val = mat->rowval;
      break;

    case GK_CSR_COL:
      if (!mat->colptr)
        gk_errexit(SIGERR, "Column-based view of the matrix does not exists.\n");

      n   = mat->ncols;
      ptr = mat->colptr;
      ind = mat->colind;
      val = mat->colval;
      break;

    default:
      gk_errexit(SIGERR, "Invalid index type of %d.\n", what);
      return;
  }

  #pragma omp parallel if (n > 100)
  {
    ssize_t i, j, k;
    gk_ikv_t *cand;
    float *tval;

    #pragma omp single
    for (i=0; i<n; i++) 
      nn = gk_max(nn, ptr[i+1]-ptr[i]);
  
    cand = gk_ikvmalloc(nn, "gk_csr_SortIndices: cand");
    tval = gk_fmalloc(nn, "gk_csr_SortIndices: tval");
  
    #pragma omp for schedule(static)
    for (i=0; i<n; i++) {
      for (k=0, j=ptr[i]; j<ptr[i+1]; j++) {
        if (j > ptr[i] && ind[j] < ind[j-1])
          k = 1; /* an inversion */
        cand[j-ptr[i]].val = j-ptr[i];
        cand[j-ptr[i]].key = ind[j];
        tval[j-ptr[i]]     = val[j];
      }
      if (k) {
        gk_ikvsorti(ptr[i+1]-ptr[i], cand);
        for (j=ptr[i]; j<ptr[i+1]; j++) {
          ind[j] = cand[j-ptr[i]].key;
          val[j] = tval[cand[j-ptr[i]].val];
        }
      }
    }

    gk_free((void **)&cand, &tval, LTERM);
  }

}


/*************************************************************************/
/*! Creates a row/column index from the column/row data.
    \param mat the matrix itself,
    \param what is either GK_CSR_ROW or GK_CSR_COL indicating which index
           will be created.
*/
/**************************************************************************/
void gk_csr_CreateIndex(gk_csr_t *mat, int what)
{
  /* 'f' stands for forward, 'r' stands for reverse */
  ssize_t i, j, k, nf, nr;
  ssize_t *fptr, *rptr;
  int *find, *rind;
  float *fval, *rval;

  switch (what) {
    case GK_CSR_COL:
      nf   = mat->nrows;
      fptr = mat->rowptr;
      find = mat->rowind;
      fval = mat->rowval;

      if (mat->colptr) gk_free((void **)&mat->colptr, LTERM);
      if (mat->colind) gk_free((void **)&mat->colind, LTERM);
      if (mat->colval) gk_free((void **)&mat->colval, LTERM);

      nr   = mat->ncols;
      rptr = mat->colptr = gk_zsmalloc(nr+1, 0, "gk_csr_CreateIndex: rptr");
      rind = mat->colind = gk_imalloc(fptr[nf], "gk_csr_CreateIndex: rind");
      rval = mat->colval = (fval ? gk_fmalloc(fptr[nf], "gk_csr_CreateIndex: rval") : NULL);
      break;
    case GK_CSR_ROW:
      nf   = mat->ncols;
      fptr = mat->colptr;
      find = mat->colind;
      fval = mat->colval;

      if (mat->rowptr) gk_free((void **)&mat->rowptr, LTERM);
      if (mat->rowind) gk_free((void **)&mat->rowind, LTERM);
      if (mat->rowval) gk_free((void **)&mat->rowval, LTERM);

      nr   = mat->nrows;
      rptr = mat->rowptr = gk_zsmalloc(nr+1, 0, "gk_csr_CreateIndex: rptr");
      rind = mat->rowind = gk_imalloc(fptr[nf], "gk_csr_CreateIndex: rind");
      rval = mat->rowval = (fval ? gk_fmalloc(fptr[nf], "gk_csr_CreateIndex: rval") : NULL);
      break;
    default:
      gk_errexit(SIGERR, "Invalid index type of %d.\n", what);
      return;
  }


  for (i=0; i<nf; i++) {
    for (j=fptr[i]; j<fptr[i+1]; j++)
      rptr[find[j]]++;
  }
  MAKECSR(i, nr, rptr);
  
  if (rptr[nr] > 6*nr) {
    for (i=0; i<nf; i++) {
      for (j=fptr[i]; j<fptr[i+1]; j++) 
        rind[rptr[find[j]]++] = i;
    }
    SHIFTCSR(i, nr, rptr);

    if (fval) {
      for (i=0; i<nf; i++) {
        for (j=fptr[i]; j<fptr[i+1]; j++) 
          rval[rptr[find[j]]++] = fval[j];
      }
      SHIFTCSR(i, nr, rptr);
    }
  }
  else {
    if (fval) {
      for (i=0; i<nf; i++) {
        for (j=fptr[i]; j<fptr[i+1]; j++) {
          k = find[j];
          rind[rptr[k]]   = i;
          rval[rptr[k]++] = fval[j];
        }
      }
    }
    else {
      for (i=0; i<nf; i++) {
        for (j=fptr[i]; j<fptr[i+1]; j++) 
          rind[rptr[find[j]]++] = i;
      }
    }
    SHIFTCSR(i, nr, rptr);
  }
}


/*************************************************************************/
/*! Normalizes the rows/columns of the matrix to be unit 
    length.
    \param mat the matrix itself,
    \param what indicates what will be normalized and is obtained by
           specifying GK_CSR_ROW, GK_CSR_COL, GK_CSR_ROW|GK_CSR_COL. 
    \param norm indicates what norm is to normalize to, 1: 1-norm, 2: 2-norm
*/
/**************************************************************************/
void gk_csr_Normalize(gk_csr_t *mat, int what, int norm)
{
  ssize_t i, j;
  int n;
  ssize_t *ptr;
  float *val, sum;

  if (what&GK_CSR_ROW && mat->rowval) {
    n   = mat->nrows;
    ptr = mat->rowptr;
    val = mat->rowval;

    #pragma omp parallel if (ptr[n] > OMPMINOPS) 
    {
      #pragma omp for private(j,sum) schedule(static)
      for (i=0; i<n; i++) {
        for (sum=0.0, j=ptr[i]; j<ptr[i+1]; j++){
  	if (norm == 2)
  	  sum += val[j]*val[j];
  	else if (norm == 1)
  	  sum += val[j]; /* assume val[j] > 0 */ 
        }
        if (sum > 0) {
  	if (norm == 2)
  	  sum=1.0/sqrt(sum); 
  	else if (norm == 1)
  	  sum=1.0/sum; 
          for (j=ptr[i]; j<ptr[i+1]; j++)
            val[j] *= sum;
  	
        }
      }
    }
  }

  if (what&GK_CSR_COL && mat->colval) {
    n   = mat->ncols;
    ptr = mat->colptr;
    val = mat->colval;

    #pragma omp parallel if (ptr[n] > OMPMINOPS)
    {
    #pragma omp for private(j,sum) schedule(static)
      for (i=0; i<n; i++) {
        for (sum=0.0, j=ptr[i]; j<ptr[i+1]; j++)
  	if (norm == 2)
  	  sum += val[j]*val[j];
  	else if (norm == 1)
  	  sum += val[j]; 
        if (sum > 0) {
  	if (norm == 2)
  	  sum=1.0/sqrt(sum); 
  	else if (norm == 1)
  	  sum=1.0/sum; 
          for (j=ptr[i]; j<ptr[i+1]; j++)
            val[j] *= sum;
        }
      }
    }
  }
}


/*************************************************************************/
/*! Applies different row scaling methods.
    \param mat the matrix itself,
    \param type indicates the type of row scaling. Possible values are:
           GK_CSR_MAXTF, GK_CSR_SQRT, GK_CSR_LOG, GK_CSR_IDF, GK_CSR_MAXTF2.
*/
/**************************************************************************/
void gk_csr_Scale(gk_csr_t *mat, int type)
{
  ssize_t i, j;
  int nrows, ncols, nnzcols, bgfreq;
  ssize_t *rowptr;
  int *rowind, *collen;
  float *rowval, *cscale, maxtf;

  nrows  = mat->nrows;
  rowptr = mat->rowptr;
  rowind = mat->rowind;
  rowval = mat->rowval;

  switch (type) {
    case GK_CSR_MAXTF: /* TF' = .5 + .5*TF/MAX(TF) */
      #pragma omp parallel if (rowptr[nrows] > OMPMINOPS)
      {
        #pragma omp for private(j, maxtf) schedule(static)
        for (i=0; i<nrows; i++) {
          maxtf = fabs(rowval[rowptr[i]]);
          for (j=rowptr[i]; j<rowptr[i+1]; j++) 
            maxtf = (maxtf < fabs(rowval[j]) ? fabs(rowval[j]) : maxtf);
  
          for (j=rowptr[i]; j<rowptr[i+1]; j++)
            rowval[j] = .5 + .5*rowval[j]/maxtf;
        }
      }
      break;

    case GK_CSR_MAXTF2: /* TF' = .1 + .9*TF/MAX(TF) */
      #pragma omp parallel if (rowptr[nrows] > OMPMINOPS)
      {
        #pragma omp for private(j, maxtf) schedule(static)
        for (i=0; i<nrows; i++) {
          maxtf = fabs(rowval[rowptr[i]]);
          for (j=rowptr[i]; j<rowptr[i+1]; j++) 
            maxtf = (maxtf < fabs(rowval[j]) ? fabs(rowval[j]) : maxtf);
  
          for (j=rowptr[i]; j<rowptr[i+1]; j++)
            rowval[j] = .1 + .9*rowval[j]/maxtf;
        }
      }
      break;

    case GK_CSR_SQRT: /* TF' = .1+SQRT(TF) */
      #pragma omp parallel if (rowptr[nrows] > OMPMINOPS)
      {
        #pragma omp for private(j) schedule(static)
        for (i=0; i<nrows; i++) {
          for (j=rowptr[i]; j<rowptr[i+1]; j++) { 
            if (rowval[j] != 0.0)
              rowval[j] = .1+sign(rowval[j], sqrt(fabs(rowval[j])));
          }
        }
      }
      break;

    case GK_CSR_POW25: /* TF' = .1+POW(TF,.25) */
      #pragma omp parallel if (rowptr[nrows] > OMPMINOPS)
      {
        #pragma omp for private(j) schedule(static)
        for (i=0; i<nrows; i++) {
          for (j=rowptr[i]; j<rowptr[i+1]; j++) { 
            if (rowval[j] != 0.0)
              rowval[j] = .1+sign(rowval[j], sqrt(sqrt(fabs(rowval[j]))));
          }
        }
      }
      break;

    case GK_CSR_POW65: /* TF' = .1+POW(TF,.65) */
      #pragma omp parallel if (rowptr[nrows] > OMPMINOPS)
      {
        #pragma omp for private(j) schedule(static)
        for (i=0; i<nrows; i++) {
          for (j=rowptr[i]; j<rowptr[i+1]; j++) { 
            if (rowval[j] != 0.0)
              rowval[j] = .1+sign(rowval[j], powf(fabs(rowval[j]), .65));
          }
        }
      }
      break;

    case GK_CSR_POW75: /* TF' = .1+POW(TF,.75) */
      #pragma omp parallel if (rowptr[nrows] > OMPMINOPS)
      {
        #pragma omp for private(j) schedule(static)
        for (i=0; i<nrows; i++) {
          for (j=rowptr[i]; j<rowptr[i+1]; j++) { 
            if (rowval[j] != 0.0)
              rowval[j] = .1+sign(rowval[j], powf(fabs(rowval[j]), .75));
          }
        }
      }
      break;

    case GK_CSR_POW85: /* TF' = .1+POW(TF,.85) */
      #pragma omp parallel if (rowptr[nrows] > OMPMINOPS)
      {
        #pragma omp for private(j) schedule(static)
        for (i=0; i<nrows; i++) {
          for (j=rowptr[i]; j<rowptr[i+1]; j++) { 
            if (rowval[j] != 0.0)
              rowval[j] = .1+sign(rowval[j], powf(fabs(rowval[j]), .85));
          }
        }
      }
      break;

    case GK_CSR_LOG: /* TF' = 1+log_2(TF) */
      #pragma omp parallel if (rowptr[nrows] > OMPMINOPS)
      {
        double logscale = 1.0/log(2.0);
        #pragma omp for schedule(static,32)
        for (i=0; i<rowptr[nrows]; i++) {
          if (rowval[i] != 0.0)
            rowval[i] = 1+(rowval[i]>0.0 ? log(rowval[i]) : -log(-rowval[i]))*logscale;
        }
#ifdef XXX
        #pragma omp for private(j) schedule(static)
        for (i=0; i<nrows; i++) {
          for (j=rowptr[i]; j<rowptr[i+1]; j++) { 
            if (rowval[j] != 0.0)
              rowval[j] = 1+(rowval[j]>0.0 ? log(rowval[j]) : -log(-rowval[j]))*logscale;
              //rowval[j] = 1+sign(rowval[j], log(fabs(rowval[j]))*logscale);
          }
        }
#endif
      }
      break;

    case GK_CSR_IDF: /* TF' = TF*IDF */
      ncols  = mat->ncols;
      cscale = gk_fmalloc(ncols, "gk_csr_Scale: cscale");
      collen = gk_ismalloc(ncols, 0, "gk_csr_Scale: collen");

      for (i=0; i<nrows; i++) {
        for (j=rowptr[i]; j<rowptr[i+1]; j++)
          collen[rowind[j]]++;
      }

      #pragma omp parallel if (ncols > OMPMINOPS) 
      {
        #pragma omp for schedule(static)
        for (i=0; i<ncols; i++)
          cscale[i] = (collen[i] > 0 ? log(1.0*nrows/collen[i]) : 0.0);
      }

      #pragma omp parallel if (rowptr[nrows] > OMPMINOPS) 
      {
        #pragma omp for private(j) schedule(static)
        for (i=0; i<nrows; i++) {
          for (j=rowptr[i]; j<rowptr[i+1]; j++)
            rowval[j] *= cscale[rowind[j]];
        }
      }

      gk_free((void **)&cscale, &collen, LTERM);
      break;

    case GK_CSR_IDF2: /* TF' = TF*IDF */
      ncols  = mat->ncols;
      cscale = gk_fmalloc(ncols, "gk_csr_Scale: cscale");
      collen = gk_ismalloc(ncols, 0, "gk_csr_Scale: collen");

      for (i=0; i<nrows; i++) {
        for (j=rowptr[i]; j<rowptr[i+1]; j++)
          collen[rowind[j]]++;
      }

      nnzcols = 0;
      #pragma omp parallel if (ncols > OMPMINOPS) 
      {
        #pragma omp for schedule(static) reduction(+:nnzcols)
        for (i=0; i<ncols; i++)
          nnzcols += (collen[i] > 0 ? 1 : 0);

        bgfreq = gk_max(10, (ssize_t)(.5*rowptr[nrows]/nnzcols));
        printf("nnz: %zd, nnzcols: %d, bgfreq: %d\n", rowptr[nrows], nnzcols, bgfreq);

        #pragma omp for schedule(static)
        for (i=0; i<ncols; i++)
          cscale[i] = (collen[i] > 0 ? log(1.0*(nrows+2*bgfreq)/(bgfreq+collen[i])) : 0.0);
      }

      #pragma omp parallel if (rowptr[nrows] > OMPMINOPS) 
      {
        #pragma omp for private(j) schedule(static)
        for (i=0; i<nrows; i++) {
          for (j=rowptr[i]; j<rowptr[i+1]; j++)
            rowval[j] *= cscale[rowind[j]];
        }
      }

      gk_free((void **)&cscale, &collen, LTERM);
      break;

    default:
      gk_errexit(SIGERR, "Unknown scaling type of %d\n", type);
  }

}


/*************************************************************************/
/*! Computes the sums of the rows/columns
    \param mat the matrix itself,
    \param what is either GK_CSR_ROW or GK_CSR_COL indicating which 
           sums to compute.
*/
/**************************************************************************/
void gk_csr_ComputeSums(gk_csr_t *mat, int what)
{
  ssize_t i;
  int n;
  ssize_t *ptr;
  float *val, *sums;

  switch (what) {
    case GK_CSR_ROW:
      n   = mat->nrows;
      ptr = mat->rowptr;
      val = mat->rowval;

      if (mat->rsums) 
        gk_free((void **)&mat->rsums, LTERM);

      sums = mat->rsums = gk_fsmalloc(n, 0, "gk_csr_ComputeSums: sums");
      break;
    case GK_CSR_COL:
      n   = mat->ncols;
      ptr = mat->colptr;
      val = mat->colval;

      if (mat->csums) 
        gk_free((void **)&mat->csums, LTERM);

      sums = mat->csums = gk_fsmalloc(n, 0, "gk_csr_ComputeSums: sums");
      break;
    default:
      gk_errexit(SIGERR, "Invalid sum type of %d.\n", what);
      return;
  }

  #pragma omp parallel for if (ptr[n] > OMPMINOPS) schedule(static)
  for (i=0; i<n; i++) 
    sums[i] = gk_fsum(ptr[i+1]-ptr[i], val+ptr[i], 1);
}


/*************************************************************************/
/*! Computes the squared of the norms of the rows/columns
    \param mat the matrix itself,
    \param what is either GK_CSR_ROW or GK_CSR_COL indicating which 
           squared norms to compute.
*/
/**************************************************************************/
void gk_csr_ComputeSquaredNorms(gk_csr_t *mat, int what)
{
  ssize_t i;
  int n;
  ssize_t *ptr;
  float *val, *norms;

  switch (what) {
    case GK_CSR_ROW:
      n   = mat->nrows;
      ptr = mat->rowptr;
      val = mat->rowval;

      if (mat->rnorms) gk_free((void **)&mat->rnorms, LTERM);

      norms = mat->rnorms = gk_fsmalloc(n, 0, "gk_csr_ComputeSums: norms");
      break;
    case GK_CSR_COL:
      n   = mat->ncols;
      ptr = mat->colptr;
      val = mat->colval;

      if (mat->cnorms) gk_free((void **)&mat->cnorms, LTERM);

      norms = mat->cnorms = gk_fsmalloc(n, 0, "gk_csr_ComputeSums: norms");
      break;
    default:
      gk_errexit(SIGERR, "Invalid norm type of %d.\n", what);
      return;
  }

  #pragma omp parallel for if (ptr[n] > OMPMINOPS) schedule(static)
  for (i=0; i<n; i++) 
    norms[i] = gk_fdot(ptr[i+1]-ptr[i], val+ptr[i], 1, val+ptr[i], 1);
}


/*************************************************************************/
/*! Computes the similarity between two rows/columns

    \param mat the matrix itself. The routine assumes that the indices
           are sorted in increasing order.
    \param i1 is the first row/column,
    \param i2 is the second row/column,
    \param what is either GK_CSR_ROW or GK_CSR_COL indicating the type of
           objects between the similarity will be computed,
    \param simtype is the type of similarity and is one of GK_CSR_COS,
           GK_CSR_JAC, GK_CSR_MIN, GK_CSR_AMIN
    \returns the similarity between the two rows/columns.
*/
/**************************************************************************/
float gk_csr_ComputeSimilarity(gk_csr_t *mat, int i1, int i2, int what, int simtype)
{
  int nind1, nind2;
  int *ind1, *ind2;
  float *val1, *val2, stat1, stat2, sim;

  switch (what) {
    case GK_CSR_ROW:
      if (!mat->rowptr)
        gk_errexit(SIGERR, "Row-based view of the matrix does not exists.\n");
      nind1 = mat->rowptr[i1+1]-mat->rowptr[i1];
      nind2 = mat->rowptr[i2+1]-mat->rowptr[i2];
      ind1  = mat->rowind + mat->rowptr[i1];
      ind2  = mat->rowind + mat->rowptr[i2];
      val1  = mat->rowval + mat->rowptr[i1];
      val2  = mat->rowval + mat->rowptr[i2];
      break;

    case GK_CSR_COL:
      if (!mat->colptr)
        gk_errexit(SIGERR, "Column-based view of the matrix does not exists.\n");
      nind1 = mat->colptr[i1+1]-mat->colptr[i1];
      nind2 = mat->colptr[i2+1]-mat->colptr[i2];
      ind1  = mat->colind + mat->colptr[i1];
      ind2  = mat->colind + mat->colptr[i2];
      val1  = mat->colval + mat->colptr[i1];
      val2  = mat->colval + mat->colptr[i2];
      break;

    default:
      gk_errexit(SIGERR, "Invalid index type of %d.\n", what);
      return 0.0;
  }


  switch (simtype) {
    case GK_CSR_COS:
    case GK_CSR_JAC:
      sim = stat1 = stat2 = 0.0;
      i1 = i2 = 0;
      while (i1<nind1 && i2<nind2) {
        if (i1 == nind1) {
          stat2 += val2[i2]*val2[i2];
          i2++;
        }
        else if (i2 == nind2) {
          stat1 += val1[i1]*val1[i1];
          i1++;
        }
        else if (ind1[i1] < ind2[i2]) {
          stat1 += val1[i1]*val1[i1];
          i1++;
        }
        else if (ind1[i1] > ind2[i2]) {
          stat2 += val2[i2]*val2[i2];
          i2++;
        }
        else {
          sim   += val1[i1]*val2[i2];
          stat1 += val1[i1]*val1[i1];
          stat2 += val2[i2]*val2[i2];
          i1++;
          i2++;
        }
      }
      if (simtype == GK_CSR_COS)
        sim = (stat1*stat2 > 0.0 ? sim/sqrt(stat1*stat2) : 0.0);
      else 
        sim = (stat1+stat2-sim > 0.0 ? sim/(stat1+stat2-sim) : 0.0);
      break;

    case GK_CSR_MIN:
      sim = stat1 = stat2 = 0.0;
      i1 = i2 = 0;
      while (i1<nind1 && i2<nind2) {
        if (i1 == nind1) {
          stat2 += val2[i2];
          i2++;
        }
        else if (i2 == nind2) {
          stat1 += val1[i1];
          i1++;
        }
        else if (ind1[i1] < ind2[i2]) {
          stat1 += val1[i1];
          i1++;
        }
        else if (ind1[i1] > ind2[i2]) {
          stat2 += val2[i2];
          i2++;
        }
        else {
          sim   += gk_min(val1[i1],val2[i2]);
          stat1 += val1[i1];
          stat2 += val2[i2];
          i1++;
          i2++;
        }
      }
      sim = (stat1+stat2-sim > 0.0 ? sim/(stat1+stat2-sim) : 0.0);

      break;

    case GK_CSR_AMIN:
      sim = stat1 = stat2 = 0.0;
      i1 = i2 = 0;
      while (i1<nind1 && i2<nind2) {
        if (i1 == nind1) {
          stat2 += val2[i2];
          i2++;
        }
        else if (i2 == nind2) {
          stat1 += val1[i1];
          i1++;
        }
        else if (ind1[i1] < ind2[i2]) {
          stat1 += val1[i1];
          i1++;
        }
        else if (ind1[i1] > ind2[i2]) {
          stat2 += val2[i2];
          i2++;
        }
        else {
          sim   += gk_min(val1[i1],val2[i2]);
          stat1 += val1[i1];
          stat2 += val2[i2];
          i1++;
          i2++;
        }
      }
      sim = (stat1 > 0.0 ? sim/stat1 : 0.0);

      break;

    default:
      gk_errexit(SIGERR, "Unknown similarity measure %d\n", simtype);
      return -1;
  }

  return sim;

}


/*************************************************************************/
/*! Finds the n most similar rows (neighbors) to the query using cosine
    similarity.

    \param mat the matrix itself
    \param nqterms is the number of columns in the query
    \param qind is the list of query columns
    \param qval is the list of correspodning query weights
    \param simtype is the type of similarity and is one of GK_CSR_COS,
           GK_CSR_JAC, GK_CSR_MIN, GK_CSR_AMIN
    \param nsim is the maximum number of requested most similar rows.
           If -1 is provided, then everything is returned unsorted.
    \param minsim is the minimum similarity of the requested most 
           similar rows
    \param hits is the result set. This array should be at least
           of length nsim.
    \param i_marker is an array of size equal to the number of rows
           whose values are initialized to -1. If NULL is provided
           then this array is allocated and freed internally.
    \param i_cand is an array of size equal to the number of rows.
           If NULL is provided then this array is allocated and freed 
           internally.
    \returns the number of identified most similar rows, which can be
             smaller than the requested number of nnbrs in those cases
             in which there are no sufficiently many neighbors.
*/
/**************************************************************************/
int gk_csr_GetSimilarRows(gk_csr_t *mat, int nqterms, int *qind, 
        float *qval, int simtype, int nsim, float minsim, gk_fkv_t *hits, 
        int *i_marker, gk_fkv_t *i_cand)
{
  ssize_t i, ii, j, k;
  int nrows, ncols, ncand;
  ssize_t *colptr;
  int *colind, *marker;
  float *colval, *rnorms, mynorm, *rsums, mysum;
  gk_fkv_t *cand;

  if (nqterms == 0)
    return 0;

  nrows  = mat->nrows;
  ncols  = mat->ncols;
  colptr = mat->colptr;
  colind = mat->colind;
  colval = mat->colval;

  marker = (i_marker ? i_marker : gk_ismalloc(nrows, -1, "gk_csr_SimilarRows: marker"));
  cand   = (i_cand   ? i_cand   : gk_fkvmalloc(nrows, "gk_csr_SimilarRows: cand"));

  switch (simtype) {
    case GK_CSR_COS:
      for (ncand=0, ii=0; ii<nqterms; ii++) {
        i = qind[ii];
        if (i < ncols) {
          for (j=colptr[i]; j<colptr[i+1]; j++) {
            k = colind[j];
            if (marker[k] == -1) {
              cand[ncand].val = k;
              cand[ncand].key = 0;
              marker[k]       = ncand++;
            }
            cand[marker[k]].key += colval[j]*qval[ii];
          }
        }
      }
      break;

    case GK_CSR_JAC:
      for (ncand=0, ii=0; ii<nqterms; ii++) {
        i = qind[ii];
        if (i < ncols) {
          for (j=colptr[i]; j<colptr[i+1]; j++) {
            k = colind[j];
            if (marker[k] == -1) {
              cand[ncand].val = k;
              cand[ncand].key = 0;
              marker[k]       = ncand++;
            }
            cand[marker[k]].key += colval[j]*qval[ii];
          }
        }
      }

      rnorms = mat->rnorms;
      mynorm = gk_fdot(nqterms, qval, 1, qval, 1);

      for (i=0; i<ncand; i++)
        cand[i].key = cand[i].key/(rnorms[cand[i].val]+mynorm-cand[i].key);
      break;

    case GK_CSR_MIN:
      for (ncand=0, ii=0; ii<nqterms; ii++) {
        i = qind[ii];
        if (i < ncols) {
          for (j=colptr[i]; j<colptr[i+1]; j++) {
            k = colind[j];
            if (marker[k] == -1) {
              cand[ncand].val = k;
              cand[ncand].key = 0;
              marker[k]       = ncand++;
            }
            cand[marker[k]].key += gk_min(colval[j], qval[ii]);
          }
        }
      }

      rsums = mat->rsums;
      mysum = gk_fsum(nqterms, qval, 1);

      for (i=0; i<ncand; i++)
        cand[i].key = cand[i].key/(rsums[cand[i].val]+mysum-cand[i].key);
      break;

    /* Assymetric MIN  similarity */
    case GK_CSR_AMIN:
      for (ncand=0, ii=0; ii<nqterms; ii++) {
        i = qind[ii];
        if (i < ncols) {
          for (j=colptr[i]; j<colptr[i+1]; j++) {
            k = colind[j];
            if (marker[k] == -1) {
              cand[ncand].val = k;
              cand[ncand].key = 0;
              marker[k]       = ncand++;
            }
            cand[marker[k]].key += gk_min(colval[j], qval[ii]);
          }
        }
      }

      mysum = gk_fsum(nqterms, qval, 1);

      for (i=0; i<ncand; i++)
        cand[i].key = cand[i].key/mysum;
      break;

    default:
      gk_errexit(SIGERR, "Unknown similarity measure %d\n", simtype);
      return -1;
  }

  /* go and prune the hits that are bellow minsim */
  for (j=0, i=0; i<ncand; i++) {
    marker[cand[i].val] = -1;
    if (cand[i].key >= minsim) 
      cand[j++] = cand[i];
  }
  ncand = j;

  if (nsim == -1 || nsim >= ncand) {
    nsim = ncand;
  }
  else {
    nsim = gk_min(nsim, ncand);
    gk_dfkvkselect(ncand, nsim, cand);
    gk_fkvsortd(nsim, cand);
  }

  gk_fkvcopy(nsim, cand, hits);

  if (i_marker == NULL)
    gk_free((void **)&marker, LTERM);
  if (i_cand == NULL)
    gk_free((void **)&cand, LTERM);

  return nsim;
}

