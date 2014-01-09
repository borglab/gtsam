/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * io.c
 *
 * This file contains routines related to I/O
 *
 * Started 8/28/94
 * George
 *
 * $Id: io.c 11932 2012-05-10 18:18:23Z dominique $
 *
 */

#include "metisbin.h"



/*************************************************************************/
/*! This function reads in a sparse graph */
/*************************************************************************/
graph_t *ReadGraph(params_t *params)
{
  idx_t i, j, k, l, fmt, ncon, nfields, readew, readvw, readvs, edge, ewgt;
  idx_t *xadj, *adjncy, *vwgt, *adjwgt, *vsize;
  char *line=NULL, fmtstr[256], *curstr, *newstr;
  size_t lnlen=0;
  FILE *fpin;
  graph_t *graph;

  if (!gk_fexists(params->filename)) 
    errexit("File %s does not exist!\n", params->filename);

  graph = CreateGraph();

  fpin = gk_fopen(params->filename, "r", "ReadGRaph: Graph");

  /* Skip comment lines until you get to the first valid line */
  do {
    if (gk_getline(&line, &lnlen, fpin) == -1) 
      errexit("Premature end of input file: file: %s\n", params->filename);
  } while (line[0] == '%');


  fmt = ncon = 0;
  nfields = sscanf(line, "%"SCIDX" %"SCIDX" %"SCIDX" %"SCIDX, 
                &(graph->nvtxs), &(graph->nedges), &fmt, &ncon);

  if (nfields < 2) 
    errexit("The input file does not specify the number of vertices and edges.\n");

  if (graph->nvtxs <= 0 || graph->nedges <= 0) 
    errexit("The supplied nvtxs:%"PRIDX" and nedges:%"PRIDX" must be positive.\n", 
        graph->nvtxs, graph->nedges);
        
  if (fmt > 111) 
    errexit("Cannot read this type of file format [fmt=%"PRIDX"]!\n", fmt);

  sprintf(fmtstr, "%03"PRIDX, fmt%1000);
  readvs = (fmtstr[0] == '1');
  readvw = (fmtstr[1] == '1');
  readew = (fmtstr[2] == '1');
    
  /*printf("%s %"PRIDX" %"PRIDX" %"PRIDX"\n", fmtstr, readvs, readvw, readew); */


  if (ncon > 0 && !readvw) 
    errexit(
      "------------------------------------------------------------------------------\n"
      "***  I detected an error in your input file  ***\n\n"
      "You specified ncon=%"PRIDX", but the fmt parameter does not specify vertex weights\n" 
      "Make sure that the fmt parameter is set to either 10 or 11.\n"
      "------------------------------------------------------------------------------\n", ncon);

  graph->nedges *=2;
  ncon = graph->ncon = (ncon == 0 ? 1 : ncon);

  xadj   = graph->xadj   = ismalloc(graph->nvtxs+1, 0, "ReadGraph: xadj");
  adjncy = graph->adjncy = imalloc(graph->nedges, "ReadGraph: adjncy");
  vwgt   = graph->vwgt   = ismalloc(ncon*graph->nvtxs, 1, "ReadGraph: vwgt");
  adjwgt = graph->adjwgt = ismalloc(graph->nedges, 1, "ReadGraph: adjwgt");
  vsize  = graph->vsize  = ismalloc(graph->nvtxs, 1, "ReadGraph: vsize");


  /*----------------------------------------------------------------------
   * Read the sparse graph file
   *---------------------------------------------------------------------*/
  for (xadj[0]=0, k=0, i=0; i<graph->nvtxs; i++) {
    do {
      if (gk_getline(&line, &lnlen, fpin) == -1) 
        errexit("Premature end of input file while reading vertex %"PRIDX".\n", i+1);
    } while (line[0] == '%');

    curstr = line;
    newstr = NULL;

    /* Read vertex sizes */
    if (readvs) {
      vsize[i] = strtoidx(curstr, &newstr, 10);
      if (newstr == curstr)
        errexit("The line for vertex %"PRIDX" does not have vsize information\n", i+1);
      if (vsize[i] < 0)
        errexit("The size for vertex %"PRIDX" must be >= 0\n", i+1);
      curstr = newstr;
    }


    /* Read vertex weights */
    if (readvw) {
      for (l=0; l<ncon; l++) {
        vwgt[i*ncon+l] = strtoidx(curstr, &newstr, 10);
        if (newstr == curstr)
          errexit("The line for vertex %"PRIDX" does not have enough weights "
                  "for the %"PRIDX" constraints.\n", i+1, ncon);
        if (vwgt[i*ncon+l] < 0)
          errexit("The weight vertex %"PRIDX" and constraint %"PRIDX" must be >= 0\n", i+1, l);
        curstr = newstr;
      }
    }

    while (1) {
      edge = strtoidx(curstr, &newstr, 10);
      if (newstr == curstr)
        break; /* End of line */
      curstr = newstr;

      if (edge < 1 || edge > graph->nvtxs)
        errexit("Edge %"PRIDX" for vertex %"PRIDX" is out of bounds\n", edge, i+1);

      ewgt = 1;
      if (readew) {
        ewgt = strtoidx(curstr, &newstr, 10);
        if (newstr == curstr)
          errexit("Premature end of line for vertex %"PRIDX"\n", i+1);
        if (ewgt <= 0)
          errexit("The weight (%"PRIDX") for edge (%"PRIDX", %"PRIDX") must be positive.\n", 
              ewgt, i+1, edge);
        curstr = newstr;
      }

      if (k == graph->nedges)
        errexit("There are more edges in the file than the %"PRIDX" specified.\n", 
            graph->nedges/2);

      adjncy[k] = edge-1;
      adjwgt[k] = ewgt;
      k++;
    } 
    xadj[i+1] = k;
  }
  gk_fclose(fpin);

  if (k != graph->nedges) {
    printf("------------------------------------------------------------------------------\n");
    printf("***  I detected an error in your input file  ***\n\n");
    printf("In the first line of the file, you specified that the graph contained\n"
           "%"PRIDX" edges. However, I only found %"PRIDX" edges in the file.\n", 
           graph->nedges/2, k/2);
    if (2*k == graph->nedges) {
      printf("\n *> I detected that you specified twice the number of edges that you have in\n");
      printf("    the file. Remember that the number of edges specified in the first line\n");
      printf("    counts each edge between vertices v and u only once.\n\n");
    }
    printf("Please specify the correct number of edges in the first line of the file.\n");
    printf("------------------------------------------------------------------------------\n");
    exit(0);
  }

  gk_free((void *)&line, LTERM);

  return graph;
}


/*************************************************************************/
/*! This function reads in a mesh */
/*************************************************************************/
mesh_t *ReadMesh(params_t *params)
{
  idx_t i, j, k, l, nfields, ncon, node;
  idx_t *eptr, *eind, *ewgt;
  size_t nlines, ntokens;
  char *line=NULL, *curstr, *newstr;
  size_t lnlen=0;
  FILE *fpin;
  mesh_t *mesh;

  if (!gk_fexists(params->filename)) 
    errexit("File %s does not exist!\n", params->filename);

  mesh = CreateMesh();

  /* get some file stats */
  gk_getfilestats(params->filename, &nlines, &ntokens, NULL, NULL);

  fpin = gk_fopen(params->filename, "r", __func__);

  /* Skip comment lines until you get to the first valid line */
  do {
    if (gk_getline(&line, &lnlen, fpin) == -1) 
      errexit("Premature end of input file: file: %s\n", params->filename);
  } while (line[0] == '%');


  mesh->ncon = 0;
  nfields = sscanf(line, "%"SCIDX" %"SCIDX, &(mesh->ne), &(mesh->ncon));

  if (nfields < 1) 
    errexit("The input file does not specify the number of elements.\n");

  if (mesh->ne <= 0) 
    errexit("The supplied number of elements:%"PRIDX" must be positive.\n", mesh->ne);
        
  if (mesh->ne > nlines)
    errexit("The file has %zu lines which smaller than the number of "
            "elements of %"PRIDX" specified in the header line.\n", nlines, mesh->ne);

  ncon = mesh->ncon;
  eptr = mesh->eptr = ismalloc(mesh->ne+1, 0, "ReadMesh: eptr");
  eind = mesh->eind = imalloc(ntokens, "ReadMesh: eind");
  ewgt = mesh->ewgt = ismalloc((ncon == 0 ? 1 : ncon)*mesh->ne, 1, "ReadMesh: ewgt");


  /*----------------------------------------------------------------------
   * Read the mesh file
   *---------------------------------------------------------------------*/
  for (eptr[0]=0, k=0, i=0; i<mesh->ne; i++) {
    do {
      if (gk_getline(&line, &lnlen, fpin) == -1) 
        errexit("Premature end of input file while reading element %"PRIDX".\n", i+1);
    } while (line[0] == '%');

    curstr = line;
    newstr = NULL;

    /* Read element weights */
    for (l=0; l<ncon; l++) {
      ewgt[i*ncon+l] = strtoidx(curstr, &newstr, 10);
      if (newstr == curstr)
        errexit("The line for vertex %"PRIDX" does not have enough weights "
                "for the %"PRIDX" constraints.\n", i+1, ncon);
      if (ewgt[i*ncon+l] < 0)
        errexit("The weight for element %"PRIDX" and constraint %"PRIDX" must be >= 0\n", i+1, l);
      curstr = newstr;
    }

    while (1) {
      node = strtoidx(curstr, &newstr, 10);
      if (newstr == curstr)
        break; /* End of line */
      curstr = newstr;

      if (node < 1)
        errexit("Node %"PRIDX" for element %"PRIDX" is out of bounds\n", node, i+1);

      eind[k++] = node-1;
    } 
    eptr[i+1] = k;
  }
  gk_fclose(fpin);

  mesh->ncon = (ncon == 0 ? 1 : ncon);
  mesh->nn   = imax(eptr[mesh->ne], eind)+1;

  gk_free((void *)&line, LTERM);

  return mesh;
}


/*************************************************************************/
/*! This function reads in the target partition weights. If no file is 
    specified the weights are set to 1/nparts */
/*************************************************************************/
void ReadTPwgts(params_t *params, idx_t ncon)
{
  idx_t i, j, from, to, fromcnum, tocnum, nleft;
  real_t awgt=0.0, twgt;
  char *line=NULL, *curstr, *newstr;
  size_t lnlen=0;
  FILE *fpin;

  params->tpwgts = rsmalloc(params->nparts*ncon, -1.0, "ReadTPwgts: tpwgts");

  if (params->tpwgtsfile == NULL) {
    for (i=0; i<params->nparts; i++) {
      for (j=0; j<ncon; j++)
        params->tpwgts[i*ncon+j] = 1.0/params->nparts;
    }
    return;
  }

  if (!gk_fexists(params->tpwgtsfile)) 
    errexit("Graph file %s does not exist!\n", params->tpwgtsfile);

  fpin = gk_fopen(params->tpwgtsfile, "r", "ReadTPwgts: tpwgtsfile");

  while (gk_getline(&line, &lnlen, fpin) != -1) {
    gk_strchr_replace(line, " ", "");
    /* start extracting the fields */

    curstr = line;
    newstr = NULL;

    from = strtoidx(curstr, &newstr, 10);
    if (newstr == curstr)
      errexit("The 'from' component of line <%s> in the tpwgts file is incorrect.\n", line);
    curstr = newstr;

    if (curstr[0] == '-') {
      to = strtoidx(curstr+1, &newstr, 10);
      if (newstr == curstr)
        errexit("The 'to' component of line <%s> in the tpwgts file is incorrect.\n", line);
      curstr = newstr;
    }
    else {
      to = from;
    }

    if (curstr[0] == ':') {
      fromcnum = strtoidx(curstr+1, &newstr, 10);
      if (newstr == curstr)
        errexit("The 'fromcnum' component of line <%s> in the tpwgts file is incorrect.\n", line);
      curstr = newstr;

      if (curstr[0] == '-') {
        tocnum = strtoidx(curstr+1, &newstr, 10);
        if (newstr == curstr)
          errexit("The 'tocnum' component of line <%s> in the tpwgts file is incorrect.\n", line);
        curstr = newstr;
      }
      else {
        tocnum = fromcnum;
      }
    }
    else {
      fromcnum = 0;
      tocnum   = ncon-1;
    }

    if (curstr[0] == '=') {
      awgt = strtoreal(curstr+1, &newstr);
      if (newstr == curstr)
        errexit("The 'wgt' component of line <%s> in the tpwgts file is incorrect.\n", line);
      curstr = newstr;
    }
    else {
      errexit("The 'wgt' component of line <%s> in the tpwgts file is missing.\n", line);
    }

    /*printf("Read: %"PRIDX"-%"PRIDX":%"PRIDX"-%"PRIDX"=%"PRREAL"\n",
        from, to, fromcnum, tocnum, awgt);*/

    if (from < 0 || to < 0 || from >= params->nparts || to >= params->nparts)
      errexit("Invalid partition range for %"PRIDX":%"PRIDX"\n", from, to);
    if (fromcnum < 0 || tocnum < 0 || fromcnum >= ncon || tocnum >= ncon)
      errexit("Invalid constraint number range for %"PRIDX":%"PRIDX"\n", 
          fromcnum, tocnum);
    if (awgt <= 0.0 || awgt >= 1.0)
      errexit("Invalid partition weight of %"PRREAL"\n", awgt);
    for (i=from; i<=to; i++) {
      for (j=fromcnum; j<=tocnum; j++)
        params->tpwgts[i*ncon+j] = awgt;
    }
  } 

  gk_fclose(fpin);

  /* Assign weight to the unspecified constraints x partitions */
  for (j=0; j<ncon; j++) {
    /* Sum up the specified weights for the jth constraint */
    for (twgt=0.0, nleft=params->nparts, i=0; i<params->nparts; i++) {
      if (params->tpwgts[i*ncon+j] > 0) {
        twgt += params->tpwgts[i*ncon+j];
        nleft--;
      }
    }

    /* Rescale the weights to be on the safe side */
    if (nleft == 0) 
      rscale(params->nparts, 1.0/twgt, params->tpwgts+j, ncon);
  
    /* Assign the left-over weight to the remaining partitions */
    if (nleft > 0) {
      if (twgt > 1)
        errexit("The total specified target partition weights for constraint #%"PRIDX
                " of %"PRREAL" exceeds 1.0.\n", j, twgt);
  
      awgt = (1.0 - twgt)/nleft;
      for (i=0; i<params->nparts; i++)
        params->tpwgts[i*ncon+j] = 
            (params->tpwgts[i*ncon+j] < 0 ? awgt : params->tpwgts[i*ncon+j]);
    }
  }
  #ifdef HAVE_GETLINE
  free(line);
  line = NULL; /* set to null to match behavior of gk_free() */
  #else
  gk_free((void *)&line, LTERM);
  #endif
}


/*************************************************************************/
/*! This function reads in a partition/ordering vector  */
/**************************************************************************/
void ReadPOVector(graph_t *graph, char *filename, idx_t *vector)
{
  idx_t i;
  FILE *fpin;

  fpin = gk_fopen(filename, "r", __func__);
  for (i=0; i<graph->nvtxs; i++) {
    if (fscanf(fpin, "%"SCIDX"\n", vector+i) != 1)
      errexit("[%s] Premature end of file %s at line %d [nvtxs: %d]\n",
          __func__, filename, i, graph->nvtxs);
  }
  gk_fclose(fpin);
}


/*************************************************************************/
/*! This function writes out the partition vector */
/*************************************************************************/
void WritePartition(char *fname, idx_t *part, idx_t n, idx_t nparts)
{
  FILE *fpout;
  idx_t i;
  char filename[MAXLINE];

  sprintf(filename, "%s.part.%"PRIDX, fname, nparts);

  fpout = gk_fopen(filename, "w", __func__);

  for (i=0; i<n; i++)
    fprintf(fpout,"%" PRIDX "\n", part[i]);

  gk_fclose(fpout);
}


/*************************************************************************/
/*! This function writes out the partition vectors for a mesh */
/*************************************************************************/
void WriteMeshPartition(char *fname, idx_t nparts, idx_t ne, idx_t *epart, 
       idx_t nn, idx_t *npart)
{
  FILE *fpout;
  idx_t i;
  char filename[256];

  sprintf(filename,"%s.epart.%"PRIDX,fname, nparts);

  fpout = gk_fopen(filename, "w", __func__);

  for (i=0; i<ne; i++)
    fprintf(fpout,"%" PRIDX "\n", epart[i]);

  gk_fclose(fpout);


  sprintf(filename,"%s.npart.%"PRIDX,fname, nparts);

  fpout = gk_fopen(filename, "w", __func__);

  for (i=0; i<nn; i++)
    fprintf(fpout, "%" PRIDX "\n", npart[i]);

  gk_fclose(fpout);

}


/*************************************************************************/
/*! This function writes out the permutation vector */
/*************************************************************************/
void WritePermutation(char *fname, idx_t *iperm, idx_t n)
{
  FILE *fpout;
  idx_t i;
  char filename[MAXLINE];

  sprintf(filename, "%s.iperm", fname);

  fpout = gk_fopen(filename, "w", __func__);

  for (i=0; i<n; i++)
    fprintf(fpout, "%" PRIDX "\n", iperm[i]);

  gk_fclose(fpout);
}


/*************************************************************************/
/*! This function writes a graph into a file  */
/*************************************************************************/
void WriteGraph(graph_t *graph, char *filename)
{
  idx_t i, j, nvtxs, ncon;
  idx_t *xadj, *adjncy, *adjwgt, *vwgt, *vsize;
  int hasvwgt=0, hasewgt=0, hasvsize=0;
  FILE *fpout;

  nvtxs  = graph->nvtxs;
  ncon   = graph->ncon;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  vwgt   = graph->vwgt;
  vsize  = graph->vsize;
  adjwgt = graph->adjwgt;

  /* determine if the graph has non-unity vwgt, vsize, or adjwgt */
  if (vwgt) {
    for (i=0; i<nvtxs*ncon; i++) {
      if (vwgt[i] != 1) {
        hasvwgt = 1;
        break;
      }
    }
  }
  if (vsize) {
    for (i=0; i<nvtxs; i++) {
      if (vsize[i] != 1) {
        hasvsize = 1;
        break;
      }
    }
  }
  if (adjwgt) { 
    for (i=0; i<xadj[nvtxs]; i++) {
      if (adjwgt[i] != 1) {
        hasewgt = 1;
        break;
      }
    }
  }

  fpout = gk_fopen(filename, "w", __func__);

  /* write the header line */
  fprintf(fpout, "%"PRIDX" %"PRIDX, nvtxs, xadj[nvtxs]/2);
  if (hasvwgt || hasvsize || hasewgt) {
    fprintf(fpout, " %d%d%d", hasvsize, hasvwgt, hasewgt);
    if (hasvwgt)
      fprintf(fpout, " %d", (int)graph->ncon);
  }


  /* write the rest of the graph */
  for (i=0; i<nvtxs; i++) {
    fprintf(fpout, "\n");
    if (hasvsize) 
      fprintf(fpout, " %"PRIDX, vsize[i]);

    if (hasvwgt) {
      for (j=0; j<ncon; j++)
        fprintf(fpout, " %"PRIDX, vwgt[i*ncon+j]);
    }

    for (j=xadj[i]; j<xadj[i+1]; j++) {
      fprintf(fpout, " %"PRIDX, adjncy[j]+1);
      if (hasewgt)
        fprintf(fpout, " %"PRIDX, adjwgt[j]);
    }
  }

  gk_fclose(fpout);
}
