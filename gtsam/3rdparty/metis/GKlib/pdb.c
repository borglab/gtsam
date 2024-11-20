/************************************************************************/
/*! \file pdb.c

\brief Functions for parsing pdb files.

Pdb reader (parser).  Loads arrays of pointers for easy backbone access.

\date Started 10/20/06
\author Kevin
\version $Id: pdb.c 10711 2011-08-31 22:23:04Z karypis $
*/
/************************************************************************/
#include <GKlib.h>

/************************************************************************/
/*! \brief Converts three-letter amino acid codes to one-leter codes.
 
This function takes a three letter \c * and converts it to a single \c 

\param res is the three-letter code to be converted.
\returns A \c representing the amino acid.
*/
/************************************************************************/
char gk_threetoone(char *res) { /* {{{ */
	/* make sure the matching works */
	res[0] = toupper(res[0]);
	res[1] = toupper(res[1]);
	res[2] = toupper(res[2]);
	if(strcmp(res,"ALA") == 0) {
		return 'A';
	}
	else if(strcmp(res,"CYS") == 0) {
		return 'C';
	}
	else if(strcmp(res,"ASP") == 0) {
		return 'D';
	}
	else if(strcmp(res,"GLU") == 0) {
		return 'E';
	}
	else if(strcmp(res,"PHE") == 0) {
		return 'F';
	}
	else if(strcmp(res,"GLY") == 0) {
		return 'G';
	}
	else if(strcmp(res,"HIS") == 0) {
		return 'H';
	}
	else if(strcmp(res,"ILE") == 0) {
		return 'I';
	}
	else if(strcmp(res,"LYS") == 0) {
		return 'K';
	}
	else if(strcmp(res,"LEU") == 0) {
		return 'L';
	}
	else if(strcmp(res,"MET") == 0) {
		return 'M';
	}
	else if(strcmp(res,"ASN") == 0) {
		return 'N';
	}
	else if(strcmp(res,"PRO") == 0) {
		return 'P';
	}
	else if(strcmp(res,"GLN") == 0) {
		return 'Q';
	}
	else if(strcmp(res,"ARG") == 0) {
		return 'R';
	}
	else if(strcmp(res,"SER") == 0) {
		return 'S';
	}
	else if(strcmp(res,"THR") == 0) {
		return 'T';
	}
	else if(strcmp(res,"SCY") == 0) {
		return 'U';
	}
	else if(strcmp(res,"VAL") == 0) {
		return 'V';
	}
	else if(strcmp(res,"TRP") == 0) {
		return 'W';
	}
	else if(strcmp(res,"TYR") == 0) {
		return 'Y';
	}
	else  {
		return 'X';
	}
} /* }}} */

/************************************************************************/
/*! \brief Frees the memory of a pdbf structure.
 
This function takes a pdbf pointer and frees all the memory below it. 

\param p is the pdbf structure to be freed.
*/
/************************************************************************/
void gk_freepdbf(pdbf *p) { /* {{{ */
	int i;
	if(p != NULL) {
		gk_free((void **)&p->resSeq, LTERM);
		for(i=0; i<p->natoms; i++) {
			gk_free((void **)&p->atoms[i].name, &p->atoms[i].resname, LTERM);
    }
		for(i=0; i<p->nresidues; i++) {
      gk_free((void *)&p->threeresSeq[i], LTERM);
    }
		/* this may look like it's wrong, but it's just a 1-d array of pointers, and
			 the pointers themselves are freed above */
	  gk_free((void **)&p->bbs, &p->cas, &p->atoms, &p->cm, &p->threeresSeq, LTERM);
	}
	gk_free((void **)&p, LTERM);
} /* }}} */

/************************************************************************/
/*! \brief Reads a pdb file into a pdbf structure
 
This function allocates a pdbf structure and reads the file fname into 
that structure.

\param fname is the file name to be read
\returns A filled pdbf structure.
*/
/************************************************************************/
pdbf *gk_readpdbfile(char *fname) { /* {{{ */
	int i=0, res=0; 
	char linetype[7];
	int  aserial;
	char aname[5] = "    \0";
	char altLoc   = ' ';
	char rname[4] = "   \0";
	char chainid  = ' ';
	char oldchainid  = ' ';
	int  rserial;
	int  oldRserial = -37;
	char icode    = ' ';
	char element  = ' ';
	double x;
	double y;
	double z;
	double avgx;
	double avgy;
	double avgz;
	double opcy;
	double tmpt;
	char line[MAXLINELEN];
	int corruption=0;
  int nresatoms;

	int atoms=0, residues=0, cas=0, bbs=0, firstres=1;
	pdbf *toFill = gk_malloc(sizeof(pdbf),"fillme");
	FILE *FPIN; 

	FPIN = gk_fopen(fname,"r",fname);	
	while(fgets(line, 256, FPIN))	{
		sscanf(line,"%s ",linetype);
		/* It seems the only reliable parts are through temperature, so we only use these parts */
		/* if(strstr(linetype, "ATOM") != NULL || strstr(linetype, "HETATM") != NULL) { */
		if(strstr(linetype, "ATOM") != NULL) {
			sscanf(line, "%6s%5d%*1c%4c%1c%3c%*1c%1c%4d%1c%*3c%8lf%8lf%8lf%6lf%6lf %c\n",
			linetype,&aserial,aname,&altLoc,rname,&chainid,&rserial,&icode,&x,&y,&z,&opcy,&tmpt,&element);
			sscanf(linetype, " %s ",linetype);
			sscanf(aname, " %s ",aname);
			sscanf(rname, " %s ",rname);
			if(altLoc != ' ') {
				corruption = corruption|CRP_ALTLOCS;	
			}

			if(firstres == 1) {
				oldRserial = rserial;
				oldchainid = chainid;
				residues++;
				firstres = 0;
			}
			if(oldRserial != rserial) {
				residues++;
				oldRserial = rserial;
			}
			if(oldchainid != chainid) {
				corruption = corruption|CRP_MULTICHAIN;
			}
			oldchainid = chainid;
			atoms++;
		  if(strcmp(aname,"CA") == 0) {
				cas++;
			}
			if(strcmp(aname,"N") == 0 || strcmp(aname,"CA") == 0 || 
         strcmp(aname,"C") == 0 || strcmp(aname,"O") == 0) {
				bbs++;
			}
		}
		else if(strstr(linetype, "ENDMDL") != NULL || strstr(linetype, "END") != NULL || strstr(linetype, "TER") != NULL) {
			break;
		}
	}
	fclose(FPIN);

	/* printf("File has coordinates for %d atoms in %d residues\n",atoms,residues); */
	toFill->natoms      = atoms;
	toFill->ncas        = cas;
	toFill->nbbs        = bbs;
	toFill->nresidues   = residues;
	toFill->resSeq      = (char *) gk_malloc (residues*sizeof(char),"residue seq");
	toFill->threeresSeq = (char **)gk_malloc (residues*sizeof(char *),"residue seq");
	toFill->atoms       = (atom *) gk_malloc (atoms*sizeof(atom),  "atoms");
	toFill->bbs         = (atom **)gk_malloc (  bbs*sizeof(atom *),"bbs");
	toFill->cas         = (atom **)gk_malloc (  cas*sizeof(atom *),"cas");
	toFill->cm          = (center_of_mass *)gk_malloc(residues*sizeof(center_of_mass),"center of mass");
	res=0; firstres=1; cas=0; bbs=0; i=0; 
  avgx = 0.0; avgy = 0.0; avgz = 0.0;
  nresatoms = 0;

	FPIN = gk_fopen(fname,"r",fname);	
	while(fgets(line, 256, FPIN))	{
		sscanf(line,"%s ",linetype);
		/* It seems the only reliable parts are through temperature, so we only use these parts */
		/* if(strstr(linetype, "ATOM") != NULL || strstr(linetype, "HETATM") != NULL) { */
		if(strstr(linetype, "ATOM") != NULL ) {

			/* to ensure our memory doesn't get corrupted by the biologists, we only read this far */
			sscanf(line, "%6s%5d%*1c%4c%1c%3c%*1c%1c%4d%1c%*3c%8lf%8lf%8lf%6lf%6lf %c\n",
			linetype,&aserial,aname,&altLoc,rname,&chainid,&rserial,&icode,&x,&y,&z,&opcy,&tmpt,&element);
			sscanf(aname, "%s",aname);
			sscanf(rname, "%s",rname);

			if(firstres == 1) {
				toFill->resSeq[res] = gk_threetoone(rname);
			  toFill->threeresSeq[res] = gk_strdup(rname); 
				oldRserial = rserial;
				res++;
				firstres = 0;
			}
			if(oldRserial != rserial) {
        /* we're changing residues. store the center of mass from the last one & reset */
        toFill->cm[res-1].x = avgx/nresatoms;
        toFill->cm[res-1].y = avgy/nresatoms;
        toFill->cm[res-1].z = avgz/nresatoms;
	      avgx = 0.0; avgy = 0.0; avgz = 0.0;
        nresatoms = 0;
        toFill->cm[res-1].name = toFill->resSeq[res-1];

			  toFill->threeresSeq[res] = gk_strdup(rname); 
				toFill->resSeq[res] = gk_threetoone(rname);
				res++;
				oldRserial = rserial;
			}
      avgx += x;
      avgy += y;
      avgz += z;
      nresatoms++;

			toFill->atoms[i].x       = x;
			toFill->atoms[i].y       = y;
			toFill->atoms[i].z       = z;
			toFill->atoms[i].opcy    = opcy;
			toFill->atoms[i].tmpt    = tmpt;
			toFill->atoms[i].element = element;
			toFill->atoms[i].serial  = aserial;
			toFill->atoms[i].chainid = chainid;
			toFill->atoms[i].altLoc  = altLoc;
			toFill->atoms[i].rserial = rserial;
			toFill->atoms[i].icode   = icode;
			toFill->atoms[i].name    = gk_strdup(aname); 
			toFill->atoms[i].resname = gk_strdup(rname); 
			/* Set up pointers for the backbone and c-alpha shortcuts */
			 if(strcmp(aname,"CA") == 0) {
				toFill->cas[cas] = &(toFill->atoms[i]);
				cas++;
			}
			if(strcmp(aname,"N") == 0 || strcmp(aname,"CA") == 0 || strcmp(aname,"C") == 0 || strcmp(aname,"O") == 0) {
				toFill->bbs[bbs] = &(toFill->atoms[i]);
				bbs++;
			}
			i++;
		}
		else if(strstr(linetype, "ENDMDL") != NULL || strstr(linetype, "END") != NULL || strstr(linetype, "TER") != NULL) {
			break;
		}
	}
  /* get that last average */
  toFill->cm[res-1].x = avgx/nresatoms;
  toFill->cm[res-1].y = avgy/nresatoms;
  toFill->cm[res-1].z = avgz/nresatoms;
	/* Begin test code */
	if(cas != residues) {
		printf("Number of residues and CA coordinates differs by %d (!)\n",residues-cas);
		if(cas < residues) {
			corruption = corruption|CRP_MISSINGCA;	
		}
		else if(cas > residues) {
			corruption = corruption|CRP_MULTICA;	
		}
	}
	if(bbs < residues*4) {
		corruption = corruption|CRP_MISSINGBB;
	}
	else if(bbs > residues*4) {
		corruption = corruption|CRP_MULTIBB;
	}
	fclose(FPIN);
	toFill->corruption = corruption;
	/* if(corruption == 0) 
		printf("File was clean!\n"); */
	return(toFill);
} /* }}} */

/************************************************************************/
/*! \brief Writes the sequence of residues from a pdb file.
 
This function takes a pdbf structure and a filename, and writes out 
the amino acid sequence according to the atomic coordinates.  The output
is in fasta format.


\param p is the pdbf structure with the sequence of interest
\param fname is the file name to be written
*/
/************************************************************************/
void gk_writefastafrompdb(pdbf *pb, char *fname) {
  int i;
  FILE *FPOUT;
  
  FPOUT = gk_fopen(fname,"w",fname);
  fprintf(FPOUT,"> %s\n",fname);

  for(i=0; i<pb->nresidues; i++) 
    fprintf(FPOUT,"%c",pb->resSeq[i]);

  fprintf(FPOUT,"\n");
  fclose(FPOUT);
}

/************************************************************************/
/*! \brief Writes all centers of mass in pdb-format to file fname.
 
This function takes a pdbf structure and writes out the calculated 
mass center information to file fname as though each one was a c-alpha.

\param p is the pdbf structure to write out
\param fname is the file name to be written
*/
/************************************************************************/
void gk_writecentersofmass(pdbf *p, char *fname) {
	int i;
	FILE *FPIN; 
	FPIN = gk_fopen(fname,"w",fname);	
	for(i=0; i<p->nresidues; i++) {
		 fprintf(FPIN,"%-6s%5d %4s%1c%3s %1c%4d%1c   %8.3lf%8.3lf%8.3lf%6.2f%6.2f\n",
		"ATOM  ",i,"CA",' ',p->threeresSeq[i],' ',i,' ',p->cm[i].x,p->cm[i].y,p->cm[i].z,1.0,-37.0); 
	}
	fclose(FPIN);
}

/************************************************************************/
/*! \brief Writes all atoms in p in pdb-format to file fname.
 
This function takes a pdbf structure and writes out all the atom 
information to file fname.

\param p is the pdbf structure to write out
\param fname is the file name to be written
*/
/************************************************************************/
void gk_writefullatom(pdbf *p, char *fname) {
	int i;
	FILE *FPIN; 
	FPIN = gk_fopen(fname,"w",fname);	
	for(i=0; i<p->natoms; i++) {
		 fprintf(FPIN,"%-6s%5d %4s%1c%3s %1c%4d%1c   %8.3lf%8.3lf%8.3lf%6.2f%6.2f\n",
		"ATOM  ",p->atoms[i].serial,p->atoms[i].name,p->atoms[i].altLoc,p->atoms[i].resname,p->atoms[i].chainid,p->atoms[i].rserial,p->atoms[i].icode,p->atoms[i].x,p->atoms[i].y,p->atoms[i].z,p->atoms[i].opcy,p->atoms[i].tmpt); 
	}
	fclose(FPIN);
}

/************************************************************************/
/*! \brief Writes out all the backbone atoms of a structure in pdb format
 
This function takes a pdbf structure p and writes only the backbone atoms
to a filename fname.

\param p is the pdb structure to write out.
\param fname is the file name to be written.
*/
/************************************************************************/
void gk_writebackbone(pdbf *p, char *fname) {
	int i;
	FILE *FPIN; 
	FPIN = gk_fopen(fname,"w",fname);	
	for(i=0; i<p->nbbs; i++) {
		 fprintf(FPIN,"%-6s%5d %4s%1c%3s %1c%4d%1c   %8.3lf%8.3lf%8.3lf%6.2f%6.2f\n",
		"ATOM  ",p->bbs[i]->serial,p->bbs[i]->name,p->bbs[i]->altLoc,p->bbs[i]->resname,p->bbs[i]->chainid,p->bbs[i]->rserial,p->bbs[i]->icode,p->bbs[i]->x,p->bbs[i]->y,p->bbs[i]->z,p->bbs[i]->opcy,p->bbs[i]->tmpt); 
	}
	fclose(FPIN);
}

/************************************************************************/
/*! \brief Writes out all the alpha carbon atoms of a structure 
 
This function takes a pdbf structure p and writes only the alpha carbon 
atoms to a filename fname.

\param p is the pdb structure to write out.
\param fname is the file name to be written.
*/
/************************************************************************/
void gk_writealphacarbons(pdbf *p, char *fname) {
	int i;
	FILE *FPIN; 
	FPIN = gk_fopen(fname,"w",fname);	
	for(i=0; i<p->ncas; i++) {
		 fprintf(FPIN,"%-6s%5d %4s%1c%3s %1c%4d%1c   %8.3lf%8.3lf%8.3lf%6.2f%6.2f\n",
		"ATOM  ",p->cas[i]->serial,p->cas[i]->name,p->cas[i]->altLoc,p->cas[i]->resname,p->cas[i]->chainid,p->cas[i]->rserial,p->cas[i]->icode,p->cas[i]->x,p->cas[i]->y,p->cas[i]->z,p->cas[i]->opcy,p->cas[i]->tmpt); 
	}
	fclose(FPIN);
}

/************************************************************************/
/*! \brief Decodes the corruption bitswitch and prints any problems
 
Due to the totally unreliable nature of the pdb format, reading a pdb
file stores a corruption bitswitch, and this function decodes that switch
and prints the result on stdout.

\param p is the pdb structure to write out.
\param fname is the file name to be written.
*/
/************************************************************************/
void gk_showcorruption(pdbf *p) {
	int corruption = p->corruption;
	if(corruption&CRP_ALTLOCS)
		printf("Multiple coordinate sets for at least one atom\n");
	if(corruption&CRP_MISSINGCA) 
		printf("Missing coordiantes for at least one CA atom\n");
	if(corruption&CRP_MISSINGBB) 
		printf("Missing coordiantes for at least one backbone atom (N,CA,C,O)\n");
	if(corruption&CRP_MULTICHAIN) 
		printf("File contains coordinates for multiple chains\n");
	if(corruption&CRP_MULTICA) 
		printf("Multiple CA atoms found for the same residue (could be alternate locators)\n");
	if(corruption&CRP_MULTICA) 
		printf("Multiple copies of backbone atoms found for the same residue (could be alternate locators)\n");
}
			/* sscanf(line, "%6s%5d%*1c%4s%1c%3s%*1c%1c%4d%1c%*3c%8lf%8lf%8lf%6lf%6lf%*6c%4s%2s%2s\n",
			linetype,&aserial,aname,&altLoc,rname,&chainid,&rserial,&icode,&x,&y,&z,&opcy,&tmpt,segId,element,charge);
			printf(".%s.%s.%s.\n",segId,element,charge);
			printf("%-6s%5d%-1s%-4s%1c%3s%1s%1c%4d%1c%3s%8.3lf%8.3lf%8.3lf%6.2f%6.2f%6s%4s%2s%2s\n",
			linetype,aserial," ",aname,altLoc,rname," ",chainid,rserial,icode," ",x,y,z,opcy,tmpt," ",segId,element,charge); */

			/* and we could probably get away with this using astral files, */
			/* sscanf(line, "%6s%5d%*1c%4s%1c%3s%*1c%1c%4d%1c%*3c%8lf%8lf%8lf%6lf%6lf%*6c%6s\n",
			linetype,&aserial,aname,&altLoc,rname,&chainid,&rserial,&icode,&x,&y,&z,&opcy,&tmpt,element);
			printf("%-6s%5d%-1s%-4s%1c%3s%1s%1c%4d%1c%3s%8.3lf%8.3lf%8.3lf%6.2f%6.2f%6s%6s\n",
			linetype,aserial," ",aname,altLoc,rname," ",chainid,rserial,icode," ",x,y,z,opcy,tmpt," ",element); */
