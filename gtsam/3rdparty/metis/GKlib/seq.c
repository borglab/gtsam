/*
 *
 * Sequence handler library by Huzefa Rangwala
 * Date : 03.01.2007
 *
 *
 *
 */


#include <GKlib.h>




/*********************************************************/
/* ! \brief Initializes the <tt>gk_seq_t</tt> variable




\param A pointer to gk_seq_t itself
\returns null
*/
/***********************************************************************/

void gk_seq_init(gk_seq_t *seq)
{
    
    seq->len = 0;
    seq->sequence = NULL;
        
    seq->pssm = NULL;
    seq->psfm = NULL;
    
    seq->name = NULL;
    
}

/***********************************************************************/
/*! \brief This function creates the localizations for the various sequences

\param    string i.e amino acids, nucleotides, sequences
\returns  gk_i2cc2i_t variable
*/
/*********************************************************************/

gk_i2cc2i_t *gk_i2cc2i_create_common(char *alphabet)
{
    
    
    int nsymbols;
    gk_idx_t i;
    gk_i2cc2i_t *t;

    nsymbols = strlen(alphabet);
    t        = gk_malloc(sizeof(gk_i2cc2i_t),"gk_i2c_create_common");
    t->n     = nsymbols;
    t->i2c   = gk_cmalloc(256, "gk_i2c_create_common");
    t->c2i   = gk_imalloc(256, "gk_i2c_create_common");
    

    gk_cset(256, -1, t->i2c);
    gk_iset(256, -1, t->c2i);
    
    for(i=0;i<nsymbols;i++){
	t->i2c[i] = alphabet[i];
	t->c2i[(int)alphabet[i]] = i;
    }

    return t;

}


/*********************************************************************/
/*! \brief This function reads a pssm in the format of gkmod pssm

\param file_name is the name of the pssm file
\returns gk_seq_t
*/
/********************************************************************/
gk_seq_t *gk_seq_ReadGKMODPSSM(char *filename)
{
    gk_seq_t *seq;
    gk_idx_t i, j, ii;
    size_t ntokens, nbytes, len;
    FILE *fpin;
    
    
    gk_Tokens_t tokens;
    static char *AAORDER = "ARNDCQEGHILKMFPSTWYVBZX*";
    static int PSSMWIDTH = 20;
    char *header, line[MAXLINELEN];
    gk_i2cc2i_t *converter;

    header = gk_cmalloc(PSSMWIDTH, "gk_seq_ReadGKMODPSSM: header");
    
    converter = gk_i2cc2i_create_common(AAORDER);
    
    gk_getfilestats(filename, &len, &ntokens, NULL, &nbytes);
    len --;

    seq = gk_malloc(sizeof(gk_seq_t),"gk_seq_ReadGKMODPSSM");
    gk_seq_init(seq);
    
    seq->len = len;
    seq->sequence = gk_imalloc(len, "gk_seq_ReadGKMODPSSM");
    seq->pssm     = gk_iAllocMatrix(len, PSSMWIDTH, 0, "gk_seq_ReadGKMODPSSM");
    seq->psfm     = gk_iAllocMatrix(len, PSSMWIDTH, 0, "gk_seq_ReadGKMODPSSM");
    
    seq->nsymbols = PSSMWIDTH;
    seq->name     = gk_getbasename(filename);
    
    fpin = gk_fopen(filename,"r","gk_seq_ReadGKMODPSSM");


    /* Read the header line */
    if (fgets(line, MAXLINELEN-1, fpin) == NULL)
      errexit("Unexpected end of file: %s\n", filename);
    gk_strtoupper(line);
    gk_strtokenize(line, " \t\n", &tokens);

    for (i=0; i<PSSMWIDTH; i++)
	header[i] = tokens.list[i][0];
    
    gk_freetokenslist(&tokens);
    

    /* Read the rest of the lines */
    for (i=0, ii=0; ii<len; ii++) {
	if (fgets(line, MAXLINELEN-1, fpin) == NULL)
          errexit("Unexpected end of file: %s\n", filename);
	gk_strtoupper(line);
	gk_strtokenize(line, " \t\n", &tokens);
	
	seq->sequence[i] = converter->c2i[(int)tokens.list[1][0]];
	
	for (j=0; j<PSSMWIDTH; j++) {
	    seq->pssm[i][converter->c2i[(int)header[j]]] = atoi(tokens.list[2+j]);
	    seq->psfm[i][converter->c2i[(int)header[j]]] = atoi(tokens.list[2+PSSMWIDTH+j]);
	}
	
      
	
	gk_freetokenslist(&tokens);
	i++;
    }
    
    seq->len = i; /* Reset the length if certain characters were skipped */
    
    gk_free((void **)&header, LTERM);
    gk_fclose(fpin);

    return seq;
}


/**************************************************************************/
/*! \brief This function frees the memory allocated to the seq structure.
 
\param   gk_seq_t
\returns nothing
*/
/**************************************************************************/
void gk_seq_free(gk_seq_t *seq)
{
    gk_iFreeMatrix(&seq->pssm, seq->len, seq->nsymbols);
    gk_iFreeMatrix(&seq->psfm, seq->len, seq->nsymbols);
    gk_free((void **)&seq->name, &seq->sequence, LTERM);
    //gk_free((void **)&seq, LTERM);
    gk_free((void **) &seq, LTERM);

}
