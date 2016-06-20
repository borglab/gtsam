/*
 * defs.h
 *
 * This file contains various constant definitions
 *
 * Started 8/9/02
 * George
 *
 */

#define CMD_PTYPE               1
#define CMD_OTYPE               2
#define CMD_CTYPE               5
#define CMD_ITYPE               6
#define CMD_RTYPE               7

#define CMD_BALANCE             10
#define CMD_CONTIG              11
#define CMD_MINCONN             12
#define CMD_MINVOL              13

#define CMD_NITER               20
#define CMD_NTRIALS             21
#define CMD_NSEPS               22

#define CMD_TPWGTS              30
#define CMD_SDIFF               31

#define CMD_DEGREE              40
#define CMD_COMPRESS            41

#define CMD_SEED                50

#define CMD_OUTPUT              100
#define CMD_NOOUTPUT            101

#define CMD_DBGLVL              1000
#define CMD_HELP                1001




/* The text labels for PTypes */
static char ptypenames[][15] = {"rb", "kway"};

/* The text labels for ObjTypes */
static char objtypenames[][15] = {"cut", "vol", "node"};

/* The text labels for CTypes */
static char ctypenames[][15] = {"rm", "shem"};

/* The text labels for RTypes */
static char rtypenames[][15] = {"fm", "greedy", "2sided", "1sided"};

/* The text labels for ITypes */
static char iptypenames[][15] = {"grow", "random", "edge", "node", "metisrb"};

/* The text labels for GTypes */
static char gtypenames[][15] = {"dual", "nodal"};
