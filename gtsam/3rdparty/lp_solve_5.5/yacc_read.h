/* prototypes of functions used in the parser */

#ifndef __READ_H__
#define __READ_H__

void lex_fatal_error(char *msg);
int set_title(char *name);
int add_constraint_name(char *name);
int store_re_op(char *yytext, int HadConstraint, int HadVar, int Had_lineair_sum);
void null_tmp_store(int init_Lin_term_count);
int store_bounds(int warn);
void storevarandweight(char *name);
int set_sos_type(int SOStype);
int set_sos_weight(double weight, int sos_decl);
int set_sec_threshold(char *name, REAL threshold);
int rhs_store(REAL value, int HadConstraint, int HadVar, int Had_lineair_sum);
int var_store(char *var, REAL value, int HadConstraint, int HadVar, int Had_lineair_sum);
int negate_constraint(void);
void add_row(void);
void add_sos_row(short SOStype);
void set_obj_dir(int maximise);

void read_error(char *);
void check_int_sec_sos_free_decl(int, int, int, int);
lprec *yacc_read(lprec *lp, int verbose, char *lp_name, int *_lineno, int (*parse) (void), void (*delete_allocated_memory) (void));
#endif
