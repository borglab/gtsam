#include "lp_lib.h"

/* entries for lp structure */
add_column_func               *_add_column;
add_columnex_func             *_add_columnex;
add_constraint_func           *_add_constraint;
add_constraintex_func         *_add_constraintex;
add_lag_con_func              *_add_lag_con;
add_SOS_func                  *_add_SOS;
column_in_lp_func             *_column_in_lp;
copy_lp_func                  *_copy_lp;
default_basis_func            *_default_basis;
del_column_func               *_del_column;
del_constraint_func           *_del_constraint;
delete_lp_func                *_delete_lp;
dualize_lp_func               *_dualize_lp;
free_lp_func                  *_free_lp;
get_anti_degen_func           *_get_anti_degen;
get_basis_func                *_get_basis;
get_basiscrash_func           *_get_basiscrash;
get_bb_depthlimit_func        *_get_bb_depthlimit;
get_bb_floorfirst_func        *_get_bb_floorfirst;
get_bb_rule_func              *_get_bb_rule;
get_bounds_tighter_func       *_get_bounds_tighter;
get_break_at_value_func       *_get_break_at_value;
get_col_name_func             *_get_col_name;
get_columnex_func             *_get_columnex;
get_constr_type_func          *_get_constr_type;
get_constr_value_func         *_get_constr_value;
get_constraints_func          *_get_constraints;
get_dual_solution_func        *_get_dual_solution;
get_epsb_func                 *_get_epsb;
get_epsd_func                 *_get_epsd;
get_epsel_func                *_get_epsel;
get_epsint_func               *_get_epsint;
get_epsperturb_func           *_get_epsperturb;
get_epspivot_func             *_get_epspivot;
get_improve_func              *_get_improve;
get_infinite_func             *_get_infinite;
get_lambda_func               *_get_lambda;
get_lowbo_func                *_get_lowbo;
get_lp_index_func             *_get_lp_index;
get_lp_name_func              *_get_lp_name;
get_Lrows_func                *_get_Lrows;
get_mat_func                  *_get_mat;
get_mat_byindex_func          *_get_mat_byindex;
get_max_level_func            *_get_max_level;
get_maxpivot_func             *_get_maxpivot;
get_mip_gap_func              *_get_mip_gap;
get_multiprice_func           *_get_multiprice;
get_nameindex_func            *_get_nameindex;
get_Ncolumns_func             *_get_Ncolumns;
get_negrange_func             *_get_negrange;
get_nz_func                   *_get_nonzeros;
get_Norig_columns_func        *_get_Norig_columns;
get_Norig_rows_func           *_get_Norig_rows;
get_Nrows_func                *_get_Nrows;
get_obj_bound_func            *_get_obj_bound;
get_objective_func            *_get_objective;
get_orig_index_func           *_get_orig_index;
get_origcol_name_func         *_get_origcol_name;
get_origrow_name_func         *_get_origrow_name;
get_partialprice_func         *_get_partialprice;
get_pivoting_func             *_get_pivoting;
get_presolve_func             *_get_presolve;
get_presolveloops_func        *_get_presolveloops;
get_primal_solution_func      *_get_primal_solution;
get_print_sol_func            *_get_print_sol;
get_pseudocosts_func          *_get_pseudocosts;
get_ptr_constraints_func      *_get_ptr_constraints;
get_ptr_dual_solution_func    *_get_ptr_dual_solution;
get_ptr_lambda_func           *_get_ptr_lambda;
get_ptr_primal_solution_func  *_get_ptr_primal_solution;
get_ptr_sensitivity_obj_func  *_get_ptr_sensitivity_obj;
get_ptr_sensitivity_objex_func *_get_ptr_sensitivity_objex;
get_ptr_sensitivity_rhs_func  *_get_ptr_sensitivity_rhs;
get_ptr_variables_func        *_get_ptr_variables;
get_rh_func                   *_get_rh;
get_rh_range_func             *_get_rh_range;
get_row_func                  *_get_row;
get_row_name_func             *_get_row_name;
get_scalelimit_func           *_get_scalelimit;
get_scaling_func              *_get_scaling;
get_sensitivity_obj_func      *_get_sensitivity_obj;
get_sensitivity_objex_func    *_get_sensitivity_objex;
get_sensitivity_rhs_func      *_get_sensitivity_rhs;
get_simplextype_func          *_get_simplextype;
get_solutioncount_func        *_get_solutioncount;
get_solutionlimit_func        *_get_solutionlimit;
get_status_func               *_get_status;
get_statustext_func           *_get_statustext;
get_timeout_func              *_get_timeout;
get_total_iter_func           *_get_total_iter;
get_total_nodes_func          *_get_total_nodes;
get_upbo_func                 *_get_upbo;
get_var_branch_func           *_get_var_branch;
get_var_dualresult_func       *_get_var_dualresult;
get_var_primalresult_func     *_get_var_primalresult;
get_var_priority_func         *_get_var_priority;
get_variables_func            *_get_variables;
get_verbose_func              *_get_verbose;
get_working_objective_func    *_get_working_objective;
has_BFP_func                  *_has_BFP;
has_XLI_func                  *_has_XLI;
is_add_rowmode_func           *_is_add_rowmode;
is_anti_degen_func            *_is_anti_degen;
is_binary_func                *_is_binary;
is_break_at_first_func        *_is_break_at_first;
is_constr_type_func           *_is_constr_type;
is_debug_func                 *_is_debug;
is_feasible_func              *_is_feasible;
is_unbounded_func             *_is_unbounded;
is_infinite_func              *_is_infinite;
is_int_func                   *_is_int;
is_integerscaling_func        *_is_integerscaling;
is_lag_trace_func             *_is_lag_trace;
is_maxim_func                 *_is_maxim;
is_nativeBFP_func             *_is_nativeBFP;
is_nativeXLI_func             *_is_nativeXLI;
is_negative_func              *_is_negative;
is_piv_mode_func              *_is_piv_mode;
is_piv_rule_func              *_is_piv_rule;
is_presolve_func              *_is_presolve;
is_scalemode_func             *_is_scalemode;
is_scaletype_func             *_is_scaletype;
is_semicont_func              *_is_semicont;
is_SOS_var_func               *_is_SOS_var;
is_trace_func                 *_is_trace;
lp_solve_version_func         *_lp_solve_version;
make_lp_func                  *_make_lp;
print_constraints_func        *_print_constraints;
print_debugdump_func          *_print_debugdump;
print_duals_func              *_print_duals;
print_lp_func                 *_print_lp;
print_objective_func          *_print_objective;
print_scales_func             *_print_scales;
print_solution_func           *_print_solution;
print_str_func                *_print_str;
print_tableau_func            *_print_tableau;
put_abortfunc_func            *_put_abortfunc;
put_bb_nodefunc_func          *_put_bb_nodefunc;
put_bb_branchfunc_func        *_put_bb_branchfunc;
put_logfunc_func              *_put_logfunc;
put_msgfunc_func              *_put_msgfunc;
read_LPhandle_func            *_read_LPhandle;
read_MPShandle_func           *_read_MPShandle;
read_XLI_func                 *_read_XLI;
read_params_func              *_read_params;
read_basis_func               *_read_basis;
reset_basis_func              *_reset_basis;
reset_params_func             *_reset_params;
resize_lp_func                *_resize_lp;
set_add_rowmode_func          *_set_add_rowmode;
set_anti_degen_func           *_set_anti_degen;
set_basisvar_func             *_set_basisvar;
set_basis_func                *_set_basis;
set_basiscrash_func           *_set_basiscrash;
set_bb_depthlimit_func        *_set_bb_depthlimit;
set_bb_floorfirst_func        *_set_bb_floorfirst;
set_bb_rule_func              *_set_bb_rule;
set_BFP_func                  *_set_BFP;
set_binary_func               *_set_binary;
set_bounds_func               *_set_bounds;
set_bounds_tighter_func       *_set_bounds_tighter;
set_break_at_first_func       *_set_break_at_first;
set_break_at_value_func       *_set_break_at_value;
set_column_func               *_set_column;
set_columnex_func             *_set_columnex;
set_col_name_func             *_set_col_name;
set_constr_type_func          *_set_constr_type;
set_debug_func                *_set_debug;
set_epsb_func                 *_set_epsb;
set_epsd_func                 *_set_epsd;
set_epsel_func                *_set_epsel;
set_epsint_func               *_set_epsint;
set_epslevel_func             *_set_epslevel;
set_epsperturb_func           *_set_epsperturb;
set_epspivot_func             *_set_epspivot;
set_unbounded_func            *_set_unbounded;
set_improve_func              *_set_improve;
set_infinite_func             *_set_infinite;
set_int_func                  *_set_int;
set_lag_trace_func            *_set_lag_trace;
set_lowbo_func                *_set_lowbo;
set_lp_name_func              *_set_lp_name;
set_mat_func                  *_set_mat;
set_maxim_func                *_set_maxim;
set_maxpivot_func             *_set_maxpivot;
set_minim_func                *_set_minim;
set_mip_gap_func              *_set_mip_gap;
set_multiprice_func           *_set_multiprice;
set_negrange_func             *_set_negrange;
set_obj_bound_func            *_set_obj_bound;
set_obj_fn_func               *_set_obj_fn;
set_obj_fnex_func             *_set_obj_fnex;
set_obj_func                  *_set_obj;
set_outputfile_func           *_set_outputfile;
set_outputstream_func         *_set_outputstream;
set_partialprice_func         *_set_partialprice;
set_pivoting_func             *_set_pivoting;
set_preferdual_func           *_set_preferdual;
set_presolve_func             *_set_presolve;
set_print_sol_func            *_set_print_sol;
set_pseudocosts_func          *_set_pseudocosts;
set_rh_func                   *_set_rh;
set_rh_range_func             *_set_rh_range;
set_rh_vec_func               *_set_rh_vec;
set_row_func                  *_set_row;
set_rowex_func                *_set_rowex;
set_row_name_func             *_set_row_name;
set_scalelimit_func           *_set_scalelimit;
set_scaling_func              *_set_scaling;
set_semicont_func             *_set_semicont;
set_sense_func                *_set_sense;
set_simplextype_func          *_set_simplextype;
set_solutionlimit_func        *_set_solutionlimit;
set_timeout_func              *_set_timeout;
set_trace_func                *_set_trace;
set_upbo_func                 *_set_upbo;
set_var_branch_func           *_set_var_branch;
set_var_weights_func          *_set_var_weights;
set_verbose_func              *_set_verbose;
set_XLI_func                  *_set_XLI;
solve_func                    *_solve;
str_add_column_func           *_str_add_column;
str_add_constraint_func       *_str_add_constraint;
str_add_lag_con_func          *_str_add_lag_con;
str_set_obj_fn_func           *_str_set_obj_fn;
str_set_rh_vec_func           *_str_set_rh_vec;
time_elapsed_func             *_time_elapsed;
unscale_func                  *_unscale;
write_lp_func                 *_write_lp;
write_LP_func                 *_write_LP;
write_mps_func                *_write_mps;
write_MPS_func                *_write_MPS;
write_freemps_func            *_write_freemps;
write_freeMPS_func            *_write_freeMPS;
write_XLI_func                *_write_XLI;
write_basis_func              *_write_basis;
write_params_func             *_write_params;

#if defined LPSOLVEAPIFROMLPREC

static int init_lpsolve(lprec *lp)
{
  _add_column = lp->add_column;
  _add_columnex = lp->add_columnex;
  _add_constraint = lp->add_constraint;
  _add_constraintex = lp->add_constraintex;
  _add_lag_con = lp->add_lag_con;
  _add_SOS = lp->add_SOS;
  _column_in_lp = lp->column_in_lp;
  _copy_lp = lp->copy_lp;
  _default_basis = lp->default_basis;
  _del_column = lp->del_column;
  _del_constraint = lp->del_constraint;
  _delete_lp = lp->delete_lp;
  _dualize_lp = lp->dualize_lp;
  _free_lp = lp->free_lp;
  _get_anti_degen = lp->get_anti_degen;
  _get_basis = lp->get_basis;
  _get_basiscrash = lp->get_basiscrash;
  _get_bb_depthlimit = lp->get_bb_depthlimit;
  _get_bb_floorfirst = lp->get_bb_floorfirst;
  _get_bb_rule = lp->get_bb_rule;
  _get_bounds_tighter = lp->get_bounds_tighter;
  _get_break_at_value = lp->get_break_at_value;
  _get_col_name = lp->get_col_name;
  _get_columnex = lp->get_columnex;
  _get_constr_type = lp->get_constr_type;
  _get_constr_value = lp->get_constr_value;
  _get_constraints = lp->get_constraints;
  _get_dual_solution = lp->get_dual_solution;
  _get_epsb = lp->get_epsb;
  _get_epsd = lp->get_epsd;
  _get_epsel = lp->get_epsel;
  _get_epsint = lp->get_epsint;
  _get_epsperturb = lp->get_epsperturb;
  _get_epspivot = lp->get_epspivot;
  _get_improve = lp->get_improve;
  _get_infinite = lp->get_infinite;
  _get_lambda = lp->get_lambda;
  _get_lowbo = lp->get_lowbo;
  _get_lp_index = lp->get_lp_index;
  _get_lp_name = lp->get_lp_name;
  _get_Lrows = lp->get_Lrows;
  _get_mat = lp->get_mat;
  _get_mat_byindex = lp->get_mat_byindex;
  _get_max_level = lp->get_max_level;
  _get_maxpivot = lp->get_maxpivot;
  _get_mip_gap = lp->get_mip_gap;
  _get_multiprice = lp->get_multiprice;
  _get_nameindex = lp->get_nameindex;
  _get_Ncolumns = lp->get_Ncolumns;
  _get_negrange = lp->get_negrange;
  _get_nonzeros = lp->get_nonzeros;
  _get_Norig_columns = lp->get_Norig_columns;
  _get_Norig_rows = lp->get_Norig_rows;
  _get_Nrows = lp->get_Nrows;
  _get_obj_bound = lp->get_obj_bound;
  _get_objective = lp->get_objective;
  _get_orig_index = lp->get_orig_index;
  _get_origcol_name = lp->get_origcol_name;
  _get_origrow_name = lp->get_origrow_name;
  _get_partialprice = lp->get_partialprice;
  _get_pivoting = lp->get_pivoting;
  _get_presolve = lp->get_presolve;
  _get_presolveloops = lp->get_presolveloops;
  _get_primal_solution = lp->get_primal_solution;
  _get_print_sol = lp->get_print_sol;
  _get_pseudocosts = lp->get_pseudocosts;
  _get_ptr_constraints = lp->get_ptr_constraints;
  _get_ptr_dual_solution = lp->get_ptr_dual_solution;
  _get_ptr_lambda = lp->get_ptr_lambda;
  _get_ptr_primal_solution = lp->get_ptr_primal_solution;
  _get_ptr_sensitivity_obj = lp->get_ptr_sensitivity_obj;
  _get_ptr_sensitivity_objex = lp->get_ptr_sensitivity_objex;
  _get_ptr_sensitivity_rhs = lp->get_ptr_sensitivity_rhs;
  _get_ptr_variables = lp->get_ptr_variables;
  _get_rh = lp->get_rh;
  _get_rh_range = lp->get_rh_range;
  _get_row = lp->get_row;
  _get_row_name = lp->get_row_name;
  _get_scalelimit = lp->get_scalelimit;
  _get_scaling = lp->get_scaling;
  _get_sensitivity_obj = lp->get_sensitivity_obj;
  _get_sensitivity_objex = lp->get_sensitivity_objex;
  _get_sensitivity_rhs = lp->get_sensitivity_rhs;
  _get_simplextype = lp->get_simplextype;
  _get_solutioncount = lp->get_solutioncount;
  _get_solutionlimit = lp->get_solutionlimit;
  _get_status = lp->get_status;
  _get_statustext = lp->get_statustext;
  _get_timeout = lp->get_timeout;
  _get_total_iter = lp->get_total_iter;
  _get_total_nodes = lp->get_total_nodes;
  _get_upbo = lp->get_upbo;
  _get_var_branch = lp->get_var_branch;
  _get_var_dualresult = lp->get_var_dualresult;
  _get_var_primalresult = lp->get_var_primalresult;
  _get_var_priority = lp->get_var_priority;
  _get_variables = lp->get_variables;
  _get_verbose = lp->get_verbose;
  _get_working_objective = lp->get_working_objective;
  _has_BFP = lp->has_BFP;
  _has_XLI = lp->has_XLI;
  _is_add_rowmode = lp->is_add_rowmode;
  _is_anti_degen = lp->is_anti_degen;
  _is_binary = lp->is_binary;
  _is_break_at_first = lp->is_break_at_first;
  _is_constr_type = lp->is_constr_type;
  _is_debug = lp->is_debug;
  _is_feasible = lp->is_feasible;
  _is_unbounded = lp->is_unbounded;
  _is_infinite = lp->is_infinite;
  _is_int = lp->is_int;
  _is_integerscaling = lp->is_integerscaling;
  _is_lag_trace = lp->is_lag_trace;
  _is_maxim = lp->is_maxim;
  _is_nativeBFP = lp->is_nativeBFP;
  _is_nativeXLI = lp->is_nativeXLI;
  _is_negative = lp->is_negative;
  _is_piv_mode = lp->is_piv_mode;
  _is_piv_rule = lp->is_piv_rule;
  _is_presolve = lp->is_presolve;
  _is_scalemode = lp->is_scalemode;
  _is_scaletype = lp->is_scaletype;
  _is_semicont = lp->is_semicont;
  _is_SOS_var = lp->is_SOS_var;
  _is_trace = lp->is_trace;
  _lp_solve_version = lp->lp_solve_version;
  _make_lp = lp->make_lp;
  _print_constraints = lp->print_constraints;
  _print_debugdump = lp->print_debugdump;
  _print_duals = lp->print_duals;
  _print_lp = lp->print_lp;
  _print_objective = lp->print_objective;
  _print_scales = lp->print_scales;
  _print_solution = lp->print_solution;
  _print_str = lp->print_str;
  _print_tableau = lp->print_tableau;
  _put_abortfunc = lp->put_abortfunc;
  _put_bb_nodefunc = lp->put_bb_nodefunc;
  _put_bb_branchfunc = lp->put_bb_branchfunc;
  _put_logfunc = lp->put_logfunc;
  _put_msgfunc = lp->put_msgfunc;
  _read_LPhandle = lp->read_LPhandle;
  _read_MPShandle = lp->read_MPShandle;
  _read_XLI = lp->read_XLI;
  _read_params = lp->read_params;
  _read_basis = lp->read_basis;
  _reset_basis = lp->reset_basis;
  _reset_params = lp->reset_params;
  _resize_lp = lp->resize_lp;
  _set_add_rowmode = lp->set_add_rowmode;
  _set_anti_degen = lp->set_anti_degen;
  _set_basisvar = lp->set_basisvar;
  _set_basis = lp->set_basis;
  _set_basiscrash = lp->set_basiscrash;
  _set_bb_depthlimit = lp->set_bb_depthlimit;
  _set_bb_floorfirst = lp->set_bb_floorfirst;
  _set_bb_rule = lp->set_bb_rule;
  _set_BFP = lp->set_BFP;
  _set_binary = lp->set_binary;
  _set_bounds = lp->set_bounds;
  _set_bounds_tighter = lp->set_bounds_tighter;
  _set_break_at_first = lp->set_break_at_first;
  _set_break_at_value = lp->set_break_at_value;
  _set_column = lp->set_column;
  _set_columnex = lp->set_columnex;
  _set_col_name = lp->set_col_name;
  _set_constr_type = lp->set_constr_type;
  _set_debug = lp->set_debug;
  _set_epsb = lp->set_epsb;
  _set_epsd = lp->set_epsd;
  _set_epsel = lp->set_epsel;
  _set_epsint = lp->set_epsint;
  _set_epslevel = lp->set_epslevel;
  _set_epsperturb = lp->set_epsperturb;
  _set_epspivot = lp->set_epspivot;
  _set_unbounded = lp->set_unbounded;
  _set_improve = lp->set_improve;
  _set_infinite = lp->set_infinite;
  _set_int = lp->set_int;
  _set_lag_trace = lp->set_lag_trace;
  _set_lowbo = lp->set_lowbo;
  _set_lp_name = lp->set_lp_name;
  _set_mat = lp->set_mat;
  _set_maxim = lp->set_maxim;
  _set_maxpivot = lp->set_maxpivot;
  _set_minim = lp->set_minim;
  _set_mip_gap = lp->set_mip_gap;
  _set_multiprice = lp->set_multiprice;
  _set_negrange = lp->set_negrange;
  _set_obj_bound = lp->set_obj_bound;
  _set_obj_fn = lp->set_obj_fn;
  _set_obj_fnex = lp->set_obj_fnex;
  _set_obj = lp->set_obj;
  _set_outputfile = lp->set_outputfile;
  _set_outputstream = lp->set_outputstream;
  _set_partialprice = lp->set_partialprice;
  _set_pivoting = lp->set_pivoting;
  _set_preferdual = lp->set_preferdual;
  _set_presolve = lp->set_presolve;
  _set_print_sol = lp->set_print_sol;
  _set_pseudocosts = lp->set_pseudocosts;
  _set_rh = lp->set_rh;
  _set_rh_range = lp->set_rh_range;
  _set_rh_vec = lp->set_rh_vec;
  _set_row = lp->set_row;
  _set_rowex = lp->set_rowex;
  _set_row_name = lp->set_row_name;
  _set_scalelimit = lp->set_scalelimit;
  _set_scaling = lp->set_scaling;
  _set_semicont = lp->set_semicont;
  _set_sense = lp->set_sense;
  _set_simplextype = lp->set_simplextype;
  _set_solutionlimit = lp->set_solutionlimit;
  _set_timeout = lp->set_timeout;
  _set_trace = lp->set_trace;
  _set_upbo = lp->set_upbo;
  _set_var_branch = lp->set_var_branch;
  _set_var_weights = lp->set_var_weights;
  _set_verbose = lp->set_verbose;
  _set_XLI = lp->set_XLI;
  _solve = lp->solve;
  _str_add_column = lp->str_add_column;
  _str_add_constraint = lp->str_add_constraint;
  _str_add_lag_con = lp->str_add_lag_con;
  _str_set_obj_fn = lp->str_set_obj_fn;
  _str_set_rh_vec = lp->str_set_rh_vec;
  _time_elapsed = lp->time_elapsed;
  _unscale = lp->unscale;
  _write_lp = lp->write_lp;
  _write_LP = lp->write_LP;
  _write_mps = lp->write_mps;
  _write_MPS = lp->write_MPS;
  _write_freemps = lp->write_freemps;
  _write_freeMPS = lp->write_freeMPS;
  _write_XLI = lp->write_XLI;
  _write_basis = lp->write_basis;
  _write_params = lp->write_params;

  return(TRUE);
}

#elif defined LPSOLVEAPIFROMLIB

#ifdef WIN32
#  include <windows.h>
#else
#  include <dlfcn.h>
#endif

#if defined WIN32
# define hlpsolve HINSTANCE
#else
# define hlpsolve void *
#endif

static hlpsolve open_lpsolve_lib(char *filename)
{
  hlpsolve lpsolve;

# if defined WIN32
  /* Get a handle to the Windows DLL module. */
  lpsolve = LoadLibrary("lpsolve55.dll");
# else
  lpsolve = dlopen("liblpsolve55.so", RTLD_LAZY);;
# endif
  return(lpsolve);
}

static int close_lpsolve_lib(hlpsolve lpsolve)
{
#ifdef WIN32
  FreeLibrary(lpsolve);
#else
  dlclose(lpsolve);
#endif

  return(TRUE);
}

static int init_lpsolve(hlpsolve lpsolve)
{
# if defined WIN32
#   define AddressOf GetProcAddress
# else
#   define AddressOf dlsym
# endif

  /* assign API functions to lp structure */
  _add_column = (add_column_func *) AddressOf(lpsolve, "add_column");
  _add_columnex = (add_columnex_func *) AddressOf(lpsolve, "add_columnex");
  _add_constraint = (add_constraint_func *) AddressOf(lpsolve, "add_constraint");
  _add_constraintex = (add_constraintex_func *) AddressOf(lpsolve, "add_constraintex");
  _add_lag_con = (add_lag_con_func *) AddressOf(lpsolve, "add_lag_con");
  _add_SOS = (add_SOS_func *) AddressOf(lpsolve, "add_SOS");
  _column_in_lp = (column_in_lp_func *) AddressOf(lpsolve, "column_in_lp");
  _copy_lp = (copy_lp_func *) AddressOf(lpsolve, "copy_lp");
  _default_basis = (default_basis_func *) AddressOf(lpsolve, "default_basis");
  _del_column = (del_column_func *) AddressOf(lpsolve, "del_column");
  _del_constraint = (del_constraint_func *) AddressOf(lpsolve, "del_constraint");
  _delete_lp = (delete_lp_func *) AddressOf(lpsolve, "delete_lp");
  _dualize_lp = (dualize_lp_func *) AddressOf(lpsolve, "dualize_lp");
  _free_lp = (free_lp_func *) AddressOf(lpsolve, "free_lp");
  _get_anti_degen = (get_anti_degen_func *) AddressOf(lpsolve, "get_anti_degen");
  _get_basis = (get_basis_func *) AddressOf(lpsolve, "get_basis");
  _get_basiscrash = (get_basiscrash_func *) AddressOf(lpsolve, "get_basiscrash");
  _get_bb_depthlimit = (get_bb_depthlimit_func *) AddressOf(lpsolve, "get_bb_depthlimit");
  _get_bb_floorfirst = (get_bb_floorfirst_func *) AddressOf(lpsolve, "get_bb_floorfirst");
  _get_bb_rule = (get_bb_rule_func *) AddressOf(lpsolve, "get_bb_rule");
  _get_bounds_tighter = (get_bounds_tighter_func *) AddressOf(lpsolve, "get_bounds_tighter");
  _get_break_at_value = (get_break_at_value_func *) AddressOf(lpsolve, "get_break_at_value");
  _get_col_name = (get_col_name_func *) AddressOf(lpsolve, "get_col_name");
  _get_columnex = (get_columnex_func *) AddressOf(lpsolve, "get_columnex");
  _get_constr_type = (get_constr_type_func *) AddressOf(lpsolve, "get_constr_type");
  _get_constr_value = (get_constr_value_func *) AddressOf(lpsolve, "get_constr_value");
  _get_constraints = (get_constraints_func *) AddressOf(lpsolve, "get_constraints");
  _get_dual_solution = (get_dual_solution_func *) AddressOf(lpsolve, "get_dual_solution");
  _get_epsb = (get_epsb_func *) AddressOf(lpsolve, "get_epsb");
  _get_epsd = (get_epsd_func *) AddressOf(lpsolve, "get_epsd");
  _get_epsel = (get_epsel_func *) AddressOf(lpsolve, "get_epsel");
  _get_epsint = (get_epsint_func *) AddressOf(lpsolve, "get_epsint");
  _get_epsperturb = (get_epsperturb_func *) AddressOf(lpsolve, "get_epsperturb");
  _get_epspivot = (get_epspivot_func *) AddressOf(lpsolve, "get_epspivot");
  _get_improve = (get_improve_func *) AddressOf(lpsolve, "get_improve");
  _get_infinite = (get_infinite_func *) AddressOf(lpsolve, "get_infinite");
  _get_lambda = (get_lambda_func *) AddressOf(lpsolve, "get_lambda");
  _get_lowbo = (get_lowbo_func *) AddressOf(lpsolve, "get_lowbo");
  _get_lp_index = (get_lp_index_func *) AddressOf(lpsolve, "get_lp_index");
  _get_lp_name = (get_lp_name_func *) AddressOf(lpsolve, "get_lp_name");
  _get_Lrows = (get_Lrows_func *) AddressOf(lpsolve, "get_Lrows");
  _get_mat = (get_mat_func *) AddressOf(lpsolve, "get_mat");
  _get_mat_byindex = (get_mat_byindex_func *) AddressOf(lpsolve, "get_mat_byindex");
  _get_max_level = (get_max_level_func *) AddressOf(lpsolve, "get_max_level");
  _get_maxpivot = (get_maxpivot_func *) AddressOf(lpsolve, "get_maxpivot");
  _get_mip_gap = (get_mip_gap_func *) AddressOf(lpsolve, "get_mip_gap");
  _get_multiprice = (get_multiprice_func *) AddressOf(lpsolve, "get_multiprice");
  _get_nameindex = (get_nameindex_func *) AddressOf(lpsolve, "get_nameindex");
  _get_Ncolumns = (get_Ncolumns_func *) AddressOf(lpsolve, "get_Ncolumns");
  _get_negrange = (get_negrange_func *) AddressOf(lpsolve, "get_negrange");
  _get_nonzeros = (get_nz_func *) AddressOf(lpsolve, "get_nonzeros");
  _get_Norig_columns = (get_Norig_columns_func *) AddressOf(lpsolve, "get_Norig_columns");
  _get_Norig_rows = (get_Norig_rows_func *) AddressOf(lpsolve, "get_Norig_rows");
  _get_Nrows = (get_Nrows_func *) AddressOf(lpsolve, "get_Nrows");
  _get_obj_bound = (get_obj_bound_func *) AddressOf(lpsolve, "get_obj_bound");
  _get_objective = (get_objective_func *) AddressOf(lpsolve, "get_objective");
  _get_orig_index = (get_orig_index_func *) AddressOf(lpsolve, "get_orig_index");
  _get_origcol_name = (get_origcol_name_func *) AddressOf(lpsolve, "get_origcol_name");
  _get_origrow_name = (get_origrow_name_func *) AddressOf(lpsolve, "get_origrow_name");
  _get_partialprice = (get_partialprice_func *) AddressOf(lpsolve, "get_partialprice");
  _get_pivoting = (get_pivoting_func *) AddressOf(lpsolve, "get_pivoting");
  _get_presolve = (get_presolve_func *) AddressOf(lpsolve, "get_presolve");
  _get_presolveloops = (get_presolveloops_func *) AddressOf(lpsolve, "get_presolveloops");
  _get_primal_solution = (get_primal_solution_func *) AddressOf(lpsolve, "get_primal_solution");
  _get_print_sol = (get_print_sol_func *) AddressOf(lpsolve, "get_print_sol");
  _get_pseudocosts = (get_pseudocosts_func *) AddressOf(lpsolve, "get_pseudocosts");
  _get_ptr_constraints = (get_ptr_constraints_func *) AddressOf(lpsolve, "get_ptr_constraints");
  _get_ptr_dual_solution = (get_ptr_dual_solution_func *) AddressOf(lpsolve, "get_ptr_dual_solution");
  _get_ptr_lambda = (get_ptr_lambda_func *) AddressOf(lpsolve, "get_ptr_lambda");
  _get_ptr_primal_solution = (get_ptr_primal_solution_func *) AddressOf(lpsolve, "get_ptr_primal_solution");
  _get_ptr_sensitivity_obj = (get_ptr_sensitivity_obj_func *) AddressOf(lpsolve, "get_ptr_sensitivity_obj");
  _get_ptr_sensitivity_objex = (get_ptr_sensitivity_objex_func *) AddressOf(lpsolve, "get_ptr_sensitivity_objex");
  _get_ptr_sensitivity_rhs = (get_ptr_sensitivity_rhs_func *) AddressOf(lpsolve, "get_ptr_sensitivity_rhs");
  _get_ptr_variables = (get_ptr_variables_func *) AddressOf(lpsolve, "get_ptr_variables");
  _get_rh = (get_rh_func *) AddressOf(lpsolve, "get_rh");
  _get_rh_range = (get_rh_range_func *) AddressOf(lpsolve, "get_rh_range");
  _get_row = (get_row_func *) AddressOf(lpsolve, "get_row");
  _get_row_name = (get_row_name_func *) AddressOf(lpsolve, "get_row_name");
  _get_scalelimit = (get_scalelimit_func *) AddressOf(lpsolve, "get_scalelimit");
  _get_scaling = (get_scaling_func *) AddressOf(lpsolve, "get_scaling");
  _get_sensitivity_obj = (get_sensitivity_obj_func *) AddressOf(lpsolve, "get_sensitivity_obj");
  _get_sensitivity_objex = (get_sensitivity_objex_func *) AddressOf(lpsolve, "get_sensitivity_objex");
  _get_sensitivity_rhs = (get_sensitivity_rhs_func *) AddressOf(lpsolve, "get_sensitivity_rhs");
  _get_simplextype = (get_simplextype_func *) AddressOf(lpsolve, "get_simplextype");
  _get_solutioncount = (get_solutioncount_func *) AddressOf(lpsolve, "get_solutioncount");
  _get_solutionlimit = (get_solutionlimit_func *) AddressOf(lpsolve, "get_solutionlimit");
  _get_status = (get_status_func *) AddressOf(lpsolve, "get_status");
  _get_statustext = (get_statustext_func *) AddressOf(lpsolve, "get_statustext");
  _get_timeout = (get_timeout_func *) AddressOf(lpsolve, "get_timeout");
  _get_total_iter = (get_total_iter_func *) AddressOf(lpsolve, "get_total_iter");
  _get_total_nodes = (get_total_nodes_func *) AddressOf(lpsolve, "get_total_nodes");
  _get_upbo = (get_upbo_func *) AddressOf(lpsolve, "get_upbo");
  _get_var_branch = (get_var_branch_func *) AddressOf(lpsolve, "get_var_branch");
  _get_var_dualresult = (get_var_dualresult_func *) AddressOf(lpsolve, "get_var_dualresult");
  _get_var_primalresult = (get_var_primalresult_func *) AddressOf(lpsolve, "get_var_primalresult");
  _get_var_priority = (get_var_priority_func *) AddressOf(lpsolve, "get_var_priority");
  _get_variables = (get_variables_func *) AddressOf(lpsolve, "get_variables");
  _get_verbose = (get_verbose_func *) AddressOf(lpsolve, "get_verbose");
  _get_working_objective = (get_working_objective_func *) AddressOf(lpsolve, "get_working_objective");
  _has_BFP = (has_BFP_func *) AddressOf(lpsolve, "has_BFP");
  _has_XLI = (has_XLI_func *) AddressOf(lpsolve, "has_XLI");
  _is_add_rowmode = (is_add_rowmode_func *) AddressOf(lpsolve, "is_add_rowmode");
  _is_anti_degen = (is_anti_degen_func *) AddressOf(lpsolve, "is_anti_degen");
  _is_binary = (is_binary_func *) AddressOf(lpsolve, "is_binary");
  _is_break_at_first = (is_break_at_first_func *) AddressOf(lpsolve, "is_break_at_first");
  _is_constr_type = (is_constr_type_func *) AddressOf(lpsolve, "is_constr_type");
  _is_debug = (is_debug_func *) AddressOf(lpsolve, "is_debug");
  _is_feasible = (is_feasible_func *) AddressOf(lpsolve, "is_feasible");
  _is_unbounded = (is_unbounded_func *) AddressOf(lpsolve, "is_unbounded");
  _is_infinite = (is_infinite_func *) AddressOf(lpsolve, "is_infinite");
  _is_int = (is_int_func *) AddressOf(lpsolve, "is_int");
  _is_integerscaling = (is_integerscaling_func *) AddressOf(lpsolve, "is_integerscaling");
  _is_lag_trace = (is_lag_trace_func *) AddressOf(lpsolve, "is_lag_trace");
  _is_maxim = (is_maxim_func *) AddressOf(lpsolve, "is_maxim");
  _is_nativeBFP = (is_nativeBFP_func *) AddressOf(lpsolve, "is_nativeBFP");
  _is_nativeXLI = (is_nativeXLI_func *) AddressOf(lpsolve, "is_nativeXLI");
  _is_negative = (is_negative_func *) AddressOf(lpsolve, "is_negative");
  _is_piv_mode = (is_piv_mode_func *) AddressOf(lpsolve, "is_piv_mode");
  _is_piv_rule = (is_piv_rule_func *) AddressOf(lpsolve, "is_piv_rule");
  _is_presolve = (is_presolve_func *) AddressOf(lpsolve, "is_presolve");
  _is_scalemode = (is_scalemode_func *) AddressOf(lpsolve, "is_scalemode");
  _is_scaletype = (is_scaletype_func *) AddressOf(lpsolve, "is_scaletype");
  _is_semicont = (is_semicont_func *) AddressOf(lpsolve, "is_semicont");
  _is_SOS_var = (is_SOS_var_func *) AddressOf(lpsolve, "is_SOS_var");
  _is_trace = (is_trace_func *) AddressOf(lpsolve, "is_trace");
  _lp_solve_version = (lp_solve_version_func *) AddressOf(lpsolve, "lp_solve_version");
  _make_lp = (make_lp_func *) AddressOf(lpsolve, "make_lp");
  _print_constraints = (print_constraints_func *) AddressOf(lpsolve, "print_constraints");
  _print_debugdump = (print_debugdump_func *) AddressOf(lpsolve, "print_debugdump");
  _print_duals = (print_duals_func *) AddressOf(lpsolve, "print_duals");
  _print_lp = (print_lp_func *) AddressOf(lpsolve, "print_lp");
  _print_objective = (print_objective_func *) AddressOf(lpsolve, "print_objective");
  _print_scales = (print_scales_func *) AddressOf(lpsolve, "print_scales");
  _print_solution = (print_solution_func *) AddressOf(lpsolve, "print_solution");
  _print_str = (print_str_func *) AddressOf(lpsolve, "print_str");
  _print_tableau = (print_tableau_func *) AddressOf(lpsolve, "print_tableau");
  _put_abortfunc = (put_abortfunc_func *) AddressOf(lpsolve, "put_abortfunc");
  _put_bb_nodefunc = (put_bb_nodefunc_func *) AddressOf(lpsolve, "put_bb_nodefunc");
  _put_bb_branchfunc = (put_bb_branchfunc_func *) AddressOf(lpsolve, "put_bb_branchfunc");
  _put_logfunc = (put_logfunc_func *) AddressOf(lpsolve, "put_logfunc");
  _put_msgfunc = (put_msgfunc_func *) AddressOf(lpsolve, "put_msgfunc");
  _read_LPhandle = (read_LPhandle_func *) AddressOf(lpsolve, "read_LPhandle");
  _read_MPShandle = (read_MPShandle_func *) AddressOf(lpsolve, "read_MPShandle");
  _read_XLI = (read_XLI_func *) AddressOf(lpsolve, "read_XLI");
  _read_params = (read_params_func *) AddressOf(lpsolve, "read_params");
  _read_basis = (read_basis_func *) AddressOf(lpsolve, "read_basis");
  _reset_basis = (reset_basis_func *) AddressOf(lpsolve, "reset_basis");
  _reset_params = (reset_params_func *) AddressOf(lpsolve, "reset_params");
  _resize_lp = (resize_lp_func *) AddressOf(lpsolve, "resize_lp");
  _set_add_rowmode = (set_add_rowmode_func *) AddressOf(lpsolve, "set_add_rowmode");
  _set_anti_degen = (set_anti_degen_func *) AddressOf(lpsolve, "set_anti_degen");
  _set_basisvar = (set_basisvar_func *) AddressOf(lpsolve, "set_basisvar");
  _set_basis = (set_basis_func *) AddressOf(lpsolve, "set_basis");
  _set_basiscrash = (set_basiscrash_func *) AddressOf(lpsolve, "set_basiscrash");
  _set_bb_depthlimit = (set_bb_depthlimit_func *) AddressOf(lpsolve, "set_bb_depthlimit");
  _set_bb_floorfirst = (set_bb_floorfirst_func *) AddressOf(lpsolve, "set_bb_floorfirst");
  _set_bb_rule = (set_bb_rule_func *) AddressOf(lpsolve, "set_bb_rule");
  _set_BFP = (set_BFP_func *) AddressOf(lpsolve, "set_BFP");
  _set_binary = (set_binary_func *) AddressOf(lpsolve, "set_binary");
  _set_bounds = (set_bounds_func *) AddressOf(lpsolve, "set_bounds");
  _set_bounds_tighter = (set_bounds_tighter_func *) AddressOf(lpsolve, "set_bounds_tighter");
  _set_break_at_first = (set_break_at_first_func *) AddressOf(lpsolve, "set_break_at_first");
  _set_break_at_value = (set_break_at_value_func *) AddressOf(lpsolve, "set_break_at_value");
  _set_column = (set_column_func *) AddressOf(lpsolve, "set_column");
  _set_columnex = (set_columnex_func *) AddressOf(lpsolve, "set_columnex");
  _set_col_name = (set_col_name_func *) AddressOf(lpsolve, "set_col_name");
  _set_constr_type = (set_constr_type_func *) AddressOf(lpsolve, "set_constr_type");
  _set_debug = (set_debug_func *) AddressOf(lpsolve, "set_debug");
  _set_epsb = (set_epsb_func *) AddressOf(lpsolve, "set_epsb");
  _set_epsd = (set_epsd_func *) AddressOf(lpsolve, "set_epsd");
  _set_epsel = (set_epsel_func *) AddressOf(lpsolve, "set_epsel");
  _set_epsint = (set_epsint_func *) AddressOf(lpsolve, "set_epsint");
  _set_epslevel = (set_epslevel_func *) AddressOf(lpsolve, "set_epslevel");
  _set_epsperturb = (set_epsperturb_func *) AddressOf(lpsolve, "set_epsperturb");
  _set_epspivot = (set_epspivot_func *) AddressOf(lpsolve, "set_epspivot");
  _set_unbounded = (set_unbounded_func *) AddressOf(lpsolve, "set_unbounded");
  _set_improve = (set_improve_func *) AddressOf(lpsolve, "set_improve");
  _set_infinite = (set_infinite_func *) AddressOf(lpsolve, "set_infinite");
  _set_int = (set_int_func *) AddressOf(lpsolve, "set_int");
  _set_lag_trace = (set_lag_trace_func *) AddressOf(lpsolve, "set_lag_trace");
  _set_lowbo = (set_lowbo_func *) AddressOf(lpsolve, "set_lowbo");
  _set_lp_name = (set_lp_name_func *) AddressOf(lpsolve, "set_lp_name");
  _set_mat = (set_mat_func *) AddressOf(lpsolve, "set_mat");
  _set_maxim = (set_maxim_func *) AddressOf(lpsolve, "set_maxim");
  _set_maxpivot = (set_maxpivot_func *) AddressOf(lpsolve, "set_maxpivot");
  _set_minim = (set_minim_func *) AddressOf(lpsolve, "set_minim");
  _set_mip_gap = (set_mip_gap_func *) AddressOf(lpsolve, "set_mip_gap");
  _set_multiprice = (set_multiprice_func *) AddressOf(lpsolve, "set_multiprice");
  _set_negrange = (set_negrange_func *) AddressOf(lpsolve, "set_negrange");
  _set_obj_bound = (set_obj_bound_func *) AddressOf(lpsolve, "set_obj_bound");
  _set_obj_fn = (set_obj_fn_func *) AddressOf(lpsolve, "set_obj_fn");
  _set_obj_fnex = (set_obj_fnex_func *) AddressOf(lpsolve, "set_obj_fnex");
  _set_obj = (set_obj_func *) AddressOf(lpsolve, "set_obj");
  _set_outputfile = (set_outputfile_func *) AddressOf(lpsolve, "set_outputfile");
  _set_outputstream = (set_outputstream_func *) AddressOf(lpsolve, "set_outputstream");
  _set_partialprice = (set_partialprice_func *) AddressOf(lpsolve, "set_partialprice");
  _set_pivoting = (set_pivoting_func *) AddressOf(lpsolve, "set_pivoting");
  _set_preferdual = (set_preferdual_func *) AddressOf(lpsolve, "set_preferdual");
  _set_presolve = (set_presolve_func *) AddressOf(lpsolve, "set_presolve");
  _set_print_sol = (set_print_sol_func *) AddressOf(lpsolve, "set_print_sol");
  _set_pseudocosts = (set_pseudocosts_func *) AddressOf(lpsolve, "set_pseudocosts");
  _set_rh = (set_rh_func *) AddressOf(lpsolve, "set_rh");
  _set_rh_range = (set_rh_range_func *) AddressOf(lpsolve, "set_rh_range");
  _set_rh_vec = (set_rh_vec_func *) AddressOf(lpsolve, "set_rh_vec");
  _set_row = (set_row_func *) AddressOf(lpsolve, "set_row");
  _set_rowex = (set_rowex_func *) AddressOf(lpsolve, "set_rowex");
  _set_row_name = (set_row_name_func *) AddressOf(lpsolve, "set_row_name");
  _set_scalelimit = (set_scalelimit_func *) AddressOf(lpsolve, "set_scalelimit");
  _set_scaling = (set_scaling_func *) AddressOf(lpsolve, "set_scaling");
  _set_semicont = (set_semicont_func *) AddressOf(lpsolve, "set_semicont");
  _set_sense = (set_sense_func *) AddressOf(lpsolve, "set_sense");
  _set_simplextype = (set_simplextype_func *) AddressOf(lpsolve, "set_simplextype");
  _set_solutionlimit = (set_solutionlimit_func *) AddressOf(lpsolve, "set_solutionlimit");
  _set_timeout = (set_timeout_func *) AddressOf(lpsolve, "set_timeout");
  _set_trace = (set_trace_func *) AddressOf(lpsolve, "set_trace");
  _set_upbo = (set_upbo_func *) AddressOf(lpsolve, "set_upbo");
  _set_var_branch = (set_var_branch_func *) AddressOf(lpsolve, "set_var_branch");
  _set_var_weights = (set_var_weights_func *) AddressOf(lpsolve, "set_var_weights");
  _set_verbose = (set_verbose_func *) AddressOf(lpsolve, "set_verbose");
  _set_XLI = (set_XLI_func *) AddressOf(lpsolve, "set_XLI");
  _solve = (solve_func *) AddressOf(lpsolve, "solve");
  _str_add_column = (str_add_column_func *) AddressOf(lpsolve, "str_add_column");
  _str_add_constraint = (str_add_constraint_func *) AddressOf(lpsolve, "str_add_constraint");
  _str_add_lag_con = (str_add_lag_con_func *) AddressOf(lpsolve, "str_add_lag_con");
  _str_set_obj_fn = (str_set_obj_fn_func *) AddressOf(lpsolve, "str_set_obj_fn");
  _str_set_rh_vec = (str_set_rh_vec_func *) AddressOf(lpsolve, "str_set_rh_vec");
  _time_elapsed = (time_elapsed_func *) AddressOf(lpsolve, "time_elapsed");
  _unscale = (unscale_func *) AddressOf(lpsolve, "unscale");
  _write_lp = (write_lp_func *) AddressOf(lpsolve, "write_lp");
  _write_LP = (write_LP_func *) AddressOf(lpsolve, "write_LP");
  _write_mps = (write_mps_func *) AddressOf(lpsolve, "write_mps");
  _write_MPS = (write_MPS_func *) AddressOf(lpsolve, "write_MPS");
  _write_freemps = (write_freemps_func *) AddressOf(lpsolve, "write_freemps");
  _write_freeMPS = (write_freeMPS_func *) AddressOf(lpsolve, "write_freeMPS");
  _write_XLI = (write_XLI_func *) AddressOf(lpsolve, "write_XLI");
  _write_basis = (write_basis_func *) AddressOf(lpsolve, "write_basis");
  _write_params = (write_params_func *) AddressOf(lpsolve, "write_params");

  return(TRUE);
# undef AddressOf
}

#else
#  error Either LPSOLVEAPIFROMLPREC or LPSOLVEAPIFROMLIB must be defined
#endif

#define add_column _add_column
#define add_columnex _add_columnex
#define add_constraint _add_constraint
#define add_constraintex _add_constraintex
#define add_lag_con _add_lag_con
#define add_SOS _add_SOS
#define column_in_lp _column_in_lp
#define copy_lp _copy_lp
#define default_basis _default_basis
#define del_column _del_column
#define del_constraint _del_constraint
#define delete_lp _delete_lp
#define dualize_lp _dualize_lp
#define free_lp _free_lp
#define get_anti_degen _get_anti_degen
#define get_basis _get_basis
#define get_basiscrash _get_basiscrash
#define get_bb_depthlimit _get_bb_depthlimit
#define get_bb_floorfirst _get_bb_floorfirst
#define get_bb_rule _get_bb_rule
#define get_bounds_tighter _get_bounds_tighter
#define get_break_at_value _get_break_at_value
#define get_col_name _get_col_name
#define get_columnex _get_columnex
#define get_constr_type _get_constr_type
#define get_constr_value _get_constr_value
#define get_constraints _get_constraints
#define get_dual_solution _get_dual_solution
#define get_epsb _get_epsb
#define get_epsd _get_epsd
#define get_epsel _get_epsel
#define get_epsint _get_epsint
#define get_epsperturb _get_epsperturb
#define get_epspivot _get_epspivot
#define get_improve _get_improve
#define get_infinite _get_infinite
#define get_lambda _get_lambda
#define get_lowbo _get_lowbo
#define get_lp_index _get_lp_index
#define get_lp_name _get_lp_name
#define get_Lrows _get_Lrows
#define get_mat _get_mat
#define get_mat_byindex _get_mat_byindex
#define get_max_level _get_max_level
#define get_maxpivot _get_maxpivot
#define get_mip_gap _get_mip_gap
#define get_multiprice _get_multiprice
#define get_nameindex _get_nameindex
#define get_Ncolumns _get_Ncolumns
#define get_negrange _get_negrange
#define get_nonzeros _get_nonzeros
#define get_Norig_columns _get_Norig_columns
#define get_Norig_rows _get_Norig_rows
#define get_Nrows _get_Nrows
#define get_obj_bound _get_obj_bound
#define get_objective _get_objective
#define get_orig_index _get_orig_index
#define get_origcol_name _get_origcol_name
#define get_origrow_name _get_origrow_name
#define get_partialprice _get_partialprice
#define get_pivoting _get_pivoting
#define get_presolve _get_presolve
#define get_presolveloops _get_presolveloops
#define get_primal_solution _get_primal_solution
#define get_print_sol _get_print_sol
#define get_pseudocosts _get_pseudocosts
#define get_ptr_constraints _get_ptr_constraints
#define get_ptr_dual_solution _get_ptr_dual_solution
#define get_ptr_lambda _get_ptr_lambda
#define get_ptr_primal_solution _get_ptr_primal_solution
#define get_ptr_sensitivity_obj _get_ptr_sensitivity_obj
#define get_ptr_sensitivity_objex _get_ptr_sensitivity_objex
#define get_ptr_sensitivity_rhs _get_ptr_sensitivity_rhs
#define get_ptr_variables _get_ptr_variables
#define get_rh _get_rh
#define get_rh_range _get_rh_range
#define get_row _get_row
#define get_row_name _get_row_name
#define get_scalelimit _get_scalelimit
#define get_scaling _get_scaling
#define get_sensitivity_obj _get_sensitivity_obj
#define get_sensitivity_objex _get_sensitivity_objex
#define get_sensitivity_rhs _get_sensitivity_rhs
#define get_simplextype _get_simplextype
#define get_solutioncount _get_solutioncount
#define get_solutionlimit _get_solutionlimit
#define get_status _get_status
#define get_statustext _get_statustext
#define get_timeout _get_timeout
#define get_total_iter _get_total_iter
#define get_total_nodes _get_total_nodes
#define get_upbo _get_upbo
#define get_var_branch _get_var_branch
#define get_var_dualresult _get_var_dualresult
#define get_var_primalresult _get_var_primalresult
#define get_var_priority _get_var_priority
#define get_variables _get_variables
#define get_verbose _get_verbose
#define get_working_objective _get_working_objective
#define has_BFP _has_BFP
#define has_XLI _has_XLI
#define is_add_rowmode _is_add_rowmode
#define is_anti_degen _is_anti_degen
#define is_binary _is_binary
#define is_break_at_first _is_break_at_first
#define is_constr_type _is_constr_type
#define is_debug _is_debug
#define is_feasible _is_feasible
#define is_unbounded _is_unbounded
#define is_infinite _is_infinite
#define is_int _is_int
#define is_integerscaling _is_integerscaling
#define is_lag_trace _is_lag_trace
#define is_maxim _is_maxim
#define is_nativeBFP _is_nativeBFP
#define is_nativeXLI _is_nativeXLI
#define is_negative _is_negative
#define is_piv_mode _is_piv_mode
#define is_piv_rule _is_piv_rule
#define is_presolve _is_presolve
#define is_scalemode _is_scalemode
#define is_scaletype _is_scaletype
#define is_semicont _is_semicont
#define is_SOS_var _is_SOS_var
#define is_trace _is_trace
#define lp_solve_version _lp_solve_version
#define make_lp _make_lp
#define print_constraints _print_constraints
#define print_debugdump _print_debugdump
#define print_duals _print_duals
#define print_lp _print_lp
#define print_objective _print_objective
#define print_scales _print_scales
#define print_solution _print_solution
#define print_str _print_str
#define print_tableau _print_tableau
#define put_abortfunc _put_abortfunc
#define put_bb_nodefunc _put_bb_nodefunc
#define put_bb_branchfunc _put_bb_branchfunc
#define put_logfunc _put_logfunc
#define put_msgfunc _put_msgfunc
#define read_LPhandle _read_LPhandle
#define read_MPShandle _read_MPShandle
#define read_XLI _read_XLI
#define read_params _read_params
#define read_basis _read_basis
#define reset_basis _reset_basis
#define reset_params _reset_params
#define resize_lp _resize_lp
#define set_add_rowmode _set_add_rowmode
#define set_anti_degen _set_anti_degen
#define set_basisvar _set_basisvar
#define set_basis _set_basis
#define set_basiscrash _set_basiscrash
#define set_bb_depthlimit _set_bb_depthlimit
#define set_bb_floorfirst _set_bb_floorfirst
#define set_bb_rule _set_bb_rule
#define set_BFP _set_BFP
#define set_binary _set_binary
#define set_bounds _set_bounds
#define set_bounds_tighter _set_bounds_tighter
#define set_break_at_first _set_break_at_first
#define set_break_at_value _set_break_at_value
#define set_column _set_column
#define set_columnex _set_columnex
#define set_col_name _set_col_name
#define set_constr_type _set_constr_type
#define set_debug _set_debug
#define set_epsb _set_epsb
#define set_epsd _set_epsd
#define set_epsel _set_epsel
#define set_epsint _set_epsint
#define set_epslevel _set_epslevel
#define set_epsperturb _set_epsperturb
#define set_epspivot _set_epspivot
#define set_unbounded _set_unbounded
#define set_improve _set_improve
#define set_infinite _set_infinite
#define set_int _set_int
#define set_lag_trace _set_lag_trace
#define set_lowbo _set_lowbo
#define set_lp_name _set_lp_name
#define set_mat _set_mat
#define set_maxim _set_maxim
#define set_maxpivot _set_maxpivot
#define set_minim _set_minim
#define set_mip_gap _set_mip_gap
#define set_multiprice _set_multiprice
#define set_negrange _set_negrange
#define set_obj_bound _set_obj_bound
#define set_obj_fn _set_obj_fn
#define set_obj_fnex _set_obj_fnex
#define set_obj _set_obj
#define set_outputfile _set_outputfile
#define set_outputstream _set_outputstream
#define set_partialprice _set_partialprice
#define set_pivoting _set_pivoting
#define set_preferdual _set_preferdual
#define set_presolve _set_presolve
#define set_print_sol _set_print_sol
#define set_pseudocosts _set_pseudocosts
#define set_rh _set_rh
#define set_rh_range _set_rh_range
#define set_rh_vec _set_rh_vec
#define set_row _set_row
#define set_rowex _set_rowex
#define set_row_name _set_row_name
#define set_scalelimit _set_scalelimit
#define set_scaling _set_scaling
#define set_semicont _set_semicont
#define set_sense _set_sense
#define set_simplextype _set_simplextype
#define set_solutionlimit _set_solutionlimit
#define set_timeout _set_timeout
#define set_trace _set_trace
#define set_upbo _set_upbo
#define set_var_branch _set_var_branch
#define set_var_weights _set_var_weights
#define set_verbose _set_verbose
#define set_XLI _set_XLI
#define solve _solve
#define str_add_column _str_add_column
#define str_add_constraint _str_add_constraint
#define str_add_lag_con _str_add_lag_con
#define str_set_obj_fn _str_set_obj_fn
#define str_set_rh_vec _str_set_rh_vec
#define time_elapsed _time_elapsed
#define unscale _unscale
#define write_lp _write_lp
#define write_LP _write_LP
#define write_mps _write_mps
#define write_MPS _write_MPS
#define write_freemps _write_freemps
#define write_freeMPS _write_freeMPS
#define write_XLI _write_XLI
#define write_basis _write_basis
#define write_params _write_params
