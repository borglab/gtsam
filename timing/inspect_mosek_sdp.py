"""Inspect MOSEK exports from the certifiable examples without re-solving.

Requires NumPy and MOSEK Python bindings. Constraint-family labels recognize the
examples' D=1 equality patterns; monolithic comparison requires planar 3D blocks.
"""
import argparse
import json
from pathlib import Path

import mosek
import numpy as np


def unpack(packed, dim):
    matrix = np.zeros((dim, dim))
    index = 0
    for col in range(dim):
        for row in range(col, dim):
            matrix[row, col] = matrix[col, row] = packed[index]
            index += 1
    return matrix


def analyze(prefix, monolithic_prefix=None):
    task = mosek.Task()
    task.readdata(prefix + '.task.gz')
    task.readjsonsol(prefix + '.solution.json')
    solution = mosek.soltype.itr
    count = task.getnumbarvar()
    dimensions = [task.getdimbarvarj(j) for j in range(count)]
    names = [task.getbarvarname(j) for j in range(count)]
    primal = [unpack(task.getbarxj(solution, j), dim) for j, dim in enumerate(dimensions)]
    dual = [unpack(task.getbarsj(solution, j), dim) for j, dim in enumerate(dimensions)]
    costs = [np.zeros_like(x) for x in primal]
    _, js, rows, cols, vals = task.getbarcblocktriplet()
    for j, row, col, val in zip(js, rows, cols, vals):
        costs[j][row, col] += val
        if row != col:
            costs[j][col, row] += val
    objective = sum(float(np.sum(c * x)) for c, x in zip(costs, primal))
    objective_abs = sum(float(np.sum(np.abs(c * x))) for c, x in zip(costs, primal))
    y = np.array(task.gety(solution))
    stationarity = [c - s for c, s in zip(costs, dual)]
    terms = [[] for _ in range(task.getnumcon())]
    _, indices, js, rows, cols, vals = task.getbarablocktriplet()
    for i, j, row, col, val in zip(indices, js, rows, cols, vals):
        terms[i].append((j, row, col, val))
        stationarity[j][row, col] -= y[i] * val
        if row != col:
            stationarity[j][col, row] -= y[i] * val
    xx = np.array(task.getxx(solution))
    groups = {}
    activities = []
    for i, entries in enumerate(terms):
        bound, lower, upper = task.getconbound(i)
        _, scalar_indices, scalar_vals = task.getarow(i)
        activity = sum(val * primal[j][row, col] * (1 if row == col else 2)
                       for j, row, col, val in entries)
        activity += sum(v * xx[k] for k, v in zip(scalar_indices, scalar_vals))
        activities.append(activity)
        if bound != mosek.boundkey.fx or any(scalar_vals):
            raise ValueError("This inspector supports pure SDPs with equalities only.")
        residual = abs(activity - lower)
        if len({j for j, row, col, v in entries}) > 1:
            group = 'overlap'
        elif lower == 1 and len(entries) == 1:
            group = 'h_pair' if entries[0][1] != entries[0][2] else 'h_diagonal'
        elif lower == 1:
            group = 'unit_norm'
        else:
            group = 'gauge'
        groups.setdefault(group, []).append((residual, i))
    residual_max = max(float(np.max(np.abs(r))) for r in stationarity)
    worst_j = max(range(count), key=lambda j: np.max(np.abs(stationarity[j])))
    worst_row, worst_col = np.unravel_index(np.argmax(np.abs(stationarity[worst_j])), stationarity[worst_j].shape)
    # C-A*y is the true dual certificate matrix, not the separately stored S.
    certificate = [r + s for r, s in zip(stationarity, dual)]
    eig_x = [np.linalg.eigvalsh(x) for x in primal]
    eig_s = [np.linalg.eigvalsh(s) for s in dual]
    eig_certificate = [np.linalg.eigvalsh(z) for z in certificate]
    original = json.loads(Path(prefix + '.solution.json').read_text())['Task/solutions']['interior']
    row = dict(prefix=prefix, status=original['solsta'], constraints=task.getnumcon(),
        cliques=count, max_cone_order=max(dimensions), primal_objective=objective,
        dual_objective=task.getdualobj(solution), objective_abs_terms=objective_abs,
        objective_cancellation=objective_abs/max(1,abs(objective)),
        objective_check=objective-task.getprimalobj(solution),
        activity_vs_stored_slack=float(np.max(np.abs(np.array(activities)-task.getxc(solution)))),
        constraint_groups={g: dict(count=len(v), max=max(v)[0], worst_row=max(v)[1]) for g,v in groups.items()},
        dual_stationarity_max=residual_max, worst_dual=dict(clique=names[worst_j], row=int(worst_row), col=int(worst_col)),
        primal_min_eigenvalue=min(float(e[0]) for e in eig_x),
        stored_dual_min_eigenvalue=min(float(e[0]) for e in eig_s),
        certificate_min_eigenvalue=min(float(e[0]) for e in eig_certificate),
        max_primal_entry=max(float(np.max(np.abs(x))) for x in primal),
        max_cost_coefficient=max(float(np.max(np.abs(c))) for c in costs),
        max_dual_multiplier=float(np.max(np.abs(y))))
    if monolithic_prefix:
        # Check the monolithic moment matrix in the exported chordal equations.
        monolithic = mosek.Task()
        mono_prefix = monolithic_prefix
        monolithic.readdata(mono_prefix + '.task.gz')
        monolithic.readjsonsol(mono_prefix + '.solution.json')
        mono_x = unpack(monolithic.getbarxj(solution, 0), monolithic.getdimbarvarj(0))
        clique_keys = [[int(key) for key in name.removesuffix('[]').split('_')[2:]] for name in names]
        keys = sorted({key for clique in clique_keys for key in clique})
        if len(keys)*3 != len(mono_x):
            raise ValueError("Cross-formulation comparison requires 3-coordinate D=1 blocks.")
        offsets = {key: 3*i for i, key in enumerate(keys)}
        clique_indices = [[offsets[key]+r for key in clique for r in range(3)] for clique in clique_keys]
        embedded = [mono_x[np.ix_(indices, indices)] for indices in clique_indices]
        embedded_residual = 0.0
        for i, entries in enumerate(terms):
            _, lower, _ = task.getconbound(i)
            value = sum(v*embedded[j][r,c]*(1 if r == c else 2) for j,r,c,v in entries)
            embedded_residual = max(embedded_residual, abs(value-lower))
        embedded_objective = sum(float(np.sum(c*x)) for c,x in zip(costs, embedded))
        assembled_cost = np.zeros_like(mono_x)
        for indices, c in zip(clique_indices, costs):
            assembled_cost[np.ix_(indices, indices)] += c
        mono_cost = np.zeros_like(mono_x)
        _, js, rows, cols, vals = monolithic.getbarcblocktriplet()
        for j,r,c,v in zip(js,rows,cols,vals):
            mono_cost[r,c] += v
            if r != c:
                mono_cost[c,r] += v
        row['embedded_monolithic'] = dict(max_constraint_residual=embedded_residual,
            objective=embedded_objective, coefficient_difference=float(np.max(np.abs(assembled_cost-mono_cost))))
        row['worst_dual']['key'] = chr(clique_keys[worst_j][worst_row//3] >> 56) + str(clique_keys[worst_j][worst_row//3] & ((1<<56)-1))
        row['worst_dual']['coordinate'] = int(worst_row % 3)
    print(json.dumps(row), flush=True)
    return row

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix', help='Path prefix of .task.gz and .solution.json exports')
    parser.add_argument('--monolithic', help='Optional matching monolithic export prefix; planar D=1 only')
    args = parser.parse_args()
    analyze(args.prefix, args.monolithic)
