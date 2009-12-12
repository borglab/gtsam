load beijing_graph.mat;

ordering = bottom_up_ordering(pred);
ordering.print('ordering')
