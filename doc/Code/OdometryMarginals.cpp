// Query the marginals
cout.precision(2);
Marginals marginals(graph, result);
cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;
