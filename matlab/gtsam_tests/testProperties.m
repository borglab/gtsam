% test property reading and writing
isamParams = gtsam.ISAM2Params;

isamParams.relinearizeSkip = 123;
gtsam.EXPECT('isamParams.relinearizeSkip',isamParams.relinearizeSkip==123);

isamParams.enableRelinearization = false;
gtsam.EXPECT('isamParams.enableRelinearization_false',isamParams.enableRelinearization==false);
isamParams.enableRelinearization = true;
gtsam.EXPECT('isamParams.enableRelinearization_true',isamParams.enableRelinearization==true);
