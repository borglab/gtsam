% test property reading and writing
import gtsam.*;

isamParams = ISAM2Params;

isamParams.relinearizeSkip = 123;
EXPECT('isamParams.relinearizeSkip',isamParams.relinearizeSkip==123);

isamParams.enableRelinearization = false;
EXPECT('isamParams.enableRelinearization_false',isamParams.enableRelinearization==false);
isamParams.enableRelinearization = true;
EXPECT('isamParams.enableRelinearization_true',isamParams.enableRelinearization==true);
