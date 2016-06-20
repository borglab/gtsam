% test Cal3Unified
import gtsam.*;

K = Cal3Unified;
EXPECT('fx',K.fx()==1);
EXPECT('fy',K.fy()==1);

