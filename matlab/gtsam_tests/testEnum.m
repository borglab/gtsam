% test Enum
import gtsam.*;

params = GncLMParams();

EXPECT('Get lossType',params.lossType==GncLossType.TLS);

params.lossType = GncLossType.GM;
EXPECT('Set lossType',params.lossType==GncLossType.GM);

params.setLossType(GncLossType.TLS);
EXPECT('setLossType',params.lossType==GncLossType.TLS);
