% test Enum
params = gtsam.GncLMParams();

gtsam.EXPECT('Get lossType',params.lossType==gtsam.GncLossType.TLS);

params.lossType = gtsam.GncLossType.GM;
gtsam.EXPECT('Set lossType',params.lossType==gtsam.GncLossType.GM);

params.setLossType(gtsam.GncLossType.TLS);
gtsam.EXPECT('setLossType',params.lossType==gtsam.GncLossType.TLS);
