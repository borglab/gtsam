% Regression coverage for shared-pointer declarations written with the
% wrapper DSL's T* spelling.

noise = gtsam.noiseModel.Isotropic.Sigma(2, 1.0);

triangulation = gtsam.TriangulationParameters();
triangulation.noiseModel = noise;
clear noise;
recoveredNoise = triangulation.noiseModel;
gtsam.EXPECT('triangulation noise type', ...
    isa(recoveredNoise, 'gtsam.noiseModel.Isotropic'));
gtsam.EXPECT('triangulation noise value', recoveredNoise.dim() == 2);
clear triangulation;
gtsam.EXPECT('triangulation noise lifetime', recoveredNoise.dim() == 2);

preconditioner = gtsam.DummyPreconditionerParameters();
pcg = gtsam.PCGSolverParameters();
pcg.preconditioner = preconditioner;
clear preconditioner;
recoveredPreconditioner = pcg.preconditioner;
gtsam.EXPECT('preconditioner type', ...
    isa(recoveredPreconditioner, 'gtsam.DummyPreconditionerParameters'));
clear pcg;
gtsam.EXPECT('preconditioner lifetime', ...
    isa(recoveredPreconditioner, 'gtsam.DummyPreconditionerParameters'));

preintegration = gtsam.PreintegrationParams.MakeSharedU(9.81);
legged = gtsam.LeggedEstimatorParams();
legged.preintegrationParams = preintegration;
clear preintegration;
recoveredPreintegration = legged.preintegrationParams;
gtsam.EXPECT('preintegration type', ...
    isa(recoveredPreintegration, 'gtsam.PreintegrationParams'));
clear legged;
gtsam.EXPECT('preintegration lifetime', ...
    isa(recoveredPreintegration, 'gtsam.PreintegrationParams'));

kernel = gtsam.noiseModel.mEstimator.Huber.Create(1.345);
robust = gtsam.noiseModel.Robust.Create(kernel, recoveredNoise);
clear kernel recoveredNoise;
gtsam.EXPECT('robust kernel type', ...
    isa(robust.robust(), 'gtsam.noiseModel.mEstimator.Huber'));
gtsam.EXPECT('robust noise type', ...
    isa(robust.noise(), 'gtsam.noiseModel.Isotropic'));

clear robust recoveredPreconditioner recoveredPreintegration;
