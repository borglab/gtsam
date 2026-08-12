import gtsam
from gtsam import symbol


class Options:
    """ Options for visual isam example. """

    def __init__(self):
        self.hardConstraint = False
        self.pointPriors = False
        self.batchInitialization = True
        self.reorderInterval = 10
        self.alwaysRelinearize = False


def initialize(data, truth, options):
    # Initialize iSAM
    params = gtsam.ISAM2Params()
    if options.alwaysRelinearize:
        params.relinearizeSkip = 1
    isam = gtsam.ISAM2(params=params)

    # Add constraints/priors
    # TODO: should not be from ground truth!
    newFactors = gtsam.NonlinearFactorGraph()
    initialEstimates = gtsam.Values()
    for i in range(2):
        ii = symbol('x', i)
        if i == 0:
            if options.hardConstraint:  # add hard constraint
                newFactors.add(
                    gtsam.NonlinearEqualityPose3(ii, truth.cameras[0].pose()))
            else:
                newFactors.add(
                    gtsam.PriorFactorPose3(ii, truth.cameras[i].pose(),
                                           data.noiseModels.posePrior))
        initialEstimates.insert(ii, truth.cameras[i].pose())

    nextPoseIndex = 2

    # Add visual measurement factors from two first poses and initialize
    # observed landmarks
    for i in range(2):
        ii = symbol('x', i)
        for k in range(len(data.Z[i])):
            j = data.J[i][k]
            jj = symbol('l', j)
            newFactors.add(
                gtsam.GenericProjectionFactorCal3_S2(data.Z[i][
                    k], data.noiseModels.measurement, ii, jj, data.K))
            # TODO: initial estimates should not be from ground truth!
            if not initialEstimates.exists(jj):
                initialEstimates.insert(jj, truth.points[j])
            if options.pointPriors:  # add point priors
                newFactors.add(
                    gtsam.PriorFactorPoint3(jj, truth.points[j],
                                            data.noiseModels.pointPrior))

    # Add odometry between frames 0 and 1
    newFactors.add(
        gtsam.BetweenFactorPose3(
            symbol('x', 0),
            symbol('x', 1), data.odometry[1], data.noiseModels.odometry))

    # Update ISAM
    if options.batchInitialization:  # Do a full optimize for first two poses
        batchOptimizer = gtsam.LevenbergMarquardtOptimizer(newFactors,
                                                           initialEstimates)
        fullyOptimized = batchOptimizer.optimize()
        isam.update(newFactors, fullyOptimized)
    else:
        isam.update(newFactors, initialEstimates)

    # figure(1)tic
    # t=toc plot(frame_i,t,'r.') tic
    result = isam.calculateEstimate()
    # t=toc plot(frame_i,t,'g.')

    return isam, result, nextPoseIndex


def step(data, isam, result, truth, currPoseIndex, isamArgs=()):
    '''
    Do one step isam update
    @param[in] data: measurement data (odometry and visual measurements and their noiseModels)
    @param[in/out] isam: current isam object, will be updated
    @param[in/out] result: current result object, will be updated
    @param[in] truth: ground truth data, used to initialize new variables
    @param[currPoseIndex]: index of the current pose
    '''
    # iSAM expects us to give it a new set of factors
    # along with initial estimates for any new variables introduced.
    newFactors = gtsam.NonlinearFactorGraph()
    initialEstimates = gtsam.Values()

    # Add odometry
    prevPoseIndex = currPoseIndex - 1
    odometry = data.odometry[prevPoseIndex]
    newFactors.add(
        gtsam.BetweenFactorPose3(
            symbol('x', prevPoseIndex),
            symbol('x', currPoseIndex), odometry,
            data.noiseModels.odometry))

    # Add visual measurement factors and initializations as necessary
    for k in range(len(data.Z[currPoseIndex])):
        zij = data.Z[currPoseIndex][k]
        j = data.J[currPoseIndex][k]
        jj = symbol('l', j)
        newFactors.add(
            gtsam.GenericProjectionFactorCal3_S2(
                zij, data.noiseModels.measurement,
                symbol('x', currPoseIndex), jj, data.K))
        # TODO: initialize with something other than truth
        if not result.exists(jj) and not initialEstimates.exists(jj):
            lmInit = truth.points[j]
            initialEstimates.insert(jj, lmInit)

    # Initial estimates for the new pose.
    prevPose = result.atPose3(symbol('x', prevPoseIndex))
    initialEstimates.insert(
        symbol('x', currPoseIndex), prevPose.compose(odometry))

    # Update ISAM
    # figure(1)tic
    isam.update(newFactors, initialEstimates, *isamArgs)
    # t=toc plot(frame_i,t,'r.') tic
    newResult = isam.calculateEstimate()
    # t=toc plot(frame_i,t,'g.')

    return isam, newResult
