% Set up a small SLAM example in MATLAB to test the execution time

clear;

%Parameters
noRuns=100;
steps=1;
m = 5;
velocity=1;
time_qr=[];
time_gtsam=[];
for steps=1:noRuns
 
    %figure(1);clf;
    % robot moves in the world
    trajectory = walk([0.1,0.1],velocity,m);
    mappingArea=max(trajectory,[],2);
    %plot(trajectory(1,:),trajectory(2,:),'b+'); hold on;

    visibilityTh=sqrt(mappingArea(1)^2+mappingArea(2)^2)/m; %distance between poses
    % Set up the map
    map = create_landmarks(visibilityTh, mappingArea,steps);
    %plot(map(1,:), map(2,:),'g.');
    %axis([0 mappingArea(1) 0 mappingArea(2)]); axis square;
    n=size(map,1)*size(map,2);
    % Check visibility and plot this on the problem figure
    visibilityTh=visibilityTh+steps;
    visibility = create_visibility(map, trajectory,visibilityTh);
    %gplot(visibility,[map trajectory]');
    

    % simulate the measurements
    measurement_sigma = 1;
    odo_sigma = 0.1;
    [measurements, odometry] = simulate_measurements(map, trajectory, visibility, measurement_sigma, odo_sigma);
    
    
%     % create a configuration of all zeroes
     config = create_config(n,m);

    % create the factor graph
    linearFactorGraph = create_linear_factor_graph(config, measurements, odometry, measurement_sigma, odo_sigma, n);
    % 
    % create an ordering
    ord = create_ordering(n,m);

    % show the matrix
   % figure(3); clf;
    [A_dense,b] = linearFactorGraph.matrix(ord);
    A=sparse(A_dense);
    size(A)
    %spy(A);
    %time qr
    ck=cputime;
    R_qr = qr(A);
    time_qr=[time_qr,(cputime-ck)];

    %figure(2)
    %clf
    %spy(R_qr);
    
    % eliminate with that ordering
    %time gt_sam
    ck=cputime;
    BayesNet = linearFactorGraph.eliminate(ord);
    time_gtsam=[time_gtsam,(cputime-ck)];
    
    clear trajectory visibility linearFactorGraph measurements odometry;
    m = m+5;
    velocity=velocity+1;
    steps=steps+1;
end
plot(time_qr,'r');hold on;
plot(time_gtsam,'b');




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % show the eliminated matrix
% figure(4); clf;
% [R,d] = BayesNet.matrix();
% spy(R);
% 
% % optimize in the BayesNet
% optimal = BayesNet.optimize;
% 
% % plot the solution
% figure(5);clf; 
% plot_config(optimal,n,m);hold on
% plot(trajectory(1,:),trajectory(2,:),'b+');
% plot(map(1,:), map(2,:),'g.');
% axis([0 10 0 10]);axis square;
