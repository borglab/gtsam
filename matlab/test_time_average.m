% Set up a small SLAM example in MATLAB to test the execution time

clear;
clf;
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
    
    steps
    % simulate the measurements
    measurement_sigma = 1;
    odo_sigma = 0.1;
    [measurements, odometry] = simulate_measurements(map, trajectory, visibility, measurement_sigma, odo_sigma);
    
    
    % create a configuration of all zeroes
     config = create_config(n,m);

    % create the factor graph
    gaussianFactorGraph = create_gaussian_factor_graph(config, measurements, odometry, measurement_sigma, odo_sigma, n);
    % 
    % create an ordering
    ord = create_ordering(n,m);

    ijs = gaussianFactorGraph.sparse(ord);
    A=sparse(ijs(1,:),ijs(2,:),ijs(3,:)); 

    
    runs=50; % for each graph run QR and elimination several times and average the time
    
    ck_qr=cputime;
    for it=1:runs
        R_qr = qr(A);      
    end
    time_qr=[time_qr,(cputime-ck_qr)/runs];
    
    % eliminate with that ordering
    %time gt_sam
    for it=1:runs+1
        if it==2
        ck_gt=cputime;
        end
        BayesNet = gaussianFactorGraph.eliminate_(ord); 
    end
    time_gtsam=[time_gtsam,(cputime-ck_gt)/runs];
    
    clear trajectory visibility gaussianFactorGraph measurements odometry;
    m = m+5;
    velocity=velocity+1;
    steps=steps+1;
end
plot(time_qr,'r');hold on;
plot(time_gtsam,'b');

