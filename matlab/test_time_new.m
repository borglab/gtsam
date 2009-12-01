% Set up a small SLAM example in MATLAB to test the execution time

clear;

%Parameters

noRuns=5;
steps=50;
m = 5*steps;
velocity=1*steps;
time_qr=[];
time_gtsam=[];
%    for steps=1:noRuns
%  
    figure(1);clf;
    % robot moves in the world
    trajectory = walk([0.1,0.1],velocity,m);
    mappingArea=max(trajectory,[],2);
    plot(trajectory(1,:),trajectory(2,:),'b+'); hold on;

    visibilityTh=sqrt(mappingArea(1)^2+mappingArea(2)^2)/m; %distance between poses
    % Set up the map
    map = create_landmarks(visibilityTh, mappingArea,steps);
    plot(map(1,:), map(2,:),'g.');
    axis([0 mappingArea(1) 0 mappingArea(2)]); axis square;
    n=size(map,2);
    % Check visibility and plot this on the problem figure
    visibilityTh=visibilityTh+steps;
    visibility = create_visibility(map, trajectory,visibilityTh);
    gplot(visibility,[map trajectory]');
    

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
    %ord = create_good_ordering(n,m,measurements);
    ord = create_ordering(n,m);
    % show the matrix
   % figure(3); clf;
    %[A_dense,b] = linearFactorGraph.matrix(ord);
   %A=sparse(A_dense);
    
    %sparse matrix !!!
    ijs = linearFactorGraph.sparse(ord);
    A=sparse(ijs(1,:),ijs(2,:),ijs(3,:)); 
    %spy(A);
    %time qr
    runs=1;
    
    ck_qr=cputime;
    for i=1:runs
        R_qr = qr(A);      
    end
    time_qr=(cputime-ck_qr)/runs
    %time_qr=[time_qr,(cputime-ck)];

    %figure(2)
    %clf
    %spy(R_qr);
    
    % eliminate with that ordering
    %time gt_sam
%     for i=1:runs+10
%         if i==11
%         ck_gt=cputime;
%         end
%         BayesNet = linearFactorGraph.eliminate(ord); 
%     end
ck_gt=cputime;
for i=1:runs+10
    BayesNet = linearFactorGraph.eliminate_(ord);
end
    time_gtsam=(cputime-ck_gt)/runs
    %time_gtsam=[time_gtsam,(cputime-ck)];
    
%     clear trajectory visibility linearFactorGraph measurements odometry;
%     m = m+5;
%     velocity=velocity+1;

% end
% %time_qr=time_qr/noRuns
%  plot(time_qr,'r');hold on;
%  plot(time_gtsam,'b');




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
