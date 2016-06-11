%% Plot result, including covariance ellipses
figure(1);clf; plot(initial.xs(),initial.ys(),'g-*'); axis equal
hold on; plot(result.xs(),result.ys(),'b-*')
for i=1:result.size()-1
    pose_i = result.pose(i);
    P{i}=marginals.marginalCovariance(i);
    covarianceEllipse([pose_i.x;pose_i.y],P{i},'b')
end
