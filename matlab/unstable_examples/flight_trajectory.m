function [ values ] = flight_trajectory( input_args )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    import gtsam.*;

    values = Values;
    curvature = 2;
    
    
    forward = Pose3(Rot3(),Point3(10,0,0));
    left = Pose3(Rot3.RzRyRx(0.0,0.0,curvature*pi/180),Point3(10,0,0));
    right = Pose3(Rot3.RzRyRx(0.0,0.0,-curvature*pi/180),Point3(10,0,0));

    pose = Pose3(Rot3.RzRyRx(0,0,0),Point3(0,0,1000));
    
    plan(1).direction = right;
    plan(1).steps = 20;
    
    plan(2).direction = forward;
    plan(2).steps = 5;
    
    plan(3).direction = left;
    plan(3).steps = 100;
    
    plan(4).direction = forward;
    plan(4).steps = 50;
    
    plan(5).direction = left;
    plan(5).steps = 80;
    
    plan(6).direction = forward;
    plan(6).steps = 50;
    
    plan(7).direction = right;
    plan(7).steps = 100;
    
    plan_steps = numel(plan);
    
    values_i = 0;
    
    for i=1:plan_steps
        direction = plan(i).direction;
        segment_steps = plan(i).steps;
        
        for j=1:segment_steps
           pose = pose.compose(direction); 
           values.insert(symbol('x',values_i), pose);
           values_i = values_i + 1;
        end
        
    end


end

