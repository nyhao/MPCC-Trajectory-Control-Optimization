clear;
clc;
close all;

%% Information for keyframes
x = [0; 1; 2; 3; 4];
y = [0; 1; 2; 3; 4];
z = [0; 0.1; 0.2; 0.3; 0.4];
yaw =  [0; 0; 0; 0; 0];
keyframes = [x, y, z, yaw];

cameraYaw = [0; 0; 0; 0; 0];
cameraPitch = [0; 0; 0; 0; 0];
keyorientations = [cameraYaw, cameraPitch];

keytimes = [1; 2; 3; 4; 5];

T = 1/10;

%% Load recorded user's keyframes
load('./trajectories/1_record_1_2016_08_31_17_36_29.mat')
% load('./trajectories/1_record_2_2016_08_31_18_00_02.mat')
% load('./trajectories/1_snap_1_2016_08_31_17_42_00.mat')
% load('./trajectories/2_record_1_2016_09_01_15_16_01.mat')

%% Timed trajectory generation
options = struct();
options.position_weight = 10000000;
options.camera_weight = 1000;
options.derivative_weights = [0, 0, 10, 1, 10, 10];
options.loop_flag = false;
options.velocity_constraints = [];
options.corridor_constraints = [];

timedTrajectoryGenerator = QuadCameraTrajectory(T, keytimes, keyframes, keyorientations, options);
[X_timed, fval_timed] = timedTrajectoryGenerator.generate();

%% One-shot MPCC trajectory generation
options = struct();
options.contour_weight = 5000;
options.lag_weight = 10000;
options.orientation_weight = 0.01;
options.progress_weight = 0.01;
options.jerk_weight = 0.1;
options.absolute_timing_weight = 100000;
% options.raw_condition = 12;
% options.tuned_condition = 12;

osmTrajectoryGenerator = OneShotMPCC(T, keyframes, keyorientations, options);
osmTrajectoryGenerator.setup_system();
[raw_output, tuned_output] = osmTrajectoryGenerator.generate_trajectory();

% Debug
[Ad, Bd, gd] = osmTrajectoryGenerator.build_quad_model(T, osmTrajectoryGenerator.nStates, ...
osmTrajectoryGenerator.nInputs);

%% Plots for timed output
figure;
plot3(keyframes(:,1),keyframes(:,2),keyframes(:,3),'o');
% axis([-1.5 1.5 -1.5 1.5 -0.1 1.1]);
hold on;
plot3(X_timed(7,:),X_timed(8,:),X_timed(9,:));
hold off;

figure;
subplot(4,1,1);
plot(timedTrajectoryGenerator.times,X_timed(7,:),'r', ... 
    timedTrajectoryGenerator.times,X_timed(8,:),'g',timedTrajectoryGenerator.times,X_timed(9,:),'b');
title('Position Plot');
legend('x','y','z');

subplot(4,1,2);
plot(timedTrajectoryGenerator.times,X_timed(11,:),'r', ... 
    timedTrajectoryGenerator.times,X_timed(12,:),'g',timedTrajectoryGenerator.times,X_timed(13,:),'b');
title('Velocity Plot');
legend('vx','vy','vz');

subplot(4,1,3);
plot(timedTrajectoryGenerator.times,X_timed(1,:),'r', ... 
    timedTrajectoryGenerator.times,X_timed(2,:),'g',timedTrajectoryGenerator.times,X_timed(3,:),'b');
title('Input Plot');
legend('Fx','Fy','Fz');

subplot(4,1,4);
plot(timedTrajectoryGenerator.times,X_timed(19,:),'r', ... 
    timedTrajectoryGenerator.times,X_timed(20,:),'g',timedTrajectoryGenerator.times,X_timed(21,:),'b');
title('Jerk Plot');
legend('jx','jy','jz');

figure;
plot(timedTrajectoryGenerator.times,X_timed(27,:),'r',timedTrajectoryGenerator.times,X_timed(28,:),'b');

%% Plots for raw ouput
figure;
plot3(keyframes(:,1),keyframes(:,2),keyframes(:,3),'o');
% axis([-1.5 1.5 -1.5 1.5 -0.1 1.1]);
hold on;
plot3(raw_output.trajectory(1,:),raw_output.trajectory(2,:),raw_output.trajectory(3,:));
hold off;

s = raw_output.trajectory(13,:);
ref_state = ppval(osmTrajectoryGenerator.pos_spline,s);
ref_vel = ppval(osmTrajectoryGenerator.vel_spline,s); 

figure;
subplot(4,1,1);
plot(s,ref_state(1,:),'--r',s,ref_state(2,:),'--g',s,ref_state(3,:),'--b');
hold on;
plot(s,raw_output.trajectory(1,:),'r',s,raw_output.trajectory(2,:),'g',s,raw_output.trajectory(3,:),'b');
hold off;
title('Position Plot');
legend('xref','yref','zref','x','y','z');

subplot(4,1,2);
plot(s,ref_vel(1,:),'--r',s,ref_vel(2,:),'--g',s,ref_vel(3,:),'--b');
hold on;
plot(s,raw_output.trajectory(5,:),'r',s,raw_output.trajectory(6,:),'g',s,raw_output.trajectory(7,:),'b');
hold off;
title('Velocity Plot');
legend('vxref','vyref','vzref','vx','vy','vz');

subplot(4,1,3);
haha = s(2:end);
plot(haha,raw_output.input(1,1:osmTrajectoryGenerator.nStages),'r', ... 
    haha,raw_output.input(2,1:osmTrajectoryGenerator.nStages),'g', ... 
    haha,raw_output.input(3,1:osmTrajectoryGenerator.nStages),'b');
title('Input Plot');
legend('Fx','Fy','Fz');

subplot(4,1,4);
plot(s,raw_output.trajectory(9,:),'r',s,raw_output.trajectory(10,:),'g',s,raw_output.trajectory(11,:),'b');
title('Jerk Plot');
legend('jx','jy','jz');

ref_camyaw = ppval(osmTrajectoryGenerator.cy_spline,s);
ref_campitch = ppval(osmTrajectoryGenerator.cp_spline,s);

figure;
plot(s,ref_camyaw,'--r',s,ref_campitch,'--b');
hold on;
plot(s,raw_output.trajectory(16,:),'r',s,raw_output.trajectory(4,:),'m',s,raw_output.trajectory(17,:),'b');
hold off;
title('Camera Plot');
legend('yawref','pitchref','camyaw','quadyaw','campitch');

time = 0:T:T*(osmTrajectoryGenerator.nStages);

figure;
subplot(2,1,1);
plot(raw_output.trajectory(13,:),time);
title('Time vs Theta');
subplot(2,1,2);
plot(raw_output.trajectory(13,:),raw_output.trajectory(14,:));
title('Theta Velocity vs Theta');

%% Plots for tuned output
if isfield(tuned_output, 'trajectory')
    figure;
    plot3(keyframes(:,1),keyframes(:,2),keyframes(:,3),'o');
    % axis([-1.5 1.5 -1.5 1.5 -0.1 1.1]);
    hold on;
    plot3(tuned_output.trajectory(1,:),tuned_output.trajectory(2,:),tuned_output.trajectory(3,:));
    hold off;
    
    s = tuned_output.trajectory(13,:);
    ref_state = ppval(osmTrajectoryGenerator.pos_spline,s);
    ref_vel = ppval(osmTrajectoryGenerator.vel_spline,s); 
    
    figure;
    subplot(4,1,1);
    plot(s,ref_state(1,:),'--r',s,ref_state(2,:),'--g',s,ref_state(3,:),'--b');
    hold on;
    plot(s,tuned_output.trajectory(1,:),'r',s,tuned_output.trajectory(2,:),'g',s,tuned_output.trajectory(3,:),'b');
    hold off;
    title('Position Plot');
    legend('xref','yref','zref','x','y','z');
    
    subplot(4,1,2);
    plot(s,ref_vel(1,:),'--r',s,ref_vel(2,:),'--g',s,ref_vel(3,:),'--b');
    hold on;
    plot(s,tuned_output.trajectory(5,:),'r',s,tuned_output.trajectory(6,:),'g',s,tuned_output.trajectory(7,:),'b');
    hold off;
    title('Velocity Plot');
    legend('vxref','vyref','vzref','vx','vy','vz');
    
    subplot(4,1,3);
    haha = s(2:end);
    plot(haha,tuned_output.input(1,1:osmTrajectoryGenerator.nStages),'r', ...
        haha,tuned_output.input(2,1:osmTrajectoryGenerator.nStages),'g', ...
        haha,tuned_output.input(3,1:osmTrajectoryGenerator.nStages),'b');
    title('Input Plot');
    legend('Fx','Fy','Fz');
    
    subplot(4,1,4);
    plot(s,raw_output.trajectory(9,:),'r',s,raw_output.trajectory(10,:),'g',s,raw_output.trajectory(11,:),'b');
    title('Jerk Plot');
    legend('jx','jy','jz');
    
    ref_camyaw = ppval(osmTrajectoryGenerator.cy_spline,s);
    ref_campitch = ppval(osmTrajectoryGenerator.cp_spline,s);
    
    figure;
    plot(s,ref_camyaw,'--r',s,ref_campitch,'--b');
    hold on;
    plot(s,raw_output.trajectory(16,:),'r',s,raw_output.trajectory(4,:),'m',s,raw_output.trajectory(17,:),'b');
    hold off;
    title('Camera Plot');
    legend('yawref','pitchref','camyaw','quadyaw','campitch');
    
    time = 0:T:T*(osmTrajectoryGenerator.nStages);
    
    figure;
    subplot(2,1,1);
    plot(tuned_output.trajectory(13,:),time);
    title('Time vs Theta');
    subplot(2,1,2);
    plot(tuned_output.trajectory(13,:),tuned_output.trajectory(14,:));
    title('Theta Velocity vs Theta');
end

%% Convert mpcc coordinates to gps
[timed_longitude, timed_latitude, timed_altitude, timed_heading, timed_tilt] = ...
    coordinate_to_gps(trajectory(8,1), trajectory(7,1), trajectory(9,1), X_timed(7,:), X_timed(8,:), ...
    X_timed(9,:), X_timed(10,:), X_timed(27,:), X_timed(28,:));

[mpcc_longitude, mpcc_latitude, mpcc_altitude, mpcc_heading, mpcc_tilt] = ...
    coordinate_to_gps(trajectory(8,1), trajectory(7,1), trajectory(9,1), raw_output.trajectory(1,:), ...
    raw_output.trajectory(2,:), raw_output.trajectory(3,:), raw_output.trajectory(4,:), ...
    raw_output.trajectory(16,:), raw_output.trajectory(17,:));

%% Generate kml file
generate_kml_file('timed_trajectory.kml', timed_longitude, timed_latitude, timed_altitude, timed_heading, timed_tilt);

generate_kml_file('mpcc_trajectory.kml', mpcc_longitude, mpcc_latitude, mpcc_altitude, mpcc_heading, mpcc_tilt);
