clear;
clc;
close all;

%% Information for keyframes
% x = [0; 1; 2; 3; 4];
% y = [0; 1; 2; 3; 4];
% z = [0; 0.1; 0.2; 0.3; 0.4];
% yaw =  [0; 0; 0; 0; 0];
% keyframes = [x, y, z, yaw];
% 
% cameraYaw = [0; 0; 0; 0; 0];
% cameraPitch = [0; 0; 0; 0; 0];
% keyorientations = [cameraYaw, cameraPitch];
% 
% keytimes = [1; 2; 3; 4; 5];
% 
% T = 1/10;

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
options.contour_weight = 1000;
options.lag_weight = 1000;
options.orientation_weight = 1;
options.progress_weight = 1;
options.jerk_weight = 1;
options.relative_timing_weight = 10000;
options.absolute_timing_weight = 10000;
% options.raw_condition = 10;
% options.tuned_condition = 10;

osmTrajectoryGenerator = OneShotMPCC(T, keyframes, keyorientations, options);
osmTrajectoryGenerator.setup_system();
[raw_output, tuned_output] = osmTrajectoryGenerator.generate_trajectory();

%% Debug
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

s = raw_output.trajectory(17,:);
for i = 1:length(s)
    if s(i) > max(osmTrajectoryGenerator.theta)
        break;
    end
end
s(i:end) = [];
ref_state = ppval(osmTrajectoryGenerator.pos_spline,s);
ref_vel = ppval(osmTrajectoryGenerator.vel_spline,s); 

figure;
subplot(3,1,1);
plot(s,ref_state(1,:),'--r',s,ref_state(2,:),'--g',s,ref_state(3,:),'--b');
hold on;
plot(s,raw_output.trajectory(1,1:length(s)),'r',s,raw_output.trajectory(2,1:length(s)),'g', ... 
    s,raw_output.trajectory(3,1:length(s)),'b');
hold off;
title('Position Plot');
legend('xref','yref','zref','x','y','z');

subplot(3,1,2);
plot(s,ref_vel(1,:),'--r',s,ref_vel(2,:),'--g',s,ref_vel(3,:),'--b');
hold on;
plot(s,raw_output.trajectory(5,1:length(s)),'r',s,raw_output.trajectory(6,1:length(s)),'g', ... 
    s,raw_output.trajectory(7,1:length(s)),'b');
hold off;
title('Velocity Plot');
legend('vxref','vyref','vzref','vx','vy','vz');

subplot(3,1,3);
haha = s(2:end);
plot(haha,raw_output.input(1,1:1:length(s) - 1),'r', ... 
    haha,raw_output.input(2,1:length(s) - 1),'g', ... 
    haha,raw_output.input(3,1:length(s) - 1),'b');
title('Input Plot');
legend('Fx','Fy','Fz');

figure;
subplot(2,1,1);
plot(s,raw_output.trajectory(9,1:length(s)),'r',s,raw_output.trajectory(10,1:length(s)),'g', ... 
    s,raw_output.trajectory(11,1:length(s)),'b');
title('Acceleration Plot');
legend('ax','ay','az');

subplot(2,1,2);
plot(s,raw_output.trajectory(13,1:length(s)),'r',s,raw_output.trajectory(14,1:length(s)),'g', ... 
    s,raw_output.trajectory(15,1:length(s)),'b');
title('Jerk Plot');
legend('jx','jy','jz');

ref_camyaw = ppval(osmTrajectoryGenerator.cy_spline,s);
ref_campitch = ppval(osmTrajectoryGenerator.cp_spline,s);

figure;
plot(s,ref_camyaw,'--r',s,ref_campitch,'--b');
hold on;
plot(s,raw_output.trajectory(21,1:length(s)),'r',s,raw_output.trajectory(4,1:length(s)),'m', ... 
    s,raw_output.trajectory(22,1:length(s)),'b');
hold off;
title('Camera Plot');
legend('yawref','pitchref','camyaw','quadyaw','campitch');

time = 0:T:T*(length(s) - 1);

figure;
subplot(2,1,1);
plot(raw_output.trajectory(17,1:length(s)),time);
title('Time vs Theta');
subplot(2,1,2);
plot(raw_output.trajectory(17,1:length(s)),raw_output.trajectory(18,1:length(s)));
title('Theta Velocity vs Theta');

%% Plots for tuned output
if isfield(tuned_output, 'trajectory')
    figure;
    plot3(keyframes(:,1),keyframes(:,2),keyframes(:,3),'o');
    % axis([-1.5 1.5 -1.5 1.5 -0.1 1.1]);
    hold on;
    plot3(tuned_output.trajectory(1,:),tuned_output.trajectory(2,:),tuned_output.trajectory(3,:));
    hold off;
    
    s = tuned_output.trajectory(17,:);
    for i = 1:length(s)
        if s(i) > max(osmTrajectoryGenerator.theta)
            break;
        end
    end
    s(i:end) = [];
    ref_state = ppval(osmTrajectoryGenerator.pos_spline,s);
    ref_vel = ppval(osmTrajectoryGenerator.vel_spline,s); 
    
    figure;
    subplot(3,1,1);
    plot(s,ref_state(1,:),'--r',s,ref_state(2,:),'--g',s,ref_state(3,:),'--b');
    hold on;
    plot(s,tuned_output.trajectory(1,1:length(s)),'r',s,tuned_output.trajectory(2,1:length(s)),'g', ... 
        s,tuned_output.trajectory(3,1:length(s)),'b');
    hold off;
    title('Position Plot');
    legend('xref','yref','zref','x','y','z');
    
    subplot(3,1,2);
    plot(s,ref_vel(1,:),'--r',s,ref_vel(2,:),'--g',s,ref_vel(3,:),'--b');
    hold on;
    plot(s,tuned_output.trajectory(5,1:length(s)),'r',s,tuned_output.trajectory(6,1:length(s)),'g', ... 
        s,tuned_output.trajectory(7,1:length(s)),'b');
    hold off;
    title('Velocity Plot');
    legend('vxref','vyref','vzref','vx','vy','vz');
    
    subplot(3,1,3);
    haha = s(2:end);
    plot(haha,tuned_output.input(1,1:length(s) - 1),'r', ...
        haha,tuned_output.input(2,1:length(s) - 1),'g', ...
        haha,tuned_output.input(3,1:length(s) - 1),'b');
    title('Input Plot');
    legend('Fx','Fy','Fz');
    
    figure;
    subplot(2,1,1);
    plot(s,tuned_output.trajectory(9,1:length(s)),'r',s,tuned_output.trajectory(10,1:length(s)),'g', ...
        s,tuned_output.trajectory(11,1:length(s)),'b');
    title('Acceleration Plot');
    legend('ax','ay','az');
    
    subplot(2,1,2);
    plot(s,tuned_output.trajectory(13,1:length(s)),'r',s,tuned_output.trajectory(14,1:length(s)),'g', ... 
        s,tuned_output.trajectory(15,1:length(s)),'b');
    title('Jerk Plot');
    legend('jx','jy','jz');
    
    ref_camyaw = ppval(osmTrajectoryGenerator.cy_spline,s);
    ref_campitch = ppval(osmTrajectoryGenerator.cp_spline,s);
    
    figure;
    plot(s,ref_camyaw,'--r',s,ref_campitch,'--b');
    hold on;
    plot(s,raw_output.trajectory(21,1:length(s)),'r',s,raw_output.trajectory(4,1:length(s)),'m', ... 
        s,raw_output.trajectory(22,1:length(s)),'b');
    hold off;
    title('Camera Plot');
    legend('yawref','pitchref','camyaw','quadyaw','campitch');
    
    time = 0:T:T*(length(s) - 1);
    
    figure;
    subplot(2,1,1);
    plot(tuned_output.trajectory(17,1:length(s)),time);
    title('Time vs Theta');
    subplot(2,1,2);
    plot(tuned_output.trajectory(17,1:length(s)),tuned_output.trajectory(18,1:length(s)));
    title('Theta Velocity vs Theta');
end

%% Convert mpcc coordinates to gps
% [timed_longitude, timed_latitude, timed_altitude, timed_heading, timed_tilt] = ...
%     coordinate_to_gps(trajectory(8,1), trajectory(7,1), trajectory(9,1), X_timed(7,:), X_timed(8,:), ...
%     X_timed(9,:), X_timed(10,:), X_timed(27,:), X_timed(28,:));

old_interval = 0:1/10:((size(X_timed,2)-1)*T);
new_interval = 0:1/100:((size(X_timed,2)-1)*T);
timed_x = interp1(old_interval, X_timed(7,:), new_interval);
timed_y = interp1(old_interval, X_timed(8,:), new_interval);
timed_z = interp1(old_interval, X_timed(9,:), new_interval);
timed_yaw = interp1(old_interval, X_timed(10,:), new_interval);
timed_camyaw = interp1(old_interval, X_timed(27,:), new_interval);
timed_campitch = interp1(old_interval, X_timed(28,:), new_interval);
[timed_longitude, timed_latitude, timed_altitude, timed_heading, timed_tilt] = ...
    coordinate_to_gps(trajectory(8,1), trajectory(7,1), trajectory(9,1), timed_x, timed_y, ...
    timed_z, timed_yaw, timed_camyaw, timed_campitch);

% [mpcc_longitude, mpcc_latitude, mpcc_altitude, mpcc_heading, mpcc_tilt] = ...
%     coordinate_to_gps(trajectory(8,1), trajectory(7,1), trajectory(9,1), raw_output.trajectory(1,:), ...
%     raw_output.trajectory(2,:), raw_output.trajectory(3,:), raw_output.trajectory(4,:), ...
%     raw_output.trajectory(16,:), raw_output.trajectory(17,:));

mpcc_x = interp1(old_interval, raw_output.trajectory(1,:), new_interval);
mpcc_y = interp1(old_interval, raw_output.trajectory(2,:), new_interval);
mpcc_z = interp1(old_interval, raw_output.trajectory(3,:), new_interval);
mpcc_yaw = interp1(old_interval, raw_output.trajectory(4,:), new_interval);
mpcc_camyaw = interp1(old_interval, raw_output.trajectory(16,:), new_interval);
mpcc_campitch = interp1(old_interval, raw_output.trajectory(17,:), new_interval);
[mpcc_longitude, mpcc_latitude, mpcc_altitude, mpcc_heading, mpcc_tilt] = ...
    coordinate_to_gps(trajectory(8,1), trajectory(7,1), trajectory(9,1), mpcc_x, ...
    mpcc_y, mpcc_z, mpcc_yaw, mpcc_camyaw, mpcc_campitch);

%% Generate kml file
generate_kml_file('timed_trajectory.kml', timed_longitude, timed_latitude, timed_altitude, timed_heading, timed_tilt);

generate_kml_file('mpcc_trajectory.kml', mpcc_longitude, mpcc_latitude, mpcc_altitude, mpcc_heading, mpcc_tilt);
