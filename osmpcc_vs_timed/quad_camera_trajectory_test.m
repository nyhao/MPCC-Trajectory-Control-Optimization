function quad_camera_trajectory_test(t, keyframes)
%clear all;
%close all;


%% General options
fly = false;
record_keyframes = false;

%% Discretization time-step
T_step = 0.1;


%% Keyframes
if nargin < 2
    if record_keyframes
        figure();
        h = impoly(gca);
        keyframes = getPosition(h);
        keyframes = [keyframes, zeros(size(keyframes, 1), 2)];
        setColor(h, 'red');
        t = 0:1:(size(keyframes, 1) - 1);
        assignin('base', 't', t);
        assignin('base', 'keyframes', keyframes);
    else
        x = 1;
        y = 1;
        height = 1.5;

%         t = 2 * [0, 2, 4];
        t = 2 * [0, 1, 2, 3, 4];

%         keyframes = [0,0,-2,0;
%                   0,0,height/2,0;
%                   x,y,height,0];
%         keyframes = [
%             -x, y, height, 0;
%              0, 2 * y, height, 0;
%             +x, y, height, 0;
%         ];
        keyframes = [
            -x, y, height, 0;
             -x/2, 1.5*y, height, 0.2;
             0, 2 * y, height, 1.2;
             +x/2, 1.5 * y, height, 0.2;
            +x, y, height, 0;
        ];
    end
end
keyorientations = zeros(size(keyframes, 1), 2);
% keyorientations(:, 1) = [0, 0.2 0.4, 0.6, 0.8];
keyorientations(:, 1) = [0, 0.2 0.4, -0.6, -0.8];
keyorientations(:, 2) = [0, -0.2 -0.4, -0.6, -0.8];

%% Optimization parameters
options = struct();
options.position_weight = 10000000;
options.camera_weight = 1000;
options.derivative_weights = [0, 0, 10, 1, 10, 10];
options.loop_flag = false;
options.velocity_constraints = [];
options.corridor_constraints = [];


%% Generate trajectory
traj = QuadCameraTrajectory(T_step, t, keyframes, keyorientations, options);
[x, fval] = generate(traj);
print_integrals = true;
replace_plots = true;
plot_quad_camera_trajectory(traj, x, T_step, t, keyframes, keyorientations, print_integrals, replace_plots);

if fly
    fly_direct = true;
    if fly_direct

        flight_unity(T_step,x,1, options.loop_flag);
        % mean = 0;
        for i=1:200000000
        %     tic;
            flight_unity(T_step,x,0, options.loop_flag);
        %     test = toc;
        %     mean = mean + test;
        %     disp(mean/i)
        end

    else

        %% Send trajectory to ROS node
        addpath('trajectory_service');
        times = t(1):T_step:t(end);
        % trajectory = prepare_trajectory(times, x);
        % upload_trajectory(trajectory, 'takeoff');
        upload_trajectory([], 'takeoff');
        publish_trajectory(times, x);
        % stream_trajectory('takeoff');
        % stop_trajectory('takeoff');
        % clear_trajectory('takeoff');

    end
end
end
