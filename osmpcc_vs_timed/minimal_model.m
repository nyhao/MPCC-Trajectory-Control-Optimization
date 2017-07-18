clear;
clc;
close all;

load('./trajectories/1_record_1_2016_08_31_17_36_29.mat')
nx = 18;
nu = 7;
m = 0.5;
g = -9.81;

% Continuous state space matrices
% dot.X = Ac*X + Bc*U
% dot.X = [vx; vy; vz; wyaw; ax; ay; az; ayaw]
% X = [x; y; z; yaw; vx; vy; vz; wyaw]
% U = [Fx; Fy; Fz; Tyaw]
Ac = zeros(nx,nx);
Ac(1, 5) = 1;
Ac(2, 6) = 1;
Ac(3, 7) = 1;
Ac(4, 8) = 1;

Ac(5, 9) = 1;
Ac(6, 10) = 1;
Ac(7, 11) = 1;
Ac(8, 12) = 1;

Ac(13, 14) = 1;

Bc = zeros(nx,nu);
Bc(9, 1) = 1; %/m;
Bc(10, 2) = 1; %/m;
Bc(11, 3) = 1; %/m;
Bc(12, 4) = 1; %/m;

Bc(14, 5) = 1;

Ac(15, 17) = 1;
Ac(16, 18) = 1;
Bc(17, 6) = 1;
Bc(18, 7) = 1;

gc = zeros(nx,1);
gc(11,1) = g;

I = eye(nx);

% Discretization
A_tilde = [Ac, Bc, I; zeros(nx + nu, nx + nu + nx)];
A_tilde_d = expm(A_tilde*T);

Ad = A_tilde_d(1:nx,1:nx);
Bd = A_tilde_d(1:nx,nx+1:nx+nu);
gd = A_tilde_d(1:nx,nx+nu+1:2*nx+nu) * gc;

Ad(17, 17) = 0;
Ad(18, 18) = 0;

% Weights
contour_weight = 1;
lag_weight = 1;
orientation_weight = 0.01;
progress_weight = 0.1;
jerk_weight = 0.01;
relative_timing_weight = 0;
absolute_timing_weight = 0;
rest_weight =  100;

poly_order = 2;

% Get theta from keyframes (Calculate chord lengths between keyframes)
keyframes_to_theta = cumsum([0;((diff(keyframes(:,1:3)).^2)*ones(3,1)).^(1/4)]).';
theta = linspace(keyframes_to_theta(1), keyframes_to_theta(end), 160+1);

% Construct spline functions for position and velocity which
% are parameterized by theta
pos_spline = cscvn(keyframes(:,1:3)');
vel_spline = fnder(pos_spline);

% Compute reference positions, orientations and velocities
% with spline functions
positions = ppval(pos_spline,theta);
velocities = ppval(vel_spline,theta);

% Construct spline functions for camera's yaw and pitch which
% are parameterized by theta
cy_spline = spline(keyframes_to_theta,keyorientations(:,1)');
cp_spline = spline(keyframes_to_theta,keyorientations(:,2)');

% Compute reference camera's yaw and pitch with spline
% functions
camera_yaw = ppval(cy_spline,theta);
camera_pitch = ppval(cp_spline,theta);

umax = [+4;+4;+8;+4;+2*m;+2*m;+2*m]*(1/m);
umin = -umax;
umin(3) = 0;
xmax = +100 * ones(nx, 1);
xmin = -xmax;
xmin(14) = 0;  % only allows forward progress

% Dimensions
model.N = 160 + 1;  % plus 1 initial stage
model.nvar = nu+nx;
model.neq = nx;
model.npar = (6*(poly_order+1)+3*(poly_order))+2+8;

% Objectives
model.objective = @(z,par) cost_function(z, par, nu);

% Equality constraints
model.eq = @(z) Ad*z(nu+1:nu+nx) + Bd*z(1:nu) + gd;

% LHS matrix of equality constraints
model.E = [zeros(nx,nu), eye(nx)];

% Initial states
model.xinitidx = nu+1:nu+nx;

% Inequality constraints
model.lb = [ umin',    xmin'  ];
model.ub = [ umax',    xmax'  ];

codeoptions = getOptions('FORCESNLPsolver');
codeoptions.printlevel = 0;
codeoptions.maxit = 5000;
codeoptions.server = 'https://forces-preview.embotech.com';

FORCES_NLP(model, codeoptions);



progress_id = ones(1,160+1);
solution = 5000;
exitflag = 0;

epsilon = 3;

while (exitflag ~= 1 || solution > epsilon)
    % Setup the problem
    old_progress_id = progress_id;
    Xout = zeros(nx,160+1);
    Xout(:,1) = [positions(:,1); 0; zeros(4,1); zeros(4,1); zeros(2,1); ...
        camera_yaw(1); camera_pitch(1); zeros(2,1)];  % initial conditions
    Uout = zeros(nu,nx);
    problem.x0 = zeros((160+1)*(nx+nu),1);  % stack up problems into one N stages array
    k = 1;
    fitlength = 40;
    
    problem.xinit = Xout(:,k);
    
    % Compute polynomial coefficients and parameters for every time-step
    param = zeros(model.npar, 160+1);
    count = 1;
    for k = progress_id
        if (k >= 160-fitlength)
            fitrange = k-fitlength:160+1;
        else
            fitrange = k:k+fitlength;
        end
        px = polyfit(theta(fitrange),positions(1,fitrange),poly_order);
        py = polyfit(theta(fitrange),positions(2,fitrange),poly_order);
        pz = polyfit(theta(fitrange),positions(3,fitrange),poly_order);
        dpx = polyder(px);
        dpy = polyder(py);
        dpz = polyder(pz);
        if (length(dpx) ~= poly_order)
            if (find(px == 0) == 1)
                dpx(2) = dpx(1);
                dpx(1) = 0;
            else
                dpx(2) = 0;
            end
        end
        if (length(dpy) ~= poly_order)
            if (find(py == 0) == 1)
                dpy(2) = dpy(1);
                dpy(1) = 0;
            else
                dpy(2) = 0;
            end
        end
        if (length(dpz) ~= poly_order)
            if (find(pz == 0) == 1)
                dpz(2) = dpz(1);
                dpz(1) = 0;
            else
                dpz(2) = 0;
            end
        end
        pcy = polyfit(theta(fitrange),camera_yaw(fitrange),poly_order);
        pcp = polyfit(theta(fitrange),camera_pitch(fitrange),poly_order);
        ptv = polyfit(theta(fitrange),zeros(1,length(fitrange)),poly_order);
        timing_theta = 0;
        param(:,count) = [px'; py'; pz'; dpx'; dpy'; dpz'; pcy'; pcp'; ptv'; ...
            timing_theta; max(theta); contour_weight; lag_weight; orientation_weight; ...
            progress_weight; jerk_weight; relative_timing_weight; absolute_timing_weight; rest_weight];
        count = count + 1;
    end
    
    problem.all_parameters = reshape(param, [(160+1)*(model.npar), 1]);  % stack up parameters
    
    % Solve the problem
    [solverout,exitflag,info] = FORCESNLPsolver(problem);
    disp(['exitflag = ', num2str(exitflag)]);
    
    % One-shot MPCC output
    names = fieldnames(solverout);
    for j = 1:160+1
        tempout = getfield(solverout, names{j});
        Xout(:,j) = tempout(nu+1:nu+nx);
        if (j <= 160)
            Uout(:,j) = tempout(1:nu);
        end
    end
    
    % Determine progress index with respect to reference theta
    % for next iteration
    for pind = 1:160+1
        [~,progress_id(pind)] = min(abs(repmat(Xout(13,pind),1,160+1) - theta));
    end
    
    % Compute solution to decide when to stop iterating
    progress_diff = abs(progress_id - old_progress_id);
    solution = mean(progress_diff);
    % coor_diff = abs(obj.positions - Xout(1:3,:));
    % solution = mean(mean(coor_diff));
    disp(['solution = ', num2str(solution)]);
end

s = Xout(13,:);
for i = 1:length(s)
    if s(i) > max(theta)
        break;
    end
end
s(i:end) = [];
ref_state = ppval(pos_spline,s);
ref_vel = ppval(vel_spline,s); 

figure;
plot3(keyframes(:,1),keyframes(:,2),keyframes(:,3),'o');
% axis([-1.5 1.5 -1.5 1.5 -0.1 1.1]);
hold on;
plot3(Xout(1,1:length(s)),Xout(2,1:length(s)),Xout(3,1:length(s)));
hold off;

figure;
subplot(3,1,1);
plot(s,ref_state(1,:),'--r',s,ref_state(2,:),'--g',s,ref_state(3,:),'--b');
hold on;
plot(s,Xout(1,1:length(s)),'r',s,Xout(2,1:length(s)),'g',s,Xout(3,1:length(s)),'b');
hold off;
ylim([-30 20]);
title('Position Plot');
% legend('xref','yref','zref','x','y','z');

subplot(3,1,2);
plot(s,ref_vel(1,:),'--r',s,ref_vel(2,:),'--g',s,ref_vel(3,:),'--b');
hold on;
plot(s,Xout(5,1:length(s)),'r',s,Xout(6,1:length(s)),'g',s,Xout(7,1:length(s)),'b');
hold off;
ylim([-6 6]);
title('Velocity Plot');
% legend('vxref','vyref','vzref','vx','vy','vz');

subplot(3,1,3);
haha = s(2:end);
plot(haha,Uout(1,1:length(s)-1),'r', ... 
    haha,Uout(2,1:length(s)-1),'g', ... 
    haha,Uout(3,1:length(s)-1),'b');
ylim([-10 15]);
title('Input Plot');
% legend('Fx','Fy','Fz');

ref_camyaw = ppval(cy_spline,s);
ref_campitch = ppval(cp_spline,s);

figure;
plot(s,ref_camyaw,'--r',s,ref_campitch,'--b');
hold on;
plot(s,Xout(15,1:length(s))+Xout(4,1:length(s)),'r',s,Xout(16,1:length(s)),'b');
hold off;
title('Camera Plot');
% legend('yawref','pitchref','camyaw','campitch');

time = 0:T:T*(length(s)-1);

figure;
subplot(2,1,1);
plot(Xout(13,1:length(s)),time);
title('Time vs Theta');
subplot(2,1,2);
plot(Xout(13,1:length(s)),Xout(14,1:length(s)));
title('Theta Velocity vs Theta');

function objective = cost_function(z, par, nu)
    % Short name for indices in z
    nx = nu + 1;
    ny = nu + 2;
    nz = nu + 3;
    nyaw = nu + 4;
    nvx = nu + 5;
    nvy = nu + 6;
    nvz = nu + 7;
    nvyaw = nu + 8;
    njerk = 1:3;
    ntheta = nu + 13;
    ntv = nu + 14;
    ncamyaw = nu + 15;
    ncampitch = nu + 16;
    nvcamyaw = nu + 17;
    nvcampitch = nu + 18;

    % Short name for indices in par
    npx = 1:3;
    npy = 4:6;
    npz = 7:9;
    npdx = 10:11;
    npdy = 12:13;
    npdz = 14:15;
    npcy = 16:18;
    npcp = 19:21;
    nptv = 22:24;
    npabs = 25;
    npend = 26;
    npcontour = 27;
    nplag = 28;
    nporientation = 29;
    npprogress = 30;
    npjerk = 31;
    nprelative = 32;
    npabsolute = 33;
    nprest = 34;

    % Point theta that exceeds theta of end keyframe back to theta of end keyframe
    z_theta = (z(ntheta) >= par(npend))*par(npend) + (z(ntheta) < par(npend))*z(ntheta);

    % Vectors needed for lag and contour errors calculation
    pos_ref = [polyval(par(npx),z_theta); polyval(par(npy),z_theta); ...
        polyval(par(npz),z_theta)];
    r = [z(nx); z(ny); z(nz)] - pos_ref;
    tangent = [polyval(par(npdx),z_theta); polyval(par(npdy),z_theta); ...
        polyval(par(npdz),z_theta)];
    unit_tangent = tangent / sqrt(tangent'*tangent);
    contour_vector = r - (r'*unit_tangent)*unit_tangent;
    vq = [z(nvx); z(nvy); z(nvz); z(nvyaw); z(nvcamyaw); z(nvcampitch)];

    % Errors calculation
    lag_error = r'*unit_tangent;
    contour_error = contour_vector'*contour_vector;
    yaw_error = z(nyaw) + z(ncamyaw) - polyval(par(npcy),z_theta);
    pitch_error = z(ncampitch) - polyval(par(npcp),z_theta);
    progress_control = z(ntv) - polyval(par(nptv),z_theta);
%     jerk_minimization = [z(njerk); z(ntj)];
    jerk_minimization = z(njerk);
    absolute_timing_control = z(ntheta) - par(npabs);
    rest_error = vq'*vq;

    % Weight functions
    contour_weight = par(npcontour);
    lag_weight = par(nplag);
    orientation_weight = par(nporientation);
%     jerk_weight_matrix = [par(npjerk), 0, 0, 0; ...
%                             0, par(npjerk), 0, 0; ...
%                             0, 0, par(npjerk), 0; ...
%                             0, 0, 0, par(npjerk)];
    jerk_weight_matrix = par(npjerk)*eye(3);
    progress_weight_function = @(rtv) (rtv > 0)*(par(nprelative) + par(npprogress)) - ...
        par(npprogress);
    absolute_timing_weight_function = @(abst) (abst > 0)*(par(npabsolute));
    rest_weight = (z(ntheta) >= par(npend))*par(nprest);

    % Combine all errors into an objective or cost function
    objective = lag_error*lag_weight*lag_error + contour_error*contour_weight + ...
        yaw_error*orientation_weight*yaw_error + pitch_error*orientation_weight*pitch_error + ...
        progress_control*progress_weight_function(polyval(par(nptv),z_theta))*progress_control + ...
        jerk_minimization'*jerk_weight_matrix*jerk_minimization + ...
        absolute_timing_control*absolute_timing_weight_function(par(npabs))*absolute_timing_control + ...
        rest_error*rest_weight*rest_error;
end