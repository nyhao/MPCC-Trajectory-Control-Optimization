clear;
clc;
close all;

%% system
T = 1/30; % sampling time for 30Hz
m = 0.5; % mass
nx = 12; % 4d position (angle around z-axis) and up to 4th derivatives
nalx = 3; % arc length and up to 4th derivatives
nu = 4; % force acting on the acceleration states
nalu = 1; % arc acceleration (virtual input)
g = -9.81; % gravitational acceleration

% continuous state space matrices
% dot.X = Ac*X + Bc*U
% dot.X = [vx; vy; vz; wyaw; ax; ay; az; ayaw]
% X = [x; y; z; yaw; vx; vy; vz; wyaw]
% U = [Fx; Fy; Fz; Tyaw]
Ac = zeros(nx+nalx,nx+nalx);
Ac(1,5) = 1;
Ac(2,6) = 1;
Ac(3,7) = 1;
Ac(4,8) = 1;

Ac(13,14) = 1;
% Ac(21,22) = 1;

Bc = zeros(nx+nalx,nu+nalu);
Bc(5,1) = 1/m;
Bc(6,2) = 1/m;
Bc(7,3) = 1/m;
Bc(8,4) = 1/m;

Bc(14,5) = 1;
% Bc(22,5) = 1;

gc = zeros(nx+nalx,1);
gc(7,1) = g;

I = eye(nx+nalx);

% discretization
A_tilde = [Ac, Bc, I; zeros(nx + nalx + nu + nalu, nx + nalx + nu + nalu + nx + nalx)]; 
A_tilde_d = expm(A_tilde*T);

Ad = A_tilde_d(1:nx+nalx,1:nx+nalx);
Bd = A_tilde_d(1:nx+nalx,nx+nalx+1:nx+nalx+nu+nalu);
gd = A_tilde_d(1:nx+nalx,nx+nalx+nu+nalu+1:2*(nx+nalx)+nu+nalu) * gc;

% % finite differentiation
% % a(i+1) = v(i+1) - v(i)
% Ad(9,9) = 0;
% Ad(10,10) = 0;
% Ad(11,11) = 0;
% Ad(12,12) = 0;
% Ad(9,5) = -1/T;
% Ad(10,6) = -1/T;
% Ad(11,7) = -1/T;
% Ad(12,8) = -1/T;
% 
% % j(i+1) = a(i+1) - a(i)
% Ad(13,13) = 0;
% Ad(14,14) = 0;
% Ad(15,15) = 0;
% Ad(16,16) = 0;
% Ad(13,9) = -1/T;
% Ad(14,10) = -1/T;
% Ad(15,11) = -1/T;
% Ad(16,12) = -1/T;
% 
% % s(i+1) = j(i+1) - j(i)
% Ad(17,17) = 0;
% Ad(18,18) = 0;
% Ad(19,19) = 0;
% Ad(20,20) = 0;
% Ad(17,13) = -1/T;
% Ad(18,14) = -1/T;
% Ad(19,15) = -1/T;
% Ad(20,16) = -1/T;
% 
% % theta''(i+1) = theta'(i+1) - theta'(i)
% Ad(23,23) = 0;
% Ad(23,22) = -1/T;
% 
% % theta'''(i+1) = theta''(i+1) - theta''(i)
% Ad(24,24) = 0;
% Ad(24,23) = -1/T;
% 
% % theta''''(i+1) = theta'''(i+1) - theta'''(i)
% Ad(25,25) = 0;
% Ad(25,24) = -1/T;

% change in acceleration (input)
% j(i+1) = a(i+1) - a(i)
Ad(9,9) = 0;
Ad(10,10) = 0;
Ad(11,11) = 0;
Ad(12,12) = 0;
Bd(9,1) = -1/(T*m);
Bd(10,2) = -1/(T*m);
Bd(11,3) = -1/(T*m);
Bd(12,4) = -1/(T*m);

% % s(i+1) = j(i+1) - j(i)
% Ad(13,13) = 0;
% Ad(14,14) = 0;
% Ad(15,15) = 0;
% Ad(16,16) = 0;
% Ad(13,9) = -1/T;
% Ad(14,10) = -1/T;
% Ad(15,11) = -1/T;
% Ad(16,12) = -1/T;

% theta'''(i+1) = theta''(i+1) - theta''(i)
Ad(15,15) = 0;
Bd(15,5) = -1/T;

% % theta''''(i+1) = theta'''(i+1) - theta'''(i)
% Ad(20,20) = 0;
% Ad(20,19) = -1/T;

%% MPCC setup
% for constant Q and P matrices
contourpenalty = 10000; 
lagpenalty = 10000; 
orientpenalty = 0;
zpenalty = 1000; 
progresspenalty = 1;%1;
fd_factor = 2;
% Qp = [contourpenalty, 0, 0; 0, lagpenalty, 0; 0, 0, zpenalty];
% Qp = [contourpenalty, 0; 0, lagpenalty];
Qlag = lagpenalty;
Qcontour = contourpenalty;
Qorient = orientpenalty;
Qj = fd_factor*eye(4);
Qdtheta = progresspenalty; 
Qjtheta = fd_factor;
% Pp = [contourpenalty, 0, 0; 0, lagpenalty, 0; 0, 0, zpenalty];
% Pp = [contourpenalty, 0; 0, lagpenalty];
Plag = lagpenalty;
Pcontour = contourpenalty;
Porient = orientpenalty;
Pj = fd_factor*eye(4);
Pdtheta = progresspenalty; 
Pjtheta = fd_factor;
% boundaries
umin = [-4, -4, 0, -4, -10];    umax = [4, 4, 10, 4, 10]; % u limits based on drones max force
xmin = -inf*ones(1,nx+nalx);    xmax = inf*ones(1,nx+nalx); % no boundaries in space
xmin(1,14) = 0; % only allows forward progress

%% reference trajectory
kmax = 160;
% theta = 0:T:T*kmax;
dot_theta = zeros(1,kmax+1);
targetpoints = [0, 1, 1, 0, 0; 0, 0, 1, 1, 0; 0, 0, 0, 0, 0];
% targetpoints = [0, 1, 1, 0, 0; 0, 0, 1, 1, 0; 0, 0.25, 0.5, 0.75, 1];
% thetatarget = linspace(theta(1), theta(end), length(targetpoints(1,:)));
% xpp = spline(thetatarget,targetpoints(1,:));
% ypp = spline(thetatarget,targetpoints(2,:));
% zpp = spline(thetatarget,targetpoints(3,:));
% xppdot = fnder(xpp);
% yppdot = fnder(ypp);
% zppdot = fnder(zpp);
% x = ppval(xpp,theta);
% y = ppval(ypp,theta);
% z = ppval(zpp,theta);
% yaw = atan2(ppval(yppdot,theta),ppval(xppdot,theta));
% Xref = [x; y; z; yaw; theta; dot_theta];
thetatarget = cumsum([0;((diff(targetpoints.').^2)*ones(3,1)).^(1/4)]).';
theta = linspace(thetatarget(1), thetatarget(end), kmax+1);
ppp = cscvn(targetpoints);

% periodic spline
loop = 0;
max_loop = 200;
if loop
    temp_theta = theta;
    if isequal(targetpoints(:,1),targetpoints(:,end))
        new_pp_breaks = [ppp.breaks];
        for n = 1:max_loop-1
            new_pp_breaks = [new_pp_breaks, ppp.breaks(2:end)+ppp.breaks(end)*n];
            theta = [theta, temp_theta(2:end)+temp_theta(end)*n];
        end
        new_pp_coefs = repmat(ppp.coefs, max_loop, 1);
        ppp = ppmak(new_pp_breaks, new_pp_coefs, 3);
        dot_theta = zeros(1,size(theta,2));
    else
        disp('Please make sure your starting point and end point is the same for periodic trajectory.');
    end
end

pppdot = fnder(ppp);
path = ppval(ppp,theta);
pathtan = ppval(pppdot,theta);
yaw = atan2(pathtan(2,:),pathtan(1,:));
Xref = [path; yaw; theta; dot_theta];

% timed mpcc test
% num_timed_point = 2;
% num_timed_sect = num_timed_point/2;
% timing = 1;
% timed_index = zeros(1,num_timed_point);
% timed_speed = zeros(1,num_timed_point);
% timed_index(1) = find(Xref(1,:) == targetpoints(1,2) & Xref(2,:) == targetpoints(2,2) & Xref(3,:) == targetpoints(3,2), 1);
% timed_index(2) = find(Xref(1,:) == targetpoints(1,3) & Xref(2,:) == targetpoints(2,3) & Xref(3,:) == targetpoints(3,3), 1);
% % timed_index(3) = find(Xref(1,:) == targetpoints(1,4) & Xref(2,:) == targetpoints(2,4) & Xref(3,:) == targetpoints(3,4), 1);
% % timed_index(4) = find(Xref(1,:) == targetpoints(1,5) & Xref(2,:) == targetpoints(2,5) & Xref(3,:) == targetpoints(3,5), 1);
% for i = 1:2:num_timed_point
%     timed_speed(i) = 0;
%     timed_speed(i+1) = (Xref(5,timed_index(i+1))-Xref(5,timed_index(i)))/timing;
% end
% timed_speed = [timed_speed, 0];
% indicator_gradient = 1000;
% timingpenalty = 100;
% progresspenalty = [repmat([-progresspenalty, timingpenalty], 1, num_timed_sect), -progresspenalty];
% dprefval = sumtanh(timed_speed, indicator_gradient, Xref, timed_index, num_timed_point);
% Qdtval = sumtanh(progresspenalty, indicator_gradient, Xref, timed_index, num_timed_point);
dthetapenalty = 100;
Qdtval = @(dt) (dt > 0)*(dthetapenalty + progresspenalty) - progresspenalty;
indicator_gradient = 10;
indicator_midpoint = 0.5;
% Qdtval = @(dt) 0.5*((dthetapenalty+progresspenalty)*tanh(indicator_gradient*(dt-indicator_midpoint))+dthetapenalty)-0.5*progresspenalty;

polyorder = 2;
gap = 10;

%% FORCES multistage form
% assume variable ordering zi = [ui; xi] for i=1...N

% dimensions
model.N     = 11;    % horizon length
model.nvar  = nx+nalx+nu+nalu;    % number of variables
model.neq   = nx+nalx;    % number of equality constraints
model.npar  = (4*(polyorder+1)+3*(polyorder)); % number of runtime parameters

% objective 
for i = 1:model.N-1
%     model.objective{i} = @(z,par) ...
%         [sin(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+1)-polyval(par(1:3)',z(nu+nalu+nx+1))) - ...
%         cos(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))); ...
%         -cos(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+1)-polyval(par(1:3)',z(nu+nalu+nx+1))) - ...
%         sin(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))); ...
%         z(nu+nalu+3) - polyval(par(7:9)',z(nu+nalu+nx+1))]'*Qp* ...
%         [sin(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+1)-polyval(par(1:3)',z(nu+nalu+nx+1))) - ...
%         cos(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))); ...
%         -cos(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+1)-polyval(par(1:3)',z(nu+nalu+nx+1))) - ...
%         sin(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))); ...
%         z(nu+nalu+3) - polyval(par(7:9)',z(nu+nalu+nx+1))] - Qdtheta*(z(nu+nalu+nx+2)) + ...
%         (z(nu+nalu+9:nu+nalu+nx))'*Qj*(z(nu+nalu+9:nu+nalu+nx)) + ...
%         (z(nu+nalu+nx+3:nu+nalu+nx+nalx))'*Qjtheta*(z(nu+nalu+nx+3:nu+nalu+nx+nalx));

%     model.objective{i} = @(z,par) ...
%         [sin(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+1)-polyval(par(1:3)',z(nu+nalu+nx+1))) - ...
%         cos(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))); ...
%         -cos(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+1)-polyval(par(1:3)',z(nu+nalu+nx+1))) - ...
%         sin(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1)))]'*Qp* ...
%         [sin(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+1)-polyval(par(1:3)',z(nu+nalu+nx+1))) - ...
%         cos(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))); ...
%         -cos(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+1)-polyval(par(1:3)',z(nu+nalu+nx+1))) - ...
%         sin(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1)))] - Qdtheta*(z(nu+nalu+nx+2)) + ...
%         (z(nu+nalu+9:nu+nalu+nx))'*Qj*(z(nu+nalu+9:nu+nalu+nx)) + ...
%         (z(nu+nalu+nx+3:nu+nalu+nx+nalx))'*Qjtheta*(z(nu+nalu+nx+3:nu+nalu+nx+nalx)) + ...
%             [sin(atan2(polyval(par(14:15)',z(nu+nalu+nx+1)),polyval(par(12:13)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))) - ...
%         cos(atan2(polyval(par(14:15)',z(nu+nalu+nx+1)),polyval(par(12:13)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+3)-polyval(par(7:9)',z(nu+nalu+nx+1))); ...
%         -cos(atan2(polyval(par(14:15)',z(nu+nalu+nx+1)),polyval(par(12:13)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))) - ...
%         sin(atan2(polyval(par(14:15)',z(nu+nalu+nx+1)),polyval(par(12:13)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+3)-polyval(par(7:9)',z(nu+nalu+nx+1)))]'*Qp* ...
%         [sin(atan2(polyval(par(14:15)',z(nu+nalu+nx+1)),polyval(par(12:13)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))) - ...
%         cos(atan2(polyval(par(14:15)',z(nu+nalu+nx+1)),polyval(par(12:13)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+3)-polyval(par(7:9)',z(nu+nalu+nx+1))); ...
%         -cos(atan2(polyval(par(14:15)',z(nu+nalu+nx+1)),polyval(par(12:13)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))) - ...
%         sin(atan2(polyval(par(14:15)',z(nu+nalu+nx+1)),polyval(par(12:13)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+3)-polyval(par(7:9)',z(nu+nalu+nx+1)))];

%     model.objective{i} = @(z,par) ...
%         [z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)); ...
%         z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)); ...
%         z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1))]'* ...
%         [polyval(par(10:11),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
%         polyval(par(12:13),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
%         polyval(par(14:15),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2)]*Qlag* ...
%         [z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)); ...
%         z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)); ...
%         z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1))]'* ...
%         [polyval(par(10:11),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
%         polyval(par(12:13),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
%         polyval(par(14:15),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2)] + ...
%         sqrt((z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)))^2 + ...
%         (z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)))^2 + (z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1)))^2 - ...
%         ([z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)); ...
%         z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)); ...
%         z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1))]'* ...
%         [polyval(par(10:11),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
%         polyval(par(12:13),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
%         polyval(par(14:15),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2)])^2)*Qcontour* ...
%         sqrt((z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)))^2 + ...
%         (z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)))^2 + (z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1)))^2 - ...
%         ([z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)); ...
%         z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)); ...
%         z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1))]'* ...
%         [polyval(par(10:11),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
%         polyval(par(12:13),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
%         polyval(par(14:15),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2)])^2) + ...
%         (z(nu+nalu+4) - atan2(polyval(par(12:13),z(nu+nalu+nx+1)),polyval(par(10:11),z(nu+nalu+nx+1))))'*Qorient* ...
%         (z(nu+nalu+4) - atan2(polyval(par(12:13),z(nu+nalu+nx+1)),polyval(par(10:11),z(nu+nalu+nx+1)))) - ...
%         Qdtheta*(z(nu+nalu+nx+2)) + ...
%         (z(nu+nalu+9:nu+nalu+nx))'*Qj*(z(nu+nalu+9:nu+nalu+nx)) + ...
%         (z(nu+nalu+nx+3:nu+nalu+nx+nalx))'*Qjtheta*(z(nu+nalu+nx+3:nu+nalu+nx+nalx));

    model.objective{i} = @(z,par) ...
        [z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)); ...
        z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)); ...
        z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1))]'* ...
        [polyval(par(10:11),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
        polyval(par(12:13),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
        polyval(par(14:15),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2)]*Qlag* ...
        [z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)); ...
        z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)); ...
        z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1))]'* ...
        [polyval(par(10:11),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
        polyval(par(12:13),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
        polyval(par(14:15),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2)] + ...
        sqrt((z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)))^2 + ...
        (z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)))^2 + (z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1)))^2 - ...
        ([z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)); ...
        z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)); ...
        z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1))]'* ...
        [polyval(par(10:11),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
        polyval(par(12:13),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
        polyval(par(14:15),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2)])^2)*Qcontour* ...
        sqrt((z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)))^2 + ...
        (z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)))^2 + (z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1)))^2 - ...
        ([z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)); ...
        z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)); ...
        z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1))]'* ...
        [polyval(par(10:11),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
        polyval(par(12:13),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
        polyval(par(14:15),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2)])^2) + ...
        (z(nu+nalu+4) - atan2(polyval(par(12:13),z(nu+nalu+nx+1)),polyval(par(10:11),z(nu+nalu+nx+1))))'*Qorient* ...
        (z(nu+nalu+4) - atan2(polyval(par(12:13),z(nu+nalu+nx+1)),polyval(par(10:11),z(nu+nalu+nx+1)))) + ...
        (z(nu+nalu+nx+2) - polyval(par(16:18),z(nu+nalu+nx+1)))'*Qdtval(polyval(par(16:18),z(nu+nalu+nx+1)))*(z(nu+nalu+nx+2) - polyval(par(16:18),z(nu+nalu+nx+1))) + ...
        (z(nu+nalu+9:nu+nalu+nx))'*Qj*(z(nu+nalu+9:nu+nalu+nx)) + ...
        (z(nu+nalu+nx+3:nu+nalu+nx+nalx))'*Qjtheta*(z(nu+nalu+nx+3:nu+nalu+nx+nalx));
end
% model.objective{model.N} = @(z,par) ...
%     [sin(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%     *(z(nu+nalu+1)-polyval(par(1:3)',z(nu+nalu+nx+1))) - ...
%     cos(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%     *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))); ...
%     -cos(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%     *(z(nu+nalu+1)-polyval(par(1:3)',z(nu+nalu+nx+1))) - ...
%     sin(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%     *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))); ...
%     z(nu+nalu+3) - polyval(par(7:9)',z(nu+nalu+nx+1))]'*Pp* ...
%     [sin(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%     *(z(nu+nalu+1)-polyval(par(1:3)',z(nu+nalu+nx+1))) - ...
%     cos(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%     *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))); ...
%     -cos(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%     *(z(nu+nalu+1)-polyval(par(1:3)',z(nu+nalu+nx+1))) - ...
%     sin(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%     *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))); ...
%     z(nu+nalu+3) - polyval(par(7:9)',z(nu+nalu+nx+1))] - Pdtheta*(z(nu+nalu+nx+2)) + ...
%     (z(nu+nalu+9:nu+nalu+nx))'*Pj*(z(nu+nalu+9:nu+nalu+nx)) + ...
%     (z(nu+nalu+nx+3:nu+nalu+nx+nalx))'*Pjtheta*(z(nu+nalu+nx+3:nu+nalu+nx+nalx));

% model.objective{model.N} = @(z,par) ...
%         [sin(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+1)-polyval(par(1:3)',z(nu+nalu+nx+1))) - ...
%         cos(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))); ...
%         -cos(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+1)-polyval(par(1:3)',z(nu+nalu+nx+1))) - ...
%         sin(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1)))]'*Pp* ...
%         [sin(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+1)-polyval(par(1:3)',z(nu+nalu+nx+1))) - ...
%         cos(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))); ...
%         -cos(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+1)-polyval(par(1:3)',z(nu+nalu+nx+1))) - ...
%         sin(atan2(polyval(par(12:13)',z(nu+nalu+nx+1)),polyval(par(10:11)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1)))] - Pdtheta*(z(nu+nalu+nx+2)) + ...
%         (z(nu+nalu+9:nu+nalu+nx))'*Pj*(z(nu+nalu+9:nu+nalu+nx)) + ...
%         (z(nu+nalu+nx+3:nu+nalu+nx+nalx))'*Pjtheta*(z(nu+nalu+nx+3:nu+nalu+nx+nalx)) + ...
%             [sin(atan2(polyval(par(14:15)',z(nu+nalu+nx+1)),polyval(par(12:13)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))) - ...
%         cos(atan2(polyval(par(14:15)',z(nu+nalu+nx+1)),polyval(par(12:13)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+3)-polyval(par(7:9)',z(nu+nalu+nx+1))); ...
%         -cos(atan2(polyval(par(14:15)',z(nu+nalu+nx+1)),polyval(par(12:13)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))) - ...
%         sin(atan2(polyval(par(14:15)',z(nu+nalu+nx+1)),polyval(par(12:13)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+3)-polyval(par(7:9)',z(nu+nalu+nx+1)))]'*Pp* ...
%         [sin(atan2(polyval(par(14:15)',z(nu+nalu+nx+1)),polyval(par(12:13)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))) - ...
%         cos(atan2(polyval(par(14:15)',z(nu+nalu+nx+1)),polyval(par(12:13)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+3)-polyval(par(7:9)',z(nu+nalu+nx+1))); ...
%         -cos(atan2(polyval(par(14:15)',z(nu+nalu+nx+1)),polyval(par(12:13)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+2)-polyval(par(4:6)',z(nu+nalu+nx+1))) - ...
%         sin(atan2(polyval(par(14:15)',z(nu+nalu+nx+1)),polyval(par(12:13)',z(nu+nalu+nx+1)))) ...
%         *(z(nu+nalu+3)-polyval(par(7:9)',z(nu+nalu+nx+1)))];

% model.objective{model.N} = @(z,par) ...
%     [z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)); ...
%     z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)); ...
%     z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1))]'* ...
%     [polyval(par(10:11),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
%     polyval(par(12:13),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
%     polyval(par(14:15),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2)]*Plag* ...
%     [z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)); ...
%     z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)); ...
%     z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1))]'* ...
%     [polyval(par(10:11),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
%     polyval(par(12:13),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
%     polyval(par(14:15),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2)] + ...
%     sqrt((z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)))^2 + ...
%     (z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)))^2 + (z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1)))^2 - ...
%     ([z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)); ...
%     z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)); ...
%     z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1))]'* ...
%     [polyval(par(10:11),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
%     polyval(par(12:13),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
%     polyval(par(14:15),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2)])^2)*Pcontour* ...
%     sqrt((z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)))^2 + ...
%     (z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)))^2 + (z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1)))^2 - ...
%     ([z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)); ...
%     z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)); ...
%     z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1))]'* ...
%     [polyval(par(10:11),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
%     polyval(par(12:13),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
%     polyval(par(14:15),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2)])^2) + ...
%     (z(nu+nalu+4) - atan2(polyval(par(12:13),z(nu+nalu+nx+1)),polyval(par(10:11),z(nu+nalu+nx+1))))'*Porient* ...
%     (z(nu+nalu+4) - atan2(polyval(par(12:13),z(nu+nalu+nx+1)),polyval(par(10:11),z(nu+nalu+nx+1)))) - ...
%     Pdtheta*(z(nu+nalu+nx+2)) + ...
%     (z(nu+nalu+9:nu+nalu+nx))'*Pj*(z(nu+nalu+9:nu+nalu+nx)) + ...
%     (z(nu+nalu+nx+3:nu+nalu+nx+nalx))'*Pjtheta*(z(nu+nalu+nx+3:nu+nalu+nx+nalx));

model.objective{model.N} = @(z,par) ...
    [z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)); ...
    z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)); ...
    z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1))]'* ...
    [polyval(par(10:11),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
    polyval(par(12:13),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
    polyval(par(14:15),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2)]*Plag* ...
    [z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)); ...
    z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)); ...
    z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1))]'* ...
    [polyval(par(10:11),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
    polyval(par(12:13),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
    polyval(par(14:15),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2)] + ...
    sqrt((z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)))^2 + ...
    (z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)))^2 + (z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1)))^2 - ...
    ([z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)); ...
    z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)); ...
    z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1))]'* ...
    [polyval(par(10:11),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
    polyval(par(12:13),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
    polyval(par(14:15),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2)])^2)*Pcontour* ...
    sqrt((z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)))^2 + ...
    (z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)))^2 + (z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1)))^2 - ...
    ([z(nu+nalu+1) - polyval(par(1:3),z(nu+nalu+nx+1)); ...
    z(nu+nalu+2) - polyval(par(4:6),z(nu+nalu+nx+1)); ...
    z(nu+nalu+3) - polyval(par(7:9),z(nu+nalu+nx+1))]'* ...
    [polyval(par(10:11),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
    polyval(par(12:13),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2); ...
    polyval(par(14:15),z(nu+nalu+nx+1))/sqrt(polyval(par(10:11),z(nu+nalu+nx+1))^2+polyval(par(12:13),z(nu+nalu+nx+1))^2+polyval(par(14:15),z(nu+nalu+nx+1))^2)])^2) + ...
    (z(nu+nalu+4) - atan2(polyval(par(12:13),z(nu+nalu+nx+1)),polyval(par(10:11),z(nu+nalu+nx+1))))'*Porient* ...
    (z(nu+nalu+4) - atan2(polyval(par(12:13),z(nu+nalu+nx+1)),polyval(par(10:11),z(nu+nalu+nx+1)))) + ...
    (z(nu+nalu+nx+2) - polyval(par(16:18),z(nu+nalu+nx+1)))'*Qdtval(polyval(par(16:18),z(nu+nalu+nx+1)))*(z(nu+nalu+nx+2) - polyval(par(16:18),z(nu+nalu+nx+1))) + ...
    (z(nu+nalu+9:nu+nalu+nx))'*Pj*(z(nu+nalu+9:nu+nalu+nx)) + ...
    (z(nu+nalu+nx+3:nu+nalu+nx+nalx))'*Pjtheta*(z(nu+nalu+nx+3:nu+nalu+nx+nalx));

% equalities
model.eq = @(z) Ad*z(nu+nalu+1:nu+nalu+nx+nalx) + Bd*z(1:nu+nalu) + gd;

% model.E = [zeros(nx+nalx,nu+nalu), eye(nx+nalx)];
% model.E = [zeros(nx+nalx,nu+nalu), ...
%     eye(nx+nalx) + diag([zeros(4,1); (-1/T)*ones(nx-8,1); zeros(nalx,1)], -4) + ...
%     diag([zeros(nx+1,1); (-1/T)*ones(nalx-2,1)], -1)];
model.E = [[zeros(nx-4,nu+nalu); (-1/(T*m))*[eye(4), zeros(4,1)]; (-1/T)*[zeros(3,4), [0;0;1]]], eye(nx+nalx)];
% model.E = [[zeros(nx-8,nu+nalu); (-1/(T*m))*[eye(4), zeros(4,1)]; zeros(4,5); ...
%     (-1/(T*m))*[zeros(4), [0;0;1;0]]], eye(nx+nalx) + diag([zeros(8,1); (-1/T)*ones(nx-12,1); zeros(nalx,1)], -4) + ...
%     diag([zeros(nx+2,1); (-1/T)*ones(nalx-3,1)], -1)];

% initial state
model.xinitidx = nu+nalu+1:nu+nalu+nx+nalx;

% inequalities
model.lb = [ umin,    xmin  ];
model.ub = [ umax,    xmax  ];

%% Generate FORCES solver

% get options
codeoptions = getOptions('FORCESNLPsolver');
codeoptions.printlevel = 2;
codeoptions.maxit = 5000;

% generate code
FORCES_NLP(model, codeoptions);

%% simulate 1

X = zeros(nx+nalx,kmax);
% X(1:3,1) = [1;1;1];
U = zeros(nu+nalu,kmax);
solvetime = zeros(1,kmax);
iters = zeros(1,kmax);
timeElapsed = zeros(1,kmax);
problem.x0 = zeros(model.N*model.nvar,1); % stack up problems into one N stages array
k = 1;
[~, nrid(k)] = min(pdist2(X(1:3,k)', Xref(1:3,:)'));
searchlength = 40;
fitlength = 20;
figure;

while (nrid(k) ~= kmax+1)
    tic
    problem.xinit = X(:,k);
    
    % locally fit quadratic splines
    if ~loop
        if (nrid(k) >= kmax-fitlength)
            fitrange = nrid(k)-fitlength:kmax+1;
        else
            fitrange = nrid(k):nrid(k)+fitlength;
        end
    else
        fitrange = nrid(k):nrid(k)+fitlength; % inifinity loop (hack)
    end
%     px = polyfit(theta(fitrange),x(fitrange),polyorder);
%     py = polyfit(theta(fitrange),y(fitrange),polyorder);
%     pz = polyfit(theta(fitrange),z(fitrange),polyorder);
    px = polyfit(theta(fitrange),path(1,fitrange),polyorder);
    py = polyfit(theta(fitrange),path(2,fitrange),polyorder);
    pz = polyfit(theta(fitrange),path(3,fitrange),polyorder);
    ptv = polyfit(theta(fitrange),dot_theta(fitrange),polyorder);
    dpx = polyder(px);
    dpy = polyder(py);
    dpz = polyder(pz);
    if (length(dpx) ~= polyorder)
        if (find(px == 0) == 1)
            dpx(2) = dpx(1);
            dpx(1) = 0;
        else
            dpx(2) = 0;
        end
    end
    if (length(dpy) ~= polyorder)
        if (find(py == 0) == 1)
            dpy(2) = dpy(1);
            dpy(1) = 0;
        else
            dpy(2) = 0;
        end
    end
    if (length(dpz) ~= polyorder)
        if (find(pz == 0) == 1)
            dpz(2) = dpz(1);
            dpz(1) = 0;
        else
            dpz(2) = 0;
        end
    end
    qs = [px'; py'; pz'; dpx'; dpy'; dpz'; ptv'];
    
    problem.all_parameters = repmat(qs, model.N, 1);
    
    [solverout,exitflag,info] = FORCESNLPsolver(problem);

    if( exitflag == 1 )
        U(:,k) = solverout.x01(1:nu+nalu);
        solvetime(k) = info.solvetime;
        iters(k) = info.it;
    else
        error('Some problem in solver');
    end
    
    X(:,k+1) = model.eq( [U(:,k); X(:,k)] )';
    
    % mpc output
    Xout = zeros(nx+nalx,model.N);
    Uout = zeros(nu+nalu,model.N-1);
    names = fieldnames(solverout);
    for j = 1:model.N
        tempout = getfield(solverout, names{j});
        Xout(:,j) = tempout(nu+nalu+1:nu+nalu+nx+nalx);
        if (j < model.N)
            Uout(:,j) = tempout(1:nu+nalu);
        end
    end
    
    % real-time plot
    clf;
%     axis([-1.5 1.5 -1.5 1.5 -0.1 1.1]);
    axis([-2 2 -2 2 -0.1 1.1]);
    hold on;
    plot3(Xref(1,:),Xref(2,:),Xref(3,:),'--');
    plot3(X(1,k),X(2,k),X(3,k),'or');
    plot3(Xout(1,:),Xout(2,:),Xout(3,:),'.m');
    plot3(polyval(px,theta(fitrange)),polyval(py,theta(fitrange)),polyval(pz,theta(fitrange)),'LineWidth',2);
    drawnow;
    hold off;
    
    % search nearest reference point for the next time step
%     [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,:)')); % search through the whole trajectory
    if ~loop
        if (nrid(k) >= kmax-searchlength)
            [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,nrid(k):kmax+1)'));
        else
            [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,nrid(k):nrid(k)+searchlength)'));
        end % search only in a smaller search range around the previous nearest reference point
    else
        [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,nrid(k):nrid(k)+searchlength)'));
    end
    nrid(k+1) = nrid(k+1) + nrid(k) - 1; % non-reversal path
    timeElapsed(k) = toc;
    k = k + 1;
end

%% plot

s = X(13,1:k);
ref_state = ppval(ppp,s);
ref_vel = ppval(pppdot,s); 

% figure;
% subplot(3,1,1);
% plot(s,ppval(xpp,s),'--r',s,ppval(ypp,s),'--g',s,ppval(zpp,s),'--b');
% hold on;
% plot(s,X(1,1:k),'r',s,X(2,1:k),'g',s,X(3,1:k),'b');
% hold off;
% title('Position Plot');
% legend('xref','yref','zref','x','y','z');
% 
% subplot(3,1,2);
% plot(s,ppval(xppdot,s),'--m',s,ppval(yppdot,s),'--c',s,ppval(zppdot,s),'--k');
% hold on;
% plot(s,X(5,1:k),'m',s,X(6,1:k),'c',s,X(7,1:k),'k');
% hold off;
% title('Velocity Plot');
% legend('vxref','vyref','vzref','vx','vy','vz');
% 
% subplot(3,1,3);
% haha = s(2:end);
% plot(haha,U(1,1:k-1),'r',haha,U(2,1:k-1),'g',haha,U(3,1:k-1),'b');
% title('Input Plot');
% legend('Fx','Fy','Fz');

figure;
subplot(3,1,1);
plot(s,ref_state(1,:),'--r',s,ref_state(2,:),'--g',s,ref_state(3,:),'--b');
hold on;
plot(s,X(1,1:k),'r',s,X(2,1:k),'g',s,X(3,1:k),'b');
hold off;
title('Position Plot');
legend('xref','yref','zref','x','y','z');

subplot(3,1,2);
plot(s,ref_vel(1,:),'--m',s,ref_vel(2,:),'--c',s,ref_vel(3,:),'--k');
hold on;
plot(s,X(5,1:k),'m',s,X(6,1:k),'c',s,X(7,1:k),'k');
hold off;
title('Velocity Plot');
legend('vxref','vyref','vzref','vx','vy','vz');

subplot(3,1,3);
haha = s(2:end);
plot(haha,U(1,1:k-1),'r',haha,U(2,1:k-1),'g',haha,U(3,1:k-1),'b');
title('Input Plot');
legend('Fx','Fy','Fz');

time = 0:T:T*(k-1);
theta_to_change = 0;
new_time = 0;

figure;
subplot(2,1,1);
plot(X(13,1:k),time);
title('Time vs Theta');
subplot(2,1,2);
plot(X(13,1:k),X(14,1:k));
title('Theta Velocity vs Theta');

optimized_time = time;
optimized_theta = X(13,1:k);
figure;
while (1)
    decision_prompt = 'Do you want to change the trajectory? [Y/n]: ';
    usr_decision = input(decision_prompt,'s');
    if (isempty(usr_decision) || usr_decision == 'Y' || usr_decision == 'y')
        theta_prompt = 'Enter the specific theta values: ';
        theta_to_change = input(theta_prompt);
        time_prompt = 'Enter the new timings: ';
        new_time = input(time_prompt);
        theta_to_change_ind = [];
        for the = theta_to_change
            [~, nearest_theta] = min(pdist2(optimized_theta',the));
            theta_to_change_ind = [theta_to_change_ind, nearest_theta];
        end
        old_time = optimized_time(theta_to_change_ind(2)) - optimized_time(theta_to_change_ind(1));
        optimized_time(theta_to_change_ind(1)+1:theta_to_change_ind(2)-1) = 0;
        optimized_time(theta_to_change_ind(2)) = optimized_time(theta_to_change_ind(1)) + new_time;
        optimized_time(theta_to_change_ind(2)+1:end) = optimized_time(theta_to_change_ind(2)+1:end) + new_time - old_time;
        optimized_time(theta_to_change_ind(1)+1:theta_to_change_ind(2)-1) = [];
        optimized_theta(theta_to_change_ind(1)+1:theta_to_change_ind(2)-1) = [];
        time_spline = csape(optimized_theta,optimized_time);
        theta_spline = csape(optimized_time,optimized_theta);
%         time_spline = csape(theta_to_change,new_time,'periodic');
%         theta_spline = csape(new_time,theta_to_change,'periodic');
        tv_spline = fnder(theta_spline);
        if loop
%             time = ppval(time_spline,temp_theta);
%             temp_time = time;
%             dot_theta = ppval(tv_spline,ppval(time_spline,temp_theta));
%             if (abs(dot_theta(1) - dot_theta(end)) < 0.01)
%                 % TODO: is this correct??? how to test it???
%                 new_tv_breaks = [tv_spline.breaks];
%                 for n = 1:max_loop-1
%                     new_tv_breaks = [new_tv_breaks, tv_spline.breaks(2:end)+tv_spline.breaks(end)*n];
%                     time = [time, temp_time(2:end)+temp_time(end)*n];
%                 end
%                 new_tv_coefs = repmat(tv_spline.coefs, max_loop, 1);
%                 tv_spline = ppmak(new_tv_breaks, new_tv_coefs, 1);
%                 dot_theta = ppval(tv_spline,time);
%             else
%                 disp('The progress velocity is not periodic. Please select your timings again.');
%                 continue;
%             end
        else
            time = ppval(time_spline,theta);
            dot_theta = ppval(tv_spline,ppval(time_spline,theta));
%             % test combining maximizing progress velocity and following reference progress velocity 
%             dot_theta = (theta_to_change(2) - theta_to_change(1))/new_time;
%             dot_theta = repmat(dot_theta, 1, length(theta));
            theta_to_remove_ind = [];
            for the = theta_to_change
                [~, nearest_theta] = min(pdist2(theta',the));
                theta_to_remove_ind = [theta_to_remove_ind, nearest_theta];
            end % hack
            dot_theta(1:theta_to_remove_ind(1)-1) = 0; % hack
            dot_theta(theta_to_remove_ind(end)+1:end) = 0; % hack
        end 
        clf(gcf);
        subplot(2,1,1);
        plot(theta,time);
        title('New Time vs Theta');
        subplot(2,1,2);
        plot(theta,dot_theta);
        title('New Theta Velocity vs Theta');
    elseif (usr_decision == 'N' || usr_decision == 'n')
        break;
    else
        disp('Please enter a valid answer (y/n).');
    end
end

%% simulate 2

X = zeros(nx+nalx,kmax);
% X(1:3,1) = [1;1;1];
U = zeros(nu+nalu,kmax);
solvetime = zeros(1,kmax);
iters = zeros(1,kmax);
timeElapsed = zeros(1,kmax);
problem.x0 = zeros(model.N*model.nvar,1); % stack up problems into one N stages array
k = 1;
[~, nrid(k)] = min(pdist2(X(1:3,k)', Xref(1:3,:)'));
searchlength = 40;
fitlength = 20;
figure;

user_ind = find(dot_theta ~= 0);
while (nrid(k) ~= kmax+1)
    tic
    problem.xinit = X(:,k);
    
    % locally fit quadratic splines
    if ~loop
        if (nrid(k) >= kmax-fitlength)
            fitrange = nrid(k)-fitlength:kmax+1;
        else
            fitrange = nrid(k):nrid(k)+fitlength;
        end
    else
        fitrange = nrid(k):nrid(k)+fitlength; % inifinity loop (hack)
    end
%     px = polyfit(theta(fitrange),x(fitrange),polyorder);
%     py = polyfit(theta(fitrange),y(fitrange),polyorder);
%     pz = polyfit(theta(fitrange),z(fitrange),polyorder);
    px = polyfit(theta(fitrange),path(1,fitrange),polyorder);
    py = polyfit(theta(fitrange),path(2,fitrange),polyorder);
    pz = polyfit(theta(fitrange),path(3,fitrange),polyorder);
%     ptv = polyfit(theta(fitrange),dot_theta(fitrange),polyorder);
    dpx = polyder(px);
    dpy = polyder(py);
    dpz = polyder(pz);
    if (length(dpx) ~= polyorder)
        if (find(px == 0) == 1)
            dpx(2) = dpx(1);
            dpx(1) = 0;
        else
            dpx(2) = 0;
        end
    end
    if (length(dpy) ~= polyorder)
        if (find(py == 0) == 1)
            dpy(2) = dpy(1);
            dpy(1) = 0;
        else
            dpy(2) = 0;
        end
    end
    if (length(dpz) ~= polyorder)
        if (find(pz == 0) == 1)
            dpz(2) = dpz(1);
            dpz(1) = 0;
        else
            dpz(2) = 0;
        end
    end
    if (nrid(k) >= min(user_ind) && nrid(k) <= max(user_ind))
        ptv = polyfit(theta(user_ind),dot_theta(user_ind),polyorder);
    else
        ptv = polyfit(theta(fitrange),zeros(1,length(fitrange)),polyorder);
    end
    qs = [px'; py'; pz'; dpx'; dpy'; dpz'; ptv'];
    
    problem.all_parameters = repmat(qs, model.N, 1);
    
    [solverout,exitflag,info] = FORCESNLPsolver(problem);

    if( exitflag == 1 )
        U(:,k) = solverout.x01(1:nu+nalu);
        solvetime(k) = info.solvetime;
        iters(k) = info.it;
    else
        error('Some problem in solver');
    end
    
    X(:,k+1) = model.eq( [U(:,k); X(:,k)] )';
    
    % mpc output
    Xout = zeros(nx+nalx,model.N);
    Uout = zeros(nu+nalu,model.N-1);
    names = fieldnames(solverout);
    for j = 1:model.N
        tempout = getfield(solverout, names{j});
        Xout(:,j) = tempout(nu+nalu+1:nu+nalu+nx+nalx);
        if (j < model.N)
            Uout(:,j) = tempout(1:nu+nalu);
        end
    end
    
    % real-time plot
    clf;
%     axis([-1.5 1.5 -1.5 1.5 -0.1 1.1]);
    axis([-2 2 -2 2 -0.1 1.1]);
    hold on;
    plot3(Xref(1,:),Xref(2,:),Xref(3,:),'--');
    plot3(X(1,k),X(2,k),X(3,k),'or');
    plot3(Xout(1,:),Xout(2,:),Xout(3,:),'.m');
    plot3(polyval(px,theta(fitrange)),polyval(py,theta(fitrange)),polyval(pz,theta(fitrange)),'LineWidth',2);
    drawnow;
    hold off;
    
    % search nearest reference point for the next time step
%     [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,:)')); % search through the whole trajectory
    if ~loop
        if (nrid(k) >= kmax-searchlength)
            [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,nrid(k):kmax+1)'));
        else
            [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,nrid(k):nrid(k)+searchlength)'));
        end % search only in a smaller search range around the previous nearest reference point
    else
        [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,nrid(k):nrid(k)+searchlength)'));
    end
    nrid(k+1) = nrid(k+1) + nrid(k) - 1; % non-reversal path
    timeElapsed(k) = toc;
    k = k + 1;
end

s = X(13,1:k);
ref_state = ppval(ppp,s);
ref_vel = ppval(pppdot,s); 

figure;
subplot(3,1,1);
plot(s,ref_state(1,:),'--r',s,ref_state(2,:),'--g',s,ref_state(3,:),'--b');
hold on;
plot(s,X(1,1:k),'r',s,X(2,1:k),'g',s,X(3,1:k),'b');
hold off;
title('Position Plot');
legend('xref','yref','zref','x','y','z');

subplot(3,1,2);
plot(s,ref_vel(1,:),'--m',s,ref_vel(2,:),'--c',s,ref_vel(3,:),'--k');
hold on;
plot(s,X(5,1:k),'m',s,X(6,1:k),'c',s,X(7,1:k),'k');
hold off;
title('Velocity Plot');
legend('vxref','vyref','vzref','vx','vy','vz');

subplot(3,1,3);
haha = s(2:end);
plot(haha,U(1,1:k-1),'r',haha,U(2,1:k-1),'g',haha,U(3,1:k-1),'b');
title('Input Plot');
legend('Fx','Fy','Fz');

time = 0:T:T*(k-1);
figure;
subplot(2,1,1);
plot(X(13,1:k),time,'r');
title('Time vs Theta');
subplot(2,1,2);
plot(X(13,1:k),X(14,1:k),'r');
title('Theta Velocity vs Theta');
