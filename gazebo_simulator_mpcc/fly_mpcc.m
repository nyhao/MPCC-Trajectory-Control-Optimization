clear;
clc;

inputpub = rospublisher('/cmd_vel','geometry_msgs/Twist');
posesub = rossubscriber('/ground_truth_to_tf/pose');

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

Bc = zeros(nx+nalx,nu+nalu);
Bc(5,1) = 1/m;
Bc(6,2) = 1/m;
Bc(7,3) = 1/m;
Bc(8,4) = 1/m;

Bc(14,5) = 1;

gc = zeros(nx+nalx,1);
gc(7,1) = g;

I = eye(nx+nalx);

% discretization
A_tilde = [Ac, Bc, I; zeros(nx + nalx + nu + nalu, nx + nalx + nu + nalu + nx + nalx)]; 
A_tilde_d = expm(A_tilde*T);

Ad = A_tilde_d(1:nx+nalx,1:nx+nalx);
Bd = A_tilde_d(1:nx+nalx,nx+nalx+1:nx+nalx+nu+nalu);
gd = A_tilde_d(1:nx+nalx,nx+nalx+nu+nalu+1:2*(nx+nalx)+nu+nalu) * gc;

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

% theta'''(i+1) = theta''(i+1) - theta''(i)
Ad(15,15) = 0;
Bd(15,5) = -1/T;

%% MPCC setup
% for constant Q and P matrices
contourpenalty = 100; 
zpenalty = 0.000005; % z
xypenalty = 0.5; % xy
lagpenalty = 100; 
orientpenalty = 0;
progresspenalty = 0.00000001;
fd_factor = 0.1;
Qlag = lagpenalty;
Qcontour = contourpenalty;
Qorient = orientpenalty;
Qj = fd_factor*eye(4);
Qdtheta = progresspenalty; 
Qjtheta = fd_factor;
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
kmax = 320;
theta = 0:0.05:0.05*kmax;
dot_theta = zeros(1,kmax+1);
xtarget = [0, 2, 4, 6, 7];
ytarget = [0, 0, 0, 0, 0];
ztarget = [1, 1, 1, 1, 1];
thetatarget = linspace(theta(1), theta(end), length(xtarget));
xpp = spline(thetatarget,xtarget);
ypp = spline(thetatarget,ytarget);
zpp = spline(thetatarget,ztarget);
xppdot = fnder(xpp);
yppdot = fnder(ypp);
zppdot = fnder(zpp);
x = ppval(xpp,theta);
y = ppval(ypp,theta);
z = ppval(zpp,theta);
yaw = atan2(ppval(yppdot,theta),ppval(xppdot,theta));
Xref = [x; y; z; yaw; theta; dot_theta];

polyorder = 2;

%% FORCES multistage form
% assume variable ordering zi = [ui; xi] for i=1...N

% % dimensions
% model.N     = 11;    % horizon length
% model.nvar  = nx+nalx+nu+nalu;    % number of variables
% model.neq   = nx+nalx;    % number of equality constraints
% model.npar  = 3*(polyorder+1)+3*(polyorder); % number of runtime parameters
% 
% % objective 
% for i = 1:model.N-1
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
% end
% 
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
% 
% % equalities
% model.eq = @(z) Ad*z(nu+nalu+1:nu+nalu+nx+nalx) + Bd*z(1:nu+nalu) + gd;
% 
% model.E = [[zeros(nx-4,nu+nalu); (-1/(T*m))*[eye(4), zeros(4,1)]; (-1/T)*[zeros(3,4), [0;0;1]]], eye(nx+nalx)];
% 
% % initial state
% model.xinitidx = nu+nalu+1:nu+nalu+nx+nalx;
% 
% % inequalities
% model.lb = [ umin,    xmin  ];
% model.ub = [ umax,    xmax  ];

model.N     = 11;    % horizon length
model.nvar  = 28;    % number of variables
model.neq   = 21;    % number of equality constraints
model.npar  = 3*(polyorder+1)+3*(polyorder); % number of runtime parameters
% model.objective = @(z,par) ...
%         [z(8) - polyval(par(1:3),z(20)); ...
%         z(9) - polyval(par(4:6),z(20)); ...
%         z(10) - polyval(par(7:9),z(20))]'* ...
%         [polyval(par(10:11),z(20))/sqrt(polyval(par(10:11),z(20))^2+polyval(par(12:13),z(20))^2+polyval(par(14:15),z(20))^2); ...
%         polyval(par(12:13),z(20))/sqrt(polyval(par(10:11),z(20))^2+polyval(par(12:13),z(20))^2+polyval(par(14:15),z(20))^2); ...
%         polyval(par(14:15),z(20))/sqrt(polyval(par(10:11),z(20))^2+polyval(par(12:13),z(20))^2+polyval(par(14:15),z(20))^2)]*Qlag* ...
%         [z(8) - polyval(par(1:3),z(20)); ...
%         z(9) - polyval(par(4:6),z(20)); ...
%         z(10) - polyval(par(7:9),z(20))]'* ...
%         [polyval(par(10:11),z(20))/sqrt(polyval(par(10:11),z(20))^2+polyval(par(12:13),z(20))^2+polyval(par(14:15),z(20))^2); ...
%         polyval(par(12:13),z(20))/sqrt(polyval(par(10:11),z(20))^2+polyval(par(12:13),z(20))^2+polyval(par(14:15),z(20))^2); ...
%         polyval(par(14:15),z(20))/sqrt(polyval(par(10:11),z(20))^2+polyval(par(12:13),z(20))^2+polyval(par(14:15),z(20))^2)] + ...
%         sqrt((z(8) - polyval(par(1:3),z(20)))^2 + ...
%         (z(9) - polyval(par(4:6),z(20)))^2 + (z(10) - polyval(par(7:9),z(20)))^2 - ...
%         ([z(8) - polyval(par(1:3),z(20)); ...
%         z(9) - polyval(par(4:6),z(20)); ...
%         z(10) - polyval(par(7:9),z(20))]'* ...
%         [polyval(par(10:11),z(20))/sqrt(polyval(par(10:11),z(20))^2+polyval(par(12:13),z(20))^2+polyval(par(14:15),z(20))^2); ...
%         polyval(par(12:13),z(20))/sqrt(polyval(par(10:11),z(20))^2+polyval(par(12:13),z(20))^2+polyval(par(14:15),z(20))^2); ...
%         polyval(par(14:15),z(20))/sqrt(polyval(par(10:11),z(20))^2+polyval(par(12:13),z(20))^2+polyval(par(14:15),z(20))^2)])^2)*Qcontour* ...
%         sqrt((z(8) - polyval(par(1:3),z(20)))^2 + ...
%         (z(9) - polyval(par(4:6),z(20)))^2 + (z(10) - polyval(par(7:9),z(20)))^2 - ...
%         ([z(8) - polyval(par(1:3),z(20)); ...
%         z(9) - polyval(par(4:6),z(20)); ...
%         z(10) - polyval(par(7:9),z(20))]'* ...
%         [polyval(par(10:11),z(20))/sqrt(polyval(par(10:11),z(20))^2+polyval(par(12:13),z(20))^2+polyval(par(14:15),z(20))^2); ...
%         polyval(par(12:13),z(20))/sqrt(polyval(par(10:11),z(20))^2+polyval(par(12:13),z(20))^2+polyval(par(14:15),z(20))^2); ...
%         polyval(par(14:15),z(20))/sqrt(polyval(par(10:11),z(20))^2+polyval(par(12:13),z(20))^2+polyval(par(14:15),z(20))^2)])^2) + ...
%         (z(11) - atan2(polyval(par(12:13),z(20)),polyval(par(10:11),z(20))))'*Qorient* ...
%         (z(11) - atan2(polyval(par(12:13),z(20)),polyval(par(10:11),z(20)))) - ...
%         Qdtheta*(z(21)) + ...
%         (z(14:17))'*Qj*(z(14:17)) + ...
%         (z(22))'*Qjtheta*(z(22));
model.objective = @(z,par) ... (z(10)-10)'*zpenalty*(z(10)-10); + ...
    (z(8)-10)'*xypenalty*(z(8)-10);% + ...
%     (z(9)-10)'*xypenalty*(z(9)-10);
model.eq = @kinedy;
model.E = [[zeros(14,7); [0,0,0,0,-(1/T),0,0]; zeros(4,7); [0,0,-(1/T),0,0,0,0] ; [0,0,0,-(1/T),0,0,0]], ...
    eye(21) + [zeros(21,17), [zeros(6,4); -(1/T)*eye(4); zeros(11,4)]]];
model.xinitidx = 8:28;
umin = [-30*(pi/180), -30*(pi/180), -2, -3, -10, -inf, -inf];
umax = [30*(pi/180), 30*(pi/180), 2, 3, 10, inf, inf];
xmin = -inf*ones(1,21);
xmin(14) = 0;
xmax = inf*ones(1,21);
model.lb = [ umin,    xmin  ];
model.ub = [ umax,    xmax  ];

%% Generate FORCES solver

% get options
codeoptions = getOptions('FORCESNLPsolver');
codeoptions.printlevel = 2;
codeoptions.maxit = 5000;

% generate code
FORCES_NLP(model, codeoptions);

%% simulate

X = zeros(21,kmax);
U = zeros(7,kmax);
solvetime = zeros(1,kmax);
iters = zeros(1,kmax);
timeElapsed = zeros(1,kmax);
problem.x0 = zeros(model.N*model.nvar,1); % stack up problems into one N stages array
k = 1;
[~, nrid(k)] = min(pdist2(X(1:3,k)', Xref(1:3,:)'));
searchlength = 40;
fitlength = 20;

posemsg = posesub.LatestMessage;
X(1,k) = posemsg.Pose.Position.X;
X(2,k) = posemsg.Pose.Position.Y;
X(3,k) = posemsg.Pose.Position.Z;
X(4,k) = posemsg.Pose.Orientation.Z;

while (nrid(k) ~= kmax+1)
    tic
    
    problem.xinit = X(:,k);
    
    % locally fit quadratic splines
    if (nrid(k) >= kmax-fitlength)
        fitrange = nrid(k)-fitlength:kmax+1;
    else
        fitrange = nrid(k):nrid(k)+fitlength;
    end
    
    px = polyfit(theta(fitrange),x(fitrange),polyorder);
    py = polyfit(theta(fitrange),y(fitrange),polyorder);
    pz = polyfit(theta(fitrange),z(fitrange),polyorder);
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
    qs = [px'; py'; pz'; dpx'; dpy'; dpz'];
    
    problem.all_parameters = repmat(qs, model.N, 1);
    
    [solverout,exitflag,info] = FORCESNLPsolver(problem);

    if( exitflag == 1 )
        U(:,k) = solverout.x01(1:7);
        solvetime(k) = info.solvetime;
        iters(k) = info.it;
    else
        error('Some problem in solver');
    end
    
    X(:,k+1) = model.eq( [U(:,k); X(:,k)] )';
    
    inputmsg = rosmessage(inputpub);
    inputmsg.Linear.X = rad2deg(U(1,k))/30;
    inputmsg.Linear.Y = rad2deg(U(2,k))/30;
    inputmsg.Linear.Z = U(3,k);
    inputmsg.Angular.Z = 0;%U(4,k)/2;
    send(inputpub,inputmsg);
    
    posemsg = posesub.LatestMessage;
    X(1,k+1) = posemsg.Pose.Position.X;
    X(2,k+1) = posemsg.Pose.Position.Y;
    X(3,k+1) = posemsg.Pose.Position.Z;
    X(4,k+1) = posemsg.Pose.Orientation.Z;
    
    % search nearest reference point for the next time step
%     [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,:)')); % search through the whole trajectory
    if (nrid(k) >= kmax-searchlength)
        [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,nrid(k):kmax+1)'));
    else
        [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,nrid(k):nrid(k)+searchlength)'));
    end % search only in a smaller search range around the previous nearest reference point
    nrid(k+1) = nrid(k+1) + nrid(k) - 1; % non-reversal path
    timeElapsed(k) = toc;
    k = k + 1;
end
inputmsg = rosmessage(inputpub);
inputmsg.Linear.X = 0;
inputmsg.Linear.Y = 0;
inputmsg.Linear.Z = 0;
inputmsg.Angular.Z = 0;
send(inputpub,inputmsg);

%% functions

function next = kinedy(z)
dt = 1/30;

alpha  = 0.81;

rp = [z(1);z(2)];
rp_del_k = rp+alpha*(z(18:19) - rp);

acc = [tan(rp(1));tan(rp(2))]*9.81;
acc_k = [acc;-z(3)/dt;-z(4)/dt];

C_drag = [0.57;0.57];

pos_k = z(8:9) + z(12:13)*dt + 1/2*dt^2*(acc-C_drag.*z(12:13));
vel_k = z(12:13) + dt*(acc-C_drag.*z(12:13));

pos_z_k = z(10) + z(3)*dt;
Quadyaw_k = z(11) + dt*z(4);

xy_jerk_k = [-z(25)/dt;-z(26)/dt];
zyaw_jerk_k = [-z(27)/dt;-z(28)/dt];

theta_k     = z(20) + dt*z(21) + 1/2*dt^2*z(5);
theta_vel_k = z(21) + dt*z(5);

theta_jerk_k = -z(5)/dt;

GimbalPitch_k   = z(23) + dt*z(6);
GimbalYaw_k     = z(24) + dt*z(7);

next=[pos_k;pos_z_k;Quadyaw_k;vel_k;xy_jerk_k;zyaw_jerk_k;rp_del_k;theta_k;theta_vel_k;theta_jerk_k; ...
    GimbalPitch_k;GimbalYaw_k;acc_k];
end