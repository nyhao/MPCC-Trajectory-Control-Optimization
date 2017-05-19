clear;
clc;
close all;

%% system
T = 1/30; % sampling time for 30Hz
m = 0.5; % mass
global nx
nx = 12; % 4d position (angle around z-axis) and up to 4th derivatives
global nalx
nalx = 3; % arc length and up to 4th derivatives
global nu
nu = 4; % force acting on the acceleration states
global nalu
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
contourpenalty = 10000; 
lagpenalty = 10000; 
orientpenalty = 0;
progresspenalty = 2;
fd_factor = 2;
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
kmax = 160;
dot_theta = zeros(1,kmax+1);
targetpoints = [0, 1, 1, 0, 0; 0, 0, 1, 1, 0; 0, 0, 0, 0, 0];
thetatarget = cumsum([0;((diff(targetpoints.').^2)*ones(3,1)).^(1/4)]).';
theta = linspace(thetatarget(1), thetatarget(end), kmax+1);
ppp = cscvn(targetpoints);
pppdot = fnder(ppp);
path = ppval(ppp,theta);
pathtan = ppval(pppdot,theta);
yaw = atan2(pathtan(2,:),pathtan(1,:));
Xref = [path; yaw; theta; dot_theta];

% extra_theta = theta(end) + theta(2) - theta(1);
% while extra_theta(end) < 100
%     extra_theta = [extra_theta, extra_theta(end) + theta(2) - theta(1)];
% end
% theta_to_fit = [theta, extra_theta];
% extra_path = repmat(path(:,end),1,numel(extra_theta));
% path_to_fit = [path, extra_path];
% extra_yaw = repmat(yaw(end),1,numel(extra_theta));
% yaw_to_fit = [yaw, extra_yaw];
% extra_dot_theta = repmat(dot_theta(end),1,numel(extra_theta));
% dot_theta_to_fit = [dot_theta, extra_dot_theta];

dthetapenalty = 10000;
restpenalty = 100;
Qdtval = @(dt,tt,end_theta) (dt > 0)*(dthetapenalty + progresspenalty) - ...
    progresspenalty ;%+ (tt >= end_theta)*(restpenalty + progresspenalty);
abstpenalty = 1000000;
Qabst = @(abst) (abst > 0)*(abstpenalty);

polyorder = 2;
% gap = 10;

% xmax(1,13) = max(theta); % limiting progress of theta

%% FORCES multistage form
% assume variable ordering zi = [ui; xi] for i=1...N

% dimensions
model.N     = kmax+1;    % horizon length
model.nvar  = nx+nalx+nu+nalu;    % number of variables
model.neq   = nx+nalx;    % number of equality constraints
model.npar  = (4*(polyorder+1)+3*(polyorder))+2; % number of runtime parameters

% objective 
for i = 1:model.N-1
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
        (z(nu+nalu+nx+2) - polyval(par(16:18),z(nu+nalu+nx+1)))'*Qdtval(polyval(par(16:18),z(nu+nalu+nx+1)),z(nu+nalu+nx+1),par(20))*(z(nu+nalu+nx+2) - polyval(par(16:18),z(nu+nalu+nx+1))) + ...
        (z(nu+nalu+9:nu+nalu+nx))'*Qj*(z(nu+nalu+9:nu+nalu+nx)) + ...
        (z(nu+nalu+nx+3:nu+nalu+nx+nalx))'*Qjtheta*(z(nu+nalu+nx+3:nu+nalu+nx+nalx)) + ...
        (z(nu+nalu+nx+1) - par(19))'*Qabst(par(19))*(z(nu+nalu+nx+1) - par(19));
end
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
    (z(nu+nalu+nx+2) - polyval(par(16:18),z(nu+nalu+nx+1)))'*Qdtval(polyval(par(16:18),z(nu+nalu+nx+1)),z(nu+nalu+nx+1),par(20))*(z(nu+nalu+nx+2) - polyval(par(16:18),z(nu+nalu+nx+1))) + ...
    (z(nu+nalu+9:nu+nalu+nx))'*Pj*(z(nu+nalu+9:nu+nalu+nx)) + ...
    (z(nu+nalu+nx+3:nu+nalu+nx+nalx))'*Pjtheta*(z(nu+nalu+nx+3:nu+nalu+nx+nalx)) + ...
    (z(nu+nalu+nx+1) - par(19))'*Qabst(par(19))*(z(nu+nalu+nx+1) - par(19));

% equalities
model.eq = @(z) Ad*z(nu+nalu+1:nu+nalu+nx+nalx) + Bd*z(1:nu+nalu) + gd;

model.E = [[zeros(nx-4,nu+nalu); (-1/(T*m))*[eye(4), zeros(4,1)]; (-1/T)*[zeros(3,4), [0;0;1]]], eye(nx+nalx)];

% initial state
model.xinitidx = nu+nalu+1:nu+nalu+nx+nalx;
% model.xfinalidx = 1:nu+nalu;

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

progress_id = ones(1,kmax+1);
old_progress_id = 2*ones(1,kmax+1);
old_param = zeros(model.npar, kmax+1);
param = zeros(model.npar, kmax+1);
epsilon = 1;
solution = 5000;
exitflag = 0;
test = 1;

while (exitflag ~= 1 || solution > epsilon)
    old_param = param;
    old_progress_id = progress_id;
    X = zeros(nx+nalx,kmax);
    U = zeros(nu+nalu,kmax);
    solvetime = zeros(1,kmax);
    iters = zeros(1,kmax);
    timeElapsed = zeros(1,kmax);
    problem.x0 = zeros(model.N*model.nvar,1); % stack up problems into one N stages array
    k = 1;
    [~, nrid(k)] = min(pdist2(X(1:3,k)', Xref(1:3,:)'));
    searchlength = 40;
    fitlength = 40;

    problem.xinit = X(:,k);
%     problem.xfinal = zeros(nu+nalu,1);


%     param = zeros(model.npar, (kmax/gap)+1);
%     for k = 1:10:kmax+1
%         if (k >= kmax-fitlength)
%             fitrange = k-fitlength:kmax+1;
%         else
%             fitrange = k:k+fitlength;
%         end
%         px = polyfit(theta(fitrange),path(1,fitrange),polyorder);
%         py = polyfit(theta(fitrange),path(2,fitrange),polyorder);
%         pz = polyfit(theta(fitrange),path(3,fitrange),polyorder);
%         ptv = polyfit(theta(fitrange),dot_theta(fitrange),polyorder);
%         dpx = polyder(px);
%         dpy = polyder(py);
%         dpz = polyder(pz);
%         if (length(dpx) ~= polyorder)
%             if (find(px == 0) == 1)
%                 dpx(2) = dpx(1);
%                 dpx(1) = 0;
%             else
%                 dpx(2) = 0;
%             end
%         end
%         if (length(dpy) ~= polyorder)
%             if (find(py == 0) == 1)
%                 dpy(2) = dpy(1);
%                 dpy(1) = 0;
%             else
%                 dpy(2) = 0;
%             end
%         end
%         if (length(dpz) ~= polyorder)
%             if (find(pz == 0) == 1)
%                 dpz(2) = dpz(1);
%                 dpz(1) = 0;
%             else
%                 dpz(2) = 0;
%             end
%         end
%         param(:,((k-1)/gap)+1) = [px'; py'; pz'; dpx'; dpy'; dpz'; ptv'];
%     end
% 
%     problem.all_parameters = repmat(param(:,1:end-1), gap, 1);
%     problem.all_parameters = [reshape(problem.all_parameters, [(model.npar)*(kmax),1]); ...
%         param(:,end)];

    param = zeros(model.npar, kmax+1);
    count = 1;
    for k = progress_id
        if (k >= kmax-fitlength)
            fitrange = k-fitlength:kmax+1;
        else
            fitrange = k:k+fitlength;
        end
        px = polyfit(theta(fitrange),path(1,fitrange),polyorder);
        py = polyfit(theta(fitrange),path(2,fitrange),polyorder);
        pz = polyfit(theta(fitrange),path(3,fitrange),polyorder);
        ptv = polyfit(theta(fitrange),dot_theta(fitrange),polyorder);
%         fitrange = k:k+fitlength;
%         px = polyfit(theta_to_fit(fitrange),path_to_fit(1,fitrange),polyorder);
%         py = polyfit(theta_to_fit(fitrange),path_to_fit(2,fitrange),polyorder);
%         pz = polyfit(theta_to_fit(fitrange),path_to_fit(3,fitrange),polyorder);
%         ptv = polyfit(theta_to_fit(fitrange),dot_theta_to_fit(fitrange),polyorder);
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
        timing_theta = 0;
        param(:,count) = [px'; py'; pz'; dpx'; dpy'; dpz'; ptv'; timing_theta; max(theta)];
%         if k > kmax
%             param(:,count) = zeros(model.npar, 1);
%         end
        count = count + 1;
    end
    
    problem.all_parameters = reshape(param, [(kmax+1)*(model.npar), 1]);

    [solverout,exitflag,info] = FORCESNLPsolver(problem);

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
    
    for pind = 1:kmax+1
        [~,progress_id(pind)] = min(abs(repmat(Xout(13,pind),1,kmax+1) - theta));
%         [~,progress_id(pind)] = min(abs(repmat(Xout(13,pind),1,numel(theta_to_fit)) - theta_to_fit));
    end
    
%     if exitflag == 1 && test >= 8
%         break
%     end
%     test = test + 1;
    
    param_diff = abs(param - old_param);
    progress_diff = abs(progress_id - old_progress_id);
%     solution = mean(progress_diff);
    coor_diff = abs(Xref(1:3,:) - Xout(1:3,:));
    solution = mean(mean(coor_diff));
end

%% plot 1

figure;
plot3(Xref(1,:),Xref(2,:),Xref(3,:),'o');
axis([-1.5 1.5 -1.5 1.5 -0.1 1.1]);
hold on;
plot3(Xout(1,:),Xout(2,:),Xout(3,:));

s = Xout(13,:);
ref_state = ppval(ppp,s);
ref_vel = ppval(pppdot,s); 

figure;
subplot(3,1,1);
plot(s,ref_state(1,:),'--r',s,ref_state(2,:),'--g',s,ref_state(3,:),'--b');
hold on;
plot(s,Xout(1,:),'r',s,Xout(2,:),'g',s,Xout(3,:),'b');
hold off;
title('Position Plot');
legend('xref','yref','zref','x','y','z');

subplot(3,1,2);
plot(s,ref_vel(1,:),'--m',s,ref_vel(2,:),'--c',s,ref_vel(3,:),'--k');
hold on;
plot(s,Xout(5,:),'m',s,Xout(6,:),'c',s,Xout(7,:),'k');
hold off;
title('Velocity Plot');
legend('vxref','vyref','vzref','vx','vy','vz');

subplot(3,1,3);
haha = s(2:end);
plot(haha,Uout(1,1:kmax),'r',haha,Uout(2,1:kmax),'g',haha,Uout(3,1:kmax),'b');
title('Input Plot');
legend('Fx','Fy','Fz');

time = 0:T:T*(kmax);

figure;
subplot(2,1,1);
plot(Xout(13,:),time);
title('Time vs Theta');
subplot(2,1,2);
plot(Xout(13,:),Xout(14,:));
title('Theta Velocity vs Theta');

%% user input

figure;
while (1)
    relative_decision_prompt = 'Do you want to set relative timing to the trajectory? [Y/n]: ';
    relative_decision = input(relative_decision_prompt,'s');
    if (isempty(relative_decision) || relative_decision == 'Y' || relative_decision == 'y')
        theta_prompt = 'Enter the specific theta values: ';
        relative_theta = input(theta_prompt);
        time_prompt = 'Enter the new timings: ';
        relative_time = input(time_prompt);
        dot_theta = zeros(1,kmax+1);
        relative_ind = [];
        for the = relative_theta
            [~, nearest_theta] = min(pdist2(theta',the));
            relative_ind = [relative_ind, nearest_theta];
        end % hack
        sg_incr = 1;
        for db_incr = 1:2:numel(relative_ind)
            dot_theta(relative_ind(db_incr):relative_ind(db_incr+1)) = ...
                (relative_theta(db_incr+1) - relative_theta(db_incr))/relative_time(sg_incr);
            sg_incr = sg_incr + 1;
        end
        clf(gcf);
        subplot(2,1,1);
        plot(theta,time);
        title('New Time vs Theta');
        subplot(2,1,2);
        plot(theta,dot_theta);
        title('New Theta Velocity vs Theta');
    elseif (relative_decision == 'N' || relative_decision == 'n')
        relative_ind = [];
    else
        disp('Please enter a valid answer (y/n).');
    end
    absolute_decision_prompt = 'Do you want to set absolute timing to the trajectory? [Y/n]: ';
    absolute_decision = input(absolute_decision_prompt,'s');
    if (isempty(absolute_decision) || absolute_decision == 'Y' || absolute_decision == 'y')
        absolute_theta_prompt = 'Enter the specific theta values: ';
        absolute_theta = input(absolute_theta_prompt);
        absolute_time_prompt = 'Enter the new timings: ';
        absolute_time = input(absolute_time_prompt);
        absolute_tk = zeros(1,numel(absolute_time));
        for ind = 1:numel(absolute_time)
            absolute_tk(ind) = round(absolute_time(ind)/T);
        end
        break;
    elseif (absolute_decision == 'N' || absolute_decision == 'n')
        absolute_tk = [];
        break;
    else
        disp('Please enter a valid answer (y/n).');
    end
end

%% simulate 2

% X = zeros(nx+nalx,kmax);
% % X(1:3,1) = [1;1;1];
% U = zeros(nu+nalu,kmax);
% solvetime = zeros(1,kmax);
% iters = zeros(1,kmax);
% timeElapsed = zeros(1,kmax);
% problem.x0 = zeros(model.N*model.nvar,1); % stack up problems into one N stages array
% k = 1;
% [~, nrid(k)] = min(pdist2(X(1:3,k)', Xref(1:3,:)'));
% searchlength = 40;
% fitlength = 40;
% figure;
% 
% problem.xinit = X(:,k);
% 
% param = zeros(model.npar, (kmax/gap)+1);
% user_ind = find(dot_theta ~= 0);
% for k = 1:10:kmax+1
%     if (k >= kmax-fitlength)
%         fitrange = k-fitlength:kmax+1;
%     else
%         fitrange = k:k+fitlength;
%     end
%     px = polyfit(theta(fitrange),path(1,fitrange),polyorder);
%     py = polyfit(theta(fitrange),path(2,fitrange),polyorder);
%     pz = polyfit(theta(fitrange),path(3,fitrange),polyorder);
% %     ptv = polyfit(theta(fitrange),dot_theta(fitrange),polyorder);
%     dpx = polyder(px);
%     dpy = polyder(py);
%     dpz = polyder(pz);
%     if (length(dpx) ~= polyorder)
%         if (find(px == 0) == 1)
%             dpx(2) = dpx(1);
%             dpx(1) = 0;
%         else
%             dpx(2) = 0;
%         end
%     end
%     if (length(dpy) ~= polyorder)
%         if (find(py == 0) == 1)
%             dpy(2) = dpy(1);
%             dpy(1) = 0;
%         else
%             dpy(2) = 0;
%         end
%     end
%     if (length(dpz) ~= polyorder)
%         if (find(pz == 0) == 1)
%             dpz(2) = dpz(1);
%             dpz(1) = 0;
%         else
%             dpz(2) = 0;
%         end
%     end 
%     if (k >= min(user_ind) && k <= max(user_ind))
%         ptv = polyfit(theta(user_ind),dot_theta(user_ind),polyorder);
%     else
%         ptv = polyfit(theta(fitrange),zeros(1,length(fitrange)),polyorder);
%     end
%     param(:,((k-1)/10)+1) = [px'; py'; pz'; dpx'; dpy'; dpz'; ptv'];
% end
% 
% 
% problem.all_parameters = repmat(param(:,1:end-1), gap, 1);
% problem.all_parameters = [reshape(problem.all_parameters, [(model.npar)*(kmax),1]); ...
%     param(:,end)];
% 
% [solverout,exitflag,info] = FORCESNLPsolver(problem);
% 
% % mpc output
% Xout = zeros(nx+nalx,model.N);
% Uout = zeros(nu+nalu,model.N-1);
% names = fieldnames(solverout);
% for j = 1:model.N
%     tempout = getfield(solverout, names{j});
%     Xout(:,j) = tempout(nu+nalu+1:nu+nalu+nx+nalx);
%     if (j < model.N)
%         Uout(:,j) = tempout(1:nu+nalu);
%     end
% end

progress_id = ones(1,kmax+1);
old_progress_id = 2*ones(1,kmax+1);
old_param = zeros(model.npar, kmax+1);
param = zeros(model.npar, kmax+1);
epsilon = 1;
solution = 5000;
exitflag = 0;

while (exitflag ~= 1 || solution > epsilon)
    old_param = param;
    old_progress_id = progress_id;
    X = zeros(nx+nalx,kmax);
    U = zeros(nu+nalu,kmax);
    solvetime = zeros(1,kmax);
    iters = zeros(1,kmax);
    timeElapsed = zeros(1,kmax);
    problem.x0 = zeros(model.N*model.nvar,1); % stack up problems into one N stages array
    k = 1;
    [~, nrid(k)] = min(pdist2(X(1:3,k)', Xref(1:3,:)'));
    searchlength = 40;
    fitlength = 40;

    problem.xinit = X(:,k);
%     problem.xfinal = [max(theta); 0];

    param = zeros(model.npar, kmax+1);
    count = 1;
    for k = progress_id
        if (k >= kmax-fitlength)
            fitrange = k-fitlength:kmax+1;
        else
            fitrange = k:k+fitlength;
        end
        px = polyfit(theta(fitrange),path(1,fitrange),polyorder);
        py = polyfit(theta(fitrange),path(2,fitrange),polyorder);
        pz = polyfit(theta(fitrange),path(3,fitrange),polyorder);
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
        for ind = 1:2:numel(relative_ind)
            if (k >= relative_ind(ind) && k <= relative_ind(ind+1))
                ptv = polyfit(theta(relative_ind(ind):relative_ind(ind+1)),dot_theta(relative_ind(ind):relative_ind(ind+1)),polyorder);
                break;
            else
                ptv = polyfit(theta(fitrange),zeros(1,length(fitrange)),polyorder);
            end
        end
        absolute_ind = find(absolute_tk == count);
        if isempty(absolute_ind)
            timing_theta = 0;
        else
            timing_theta = absolute_theta(absolute_ind);
        end
        param(:,count) = [px'; py'; pz'; dpx'; dpy'; dpz'; ptv'; timing_theta; max(theta)];
        count = count + 1;
    end
    
    problem.all_parameters = reshape(param, [(kmax+1)*(model.npar), 1]);

    [solverout,exitflag,info] = FORCESNLPsolver(problem);

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
    
    for pind = 1:kmax+1
        [~,progress_id(pind)] = min(abs(repmat(Xout(13,pind),1,kmax+1) - theta));
    end
    
    param_diff = abs(param - old_param);
    progress_diff = abs(progress_id - old_progress_id);
    solution = mean(progress_diff);
%     coor_diff = abs(Xref(1:3,:) - Xout(1:3,:));
%     solution = mean(mean(coor_diff));
    
end

%% plot 2

figure;
plot3(Xref(1,:),Xref(2,:),Xref(3,:),'o');
axis([-1.5 1.5 -1.5 1.5 -0.1 1.1]);
hold on;
plot3(Xout(1,:),Xout(2,:),Xout(3,:));

s = Xout(13,:);
ref_state = ppval(ppp,s);
ref_vel = ppval(pppdot,s); 

figure;
subplot(3,1,1);
plot(s,ref_state(1,:),'--r',s,ref_state(2,:),'--g',s,ref_state(3,:),'--b');
hold on;
plot(s,Xout(1,:),'r',s,Xout(2,:),'g',s,Xout(3,:),'b');
hold off;
title('Position Plot');
legend('xref','yref','zref','x','y','z');

subplot(3,1,2);
plot(s,ref_vel(1,:),'--m',s,ref_vel(2,:),'--c',s,ref_vel(3,:),'--k');
hold on;
plot(s,Xout(5,:),'m',s,Xout(6,:),'c',s,Xout(7,:),'k');
hold off;
title('Velocity Plot');
legend('vxref','vyref','vzref','vx','vy','vz');

subplot(3,1,3);
haha = s(2:end);
plot(haha,Uout(1,1:kmax),'r',haha,Uout(2,1:kmax),'g',haha,Uout(3,1:kmax),'b');
title('Input Plot');
legend('Fx','Fy','Fz');

time = 0:T:T*(kmax);

figure;
subplot(2,1,1);
plot(Xout(13,:),time);
title('Time vs Theta');
subplot(2,1,2);
plot(Xout(13,:),Xout(14,:));
title('Theta Velocity vs Theta');
