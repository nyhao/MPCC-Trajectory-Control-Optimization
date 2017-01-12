clear;
clc;

%% system
T = 0.05; % sampling time for 20Hz
m = 1; % mass
nx = 20; % 4d position (angle around z-axis) and up to 4th derivatives
nalx = 5; % arc length and up to 4th derivatives
nu = 4; % force acting on the acceleration states
nalu = 1; % arc acceleration (virtual input)
g = -9.8; % gravitational acceleration

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

% Ac(9,10) = 1;
Ac(21,22) = 1;

Bc = zeros(nx+nalx,nu+nalu);
Bc(5,1) = 1/m;
Bc(6,2) = 1/m;
Bc(7,3) = 1/m;
Bc(8,4) = 1/m;

% Bc(10,5) = 1;
Bc(22,5) = 1;

gc = zeros(nx+nalx,1);
gc(7,1) = g;

I = eye(nx+nalx);

% discretization
A_tilde = [Ac, Bc, I; zeros(nx + nalx + nu + nalu, nx + nalx + nu + nalu + nx + nalx)]; 
A_tilde_d = expm(A_tilde*T);

Ad = A_tilde_d(1:nx+nalx,1:nx+nalx);
Bd = A_tilde_d(1:nx+nalx,nx+nalx+1:nx+nalx+nu+nalu);
gd = A_tilde_d(1:nx+nalx,nx+nalx+nu+nalu+1:2*(nx+nalx)+nu+nalu) * gc;

% finite differentiation
% a(i+1) = v(i+1) - v(i)
Ad(9,9) = 0;
Ad(10,10) = 0;
Ad(11,11) = 0;
Ad(12,12) = 0;
Ad(9,5) = -1/T;
Ad(10,6) = -1/T;
Ad(11,7) = -1/T;
Ad(12,8) = -1/T;

% j(i+1) = a(i+1) - a(i)
Ad(13,13) = 0;
Ad(14,14) = 0;
Ad(15,15) = 0;
Ad(16,16) = 0;
Ad(13,9) = -1/T;
Ad(14,10) = -1/T;
Ad(15,11) = -1/T;
Ad(16,12) = -1/T;

% s(i+1) = j(i+1) - j(i)
Ad(17,17) = 0;
Ad(18,18) = 0;
Ad(19,19) = 0;
Ad(20,20) = 0;
Ad(17,13) = -1/T;
Ad(18,14) = -1/T;
Ad(19,15) = -1/T;
Ad(20,16) = -1/T;

% theta''(i+1) = theta'(i+1) - theta'(i)
Ad(23,23) = 0;
Ad(23,22) = -1/T;

% theta'''(i+1) = theta''(i+1) - theta''(i)
Ad(24,24) = 0;
Ad(24,23) = -1/T;

% theta''(i+1) = theta'(i+1) - theta'(i)
Ad(25,25) = 0;
Ad(25,24) = -1/T;

%% MPCC setup
% for constant Q and P matrices
contourpenalty = 10000; 
lagpenalty = 10000; 
zpenalty = 10000; 
dthetapenalty = 100;
fd_factor = 0.000001;
Qp = [contourpenalty, 0, 0; 0, lagpenalty, 0; 0, 0, zpenalty];
Qfd = fd_factor*eye(nx-12);
Qdtheta = dthetapenalty; 
Qfdtheta = fd_factor*eye(nalx-3);
Pp = [contourpenalty, 0, 0; 0, lagpenalty, 0; 0, 0, zpenalty];
Pfd = fd_factor*eye(nx-12);
Pdtheta = dthetapenalty; 
Pfdtheta = fd_factor*eye(nalx-3);
% boundaries
umin = [-4, -4, -10, -4, 0];    umax = [4, 4, 10, 4, 10]; % u limits based on drones max force
xmin = -inf*ones(1,nx+nalx);    xmax = inf*ones(1,nx+nalx); % no boundaries in space

%% reference trajectory
kmax = 160; % 8s trajectory for 20Hz
t = 0:T:T*kmax;
theta = 0:0.05:0.05*kmax;
dot_theta = zeros(1,kmax+1);
x = cos(theta);
y = sin(theta);
z = zeros(1,kmax+1);
yaw = atan2(cos(theta),-sin(theta));
phi = @(pg) atan2(cos(pg),-sin(pg));
fd = repmat(zeros(1,kmax+1), nx-4, 1);
Xref = [x; y; z; yaw; fd; theta; dot_theta];

%% FORCES multistage form
% assume variable ordering zi = [ui; xi] for i=1...N

% dimensions
model.N     = 11;    % horizon length
model.nvar  = nx+nalx+nu+nalu;    % number of variables
model.neq   = nx+nalx;    % number of equality constraints

% objective 
for i = 1:model.N-1
    model.objective{i} = @(z) [sin(phi(z(nu+nalu+nx+1)))*(z(nu+nalu+1)-cos(z(nu+nalu+nx+1))) - ...
        cos(phi(z(nu+nalu+nx+1)))*(z(nu+nalu+2)-sin(z(nu+nalu+nx+1))); ...
        -cos(phi(z(nu+nalu+nx+1)))*(z(nu+nalu+1)-cos(z(nu+nalu+nx+1))) - ...
        sin(phi(z(nu+nalu+nx+1)))*(z(nu+nalu+2)-sin(z(nu+nalu+nx+1))); ...
        z(nu+nalu+3)]'*Qp* ...
        [sin(phi(z(nu+nalu+nx+1)))*(z(nu+nalu+1)-cos(z(nu+nalu+nx+1))) - ...
        cos(phi(z(nu+nalu+nx+1)))*(z(nu+nalu+2)-sin(z(nu+nalu+nx+1))); ...
        -cos(phi(z(nu+nalu+nx+1)))*(z(nu+nalu+1)-cos(z(nu+nalu+nx+1))) - ...
        sin(phi(z(nu+nalu+nx+1)))*(z(nu+nalu+2)-sin(z(nu+nalu+nx+1))); ...
        z(nu+nalu+3)] - Qdtheta*(z(nu+nalu+nx+2)) + ... 
        (z(nu+nalu+13:nu+nalu+nx))'*Qfd*(z(nu+nalu+13:nu+nalu+nx)) + ...
        (z(nu+nalu+nx+4:nu+nalu+nx+nalx))'*Qfdtheta*(z(nu+nalu+nx+4:nu+nalu+nx+nalx));
end
model.objective{model.N} = @(z) [sin(phi(z(nu+nalu+nx+1)))*(z(nu+nalu+1)-cos(z(nu+nalu+nx+1))) - ...
    cos(phi(z(nu+nalu+nx+1)))*(z(nu+nalu+2)-sin(z(nu+nalu+nx+1))); ...
    -cos(phi(z(nu+nalu+nx+1)))*(z(nu+nalu+1)-cos(z(nu+nalu+nx+1))) - ...
    sin(phi(z(nu+nalu+nx+1)))*(z(nu+nalu+2)-sin(z(nu+nalu+nx+1))); ...
    z(nu+nalu+3)]'*Pp* ...
    [sin(phi(z(nu+nalu+nx+1)))*(z(nu+nalu+1)-cos(z(nu+nalu+nx+1))) - ...
    cos(phi(z(nu+nalu+nx+1)))*(z(nu+nalu+2)-sin(z(nu+nalu+nx+1))); ...
    -cos(phi(z(nu+nalu+nx+1)))*(z(nu+nalu+1)-cos(z(nu+nalu+nx+1))) - ...
    sin(phi(z(nu+nalu+nx+1)))*(z(nu+nalu+2)-sin(z(nu+nalu+nx+1))); ...
    z(nu+nalu+3)] - Pdtheta*(z(nu+nalu+nx+2)) + ...
    (z(nu+nalu+13:nu+nalu+nx))'*Pfd*(z(nu+nalu+13:nu+nalu+nx)) + ...
    (z(nu+nalu+nx+4:nu+nalu+nx+nalx))'*Pfdtheta*(z(nu+nalu+nx+4:nu+nalu+nx+nalx));
    
% equalities
model.eq = @(z) Ad*z(nu+nalu+1:nu+nalu+nx+nalx) + Bd*z(1:nu+nalu) + gd;

% model.E = [zeros(nx+nalx,nu+nalu), eye(nx+nalx)];
model.E = [zeros(nx+nalx,nu+nalu), ...
    eye(nx+nalx) + diag([zeros(4,1); (-1/T)*ones(12,1); zeros(nalx,1)], -4) + ...
    diag([zeros(nx+1,1); (-1/T)*ones(3,1)], -1)];

% initial state
model.xinitidx = nu+nalu+1:nu+nalu+nx+nalx;

% inequalities
model.lb = [ umin,    xmin  ];
model.ub = [ umax,    xmax  ];

%% Generate FORCES solver

% get options
codeoptions = getOptions('FORCESNLPsolver');
codeoptions.printlevel = 2;
codeoptions.maxit = 1000;

% generate code
FORCES_NLP(model, codeoptions);

%% simulate
X = zeros(nx+nalx,1);
X(1:3,1) = [1; 0; 0];
U = zeros(nu+nalu,1);
problem.x0 = zeros(model.N*model.nvar,1); % stack up problems into one N stages array
k = 1;
[~, nrid(k)] = min(pdist2(X(1:3,k)', Xref(1:3,:)'));
searchrange = 40;
figure;

while (nrid(k) ~= kmax+1)
    
    problem.xinit = X(:,k);
    
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
    axis([-1.5 1.5 -1.5 1.5 -0.1 0.1]);
    hold on;
    plot3(Xref(1,:),Xref(2,:),Xref(3,:));
    plot3(X(1,k),X(2,k),X(3,k),'or');
    plot3(Xout(1,:),Xout(2,:),Xout(3,:),'.m');
    drawnow;
    hold off;
    
    % search nearest reference point for the next time step
%     [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,:)')); % search through the whole trajectory
    if (nrid(k) >= kmax-searchrange)
        [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,nrid(k):kmax+1)'));
    else
        [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,nrid(k):nrid(k)+searchrange)'));
    end % search only in a smaller search range around the previous nearest reference point
    nrid(k+1) = nrid(k+1) + nrid(k) - 1; % non-reversal path
    k = k + 1;
end

%% plot
t_old = 0:size(Xref,2)-1;
t_new = 0:size(X,2)-1;

figure;
plot(t_old,Xref(1,:),'--r',t_old,Xref(2,:),'--g',t_old,Xref(3,:),'--b');
hold on;
% plot(t,Xout(1,:),'r',t,Xout(2,:),'g',t,Xout(3,:),'b');
plot(t_new,X(1,:),'r',t_new,X(2,:),'g',t_new,X(3,:),'b');
hold off;

figure;
plot(t_old,Xref(5,:),'--m',t_old,Xref(6,:),'--c',t_old,Xref(7,:),'--k');
hold on;
% plot(t,Xout(5,:),'m',t,Xout(6,:),'c',t,Xout(7,:),'k');
plot(t_new,X(5,:),'m',t_new,X(6,:),'c',t_new,X(7,:),'k');
hold off;
