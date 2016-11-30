clear;
clc;

%% system
T = 2*pi; % sampling time
m = 1; % mass
nx = 8; % 4d position (angle around z-axis), velocity and acceleration
nu = 4; % force acting on the acceleration states
g = -9.8; % gravitational acceleration

% continuous state space matrices
Ac = zeros(nx,nx);
Ac(1,5) = 1;
Ac(2,6) = 1;
Ac(3,7) = 1;
Ac(4,8) = 1;

Bc = zeros(nx,nu);
Bc(5,1) = 1/m;
Bc(6,2) = 1/m;
Bc(7,3) = 1/m;
Bc(8,4) = 1/m;

gc = zeros(nx,1);
gc(7,1) = g;

% discretization
A_tilde = [Ac, Bc, I; zeros(nx + nu, nx + nu + nx)]; 
A_tilde_d = expm(A_tilde*T);

Ad = A_tilde_d(1:nx,1:nx);
Bd = A_tilde_d(1:nx,nx+1:nx+nu);
gd = A_tilde_d(1:nx,nx+nu+1:2*nx+nu) * gc;

%% MPC setup
N = 10;
Q = eye(nx);
% Q(5:end,:) = zeros(4,8);
R = eye(nu);
% [~,P] = dlqr(Ad,Bd,Q,R);
P = eye(nx);
% P(5:end,:) = zeros(4,8);
umin = [-4, -4, -10, -4];      umax = [4, 4, 10, 4]; % u limits based on drones max force
xmin = -inf*ones(1,nx);    xmax = inf*ones(1,nx); % no boundaries in space

%% reference trajectory
kmax = 20;
t = 0:2*pi/(kmax/2):4*pi;
x = sin(t); y = cos(t); z = 0.2*t; yaw = zeros(1,kmax+1);
Xref = [x; y; z; yaw; zeros(1,kmax+1); zeros(1,kmax+1); zeros(1,kmax+1); zeros(1,kmax+1)];
    
%% FORCES multistage form
% assume variable ordering zi = [ui; xi] for i=1...N

% dimensions
model.N     = 21;    % horizon length
model.nvar  = nx+nu;    % number of variables
model.neq   = nx;    % number of equality constraints
model.npar  = nx;    % number of real-time parameters

% objective 
for i = 1:model.N-1
%     model.objective{i} = @(z,ref) (z(nu+1:nu+nx)-ref)'*Q*(z(nu+1:nu+nx)-ref);
    model.objective{i} = @(z,ref) z(nu+1:nu+nx)'*Q*z(nu+1:nu+nx) - 2*ref'*z(nu+1:nu+nx);
end
% model.objective{model.N} = @(z,ref) (z(nu+1:nu+nx)-ref)'*P*(z(nu+1:nu+nx)-ref);
model.objective{model.N} = @(z,ref) z(nu+1:nu+nx)'*P*z(nu+1:nu+nx) - 2*ref'*z(nu+1:nu+nx);

% equalities
model.eq = @(z) Ad*z(nu+1:nu+nx) + Bd*z(1:nu) + gd;
              
model.E = [zeros(nx,nu), eye(nx)];

% initial state
model.xinitidx = nu+1:nu+nx;

% inequalities
model.lb = [ umin,    xmin  ];
model.ub = [ umax,    xmax  ];

%% Generate FORCES solver

% get options
codeoptions = getOptions('FORCESNLPsolver');
codeoptions.printlevel = 2;

% generate code
FORCES_NLP(model, codeoptions);

%% simulate
X = zeros(nx,kmax+1);
X(2,1) = 1; % because y axis is cos function (starts with 1)
U = zeros(nu,kmax);
problem.x0 = zeros(model.N*model.nvar,1); % stack up problems into one N stages array
problem.all_parameters = reshape(Xref, [model.N*nx,1]); % stack up parameters (reference) into one N stages array

for k = 1:1
    
    problem.xinit = X(:,k);
    
    [solverout,exitflag,info] = FORCESNLPsolver(problem);

    if( exitflag == 1 )
        U(:,k) = solverout.x01(1:nu);
        solvetime(k) = info.solvetime;
        iters(k) = info.it;
    else
        error('Some problem in solver');
    end
    
    X(:,k+1) = model.eq( [U(:,k); X(:,k)] )';
end

%% results
Xout = zeros(nx,kmax+1);
Uout = zeros(nu,kmax);
names = fieldnames(solverout);
for j = 1:model.N
    tempout = getfield(solverout, names{j});
    Xout(:,j) = tempout(nu+1:nu+nx);
    if (j < model.N)
        Uout(:,j) = tempout(1:nu);
    end
end

error = abs(Xout - Xref);

%% plot
figure; clf;
% quiver3(Xref(1,:),Xref(2,:),Xref(3,:),Xref(5,:),Xref(6,:),Xref(7,:));
plot3(Xref(1,:),Xref(2,:),Xref(3,:));
% axis([-10 10 -10 10 -5 5])
hold on;
% quiver3(Xout(1,:),Xout(2,:),Xout(3,:),Xout(5,:),Xout(6,:),Xout(7,:));
plot3(Xout(1,:),Xout(2,:),Xout(3,:));
hold off;

figure;
plot(t,Xref(1,:),'--r',t,Xref(2,:),'--g',t,Xref(3,:),'--b');
hold on;
plot(t,Xout(1,:),'r',t,Xout(2,:),'g',t,Xout(3,:)),'b';
hold off;

figure;
plot(t,Xref(5,:),'--m',t,Xref(6,:),'--c',t,Xref(7,:),'--k');
hold on;
plot(t,Xout(5,:),'m',t,Xout(6,:),'c',t,Xout(7,:),'k');
hold off;