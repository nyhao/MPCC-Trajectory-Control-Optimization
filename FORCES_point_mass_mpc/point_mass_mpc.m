clear;
clc;

%% system
T = 0.05; % sampling time for 20Hz
m = 1; % mass
nx = 20; % 4d position (angle around z-axis) and up to 4th derivatives
nu = 4; % force acting on the acceleration states
g = -9.8; % gravitational acceleration

% continuous state space matrices
% dot.X = Ac*X + Bc*U
% dot.X = [vx; vy; vz; wyaw; ax; ay; az; ayaw]
% X = [x; y; z; yaw; vx; vy; vz; wyaw]
% U = [Fx; Fy; Fz; Tyaw]
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

I = eye(nx);

% discretization
A_tilde = [Ac, Bc, I; zeros(nx + nu, nx + nu + nx)]; 
A_tilde_d = expm(A_tilde*T);

Ad = A_tilde_d(1:nx,1:nx);
Bd = A_tilde_d(1:nx,nx+1:nx+nu);
gd = A_tilde_d(1:nx,nx+nu+1:2*nx+nu) * gc;

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

%% MPC setup
% for constant Q and P matrices
Q = 0.0001*eye(nx);
Q(1:4,1:4) = 100000*eye(4);
P = 0.0001*eye(nx);
P(1:4,1:4) = 100000*eye(4);
O = eye(nx);
O(1:4,1:4) = zeros(4);
% for real-time varying Q and P matrices
Qvec = ones(nx,1);
Qvec(1:4) = 1000000*ones(4,1);
Ovec = ones(nx,1);
Ovec(1:4) = zeros(4,1);
Qmat = [repmat(Ovec, 1, 10), Qvec, repmat(Ovec, 1, 9), Qvec, repmat(Ovec, 1, 9), Qvec, repmat(Ovec, 1, 9), Qvec];
% boundaries
umin = [-4, -4, -10, -4];    umax = [4, 4, 10, 4]; % u limits based on drones max force
xmin = -inf*ones(1,nx);    xmax = inf*ones(1,nx); % no boundaries in space

%% reference trajectory
kmax = 160; % 8s trajectory for 20Hz
t = 0:T:T*kmax;
nsp = 4; % number of important sample points
t_sample = 0:T*(kmax/nsp):T*kmax; % time for important sample points
sx = [0, 1, 1, 0, 0]; % important sample x-coordinate
sy = [0, 0, 1, 1, 0]; % important sample y-coordinate
sz = [0, 0.2, 0.4, 0.6, 0.8]; % important sample z-coordinate
syaw = [0, 0, 0, 0, 0]; % important sample yaw angle
x = interp1(t_sample, sx, t, 'linear');
y = interp1(t_sample, sy, t, 'linear');
z = interp1(t_sample, sz, t, 'linear');
yaw = interp1(t_sample, syaw, t);
Xref = [x; y; z; yaw; repmat(zeros(1,kmax+1), nx-4, 1)];
    
%% FORCES multistage form
% assume variable ordering zi = [ui; xi] for i=1...N

% dimensions
model.N     = 11;    % horizon length
model.nvar  = nx+nu;    % number of variables
model.neq   = nx;    % number of equality constraints
model.npar  = nx;    % number of real-time parameters

% objective 
for i = 1:model.N-1
    model.objective{i} = @(z,ref) (z(nu+1:nu+nx)-ref)'*Q*(z(nu+1:nu+nx)-ref);
%     model.objective{i} = @(z,ref) z(nu+1:nu+nx)'*Q*z(nu+1:nu+nx) - 2*ref'*z(nu+1:nu+nx);
%     model.objective{i} = @(z,par) (z(nu+1:nu+nx)-par(1:nx))'*diag(par(nx+1:2*nx))*(z(nu+1:nu+nx)-par(1:nx));
end
model.objective{model.N} = @(z,ref) (z(nu+1:nu+nx)-ref)'*P*(z(nu+1:nu+nx)-ref);
% model.objective{model.N} = @(z,ref) z(nu+1:nu+nx)'*P*z(nu+1:nu+nx) - 2*ref'*z(nu+1:nu+nx);
% model.objective{model.N} = @(z,par) (z(nu+1:nu+nx)-par(1:nx))'*diag(par(nx+1:2*nx))*(z(nu+1:nu+nx)-par(1:nx));

% equalities
model.eq = @(z) Ad*z(nu+1:nu+nx) + Bd*z(1:nu) + gd;

% model.E = [zeros(nx,nu), eye(nx)];
model.E = [zeros(nx,nu), eye(nx) + diag([zeros(4,1); (-1/T)*ones(12,1)], -4)];

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
% X = zeros(nx,kmax+1);
% U = zeros(nu,kmax);
% problem.x0 = zeros(model.N*model.nvar,1); % stack up problems into one N stages array
% % problem.all_parameters = reshape(Xref, [model.N*nx,1]); % stack up parameters into one N stages array
% % params = [Xref; Qmat];
% % problem.all_parameters = reshape(params, [model.N*2*nx, 1]);
% 
% for k = 1:kmax
%     
%     problem.xinit = X(:,k);
%     
%     if (k+model.N <= kmax+2)
%         problem.all_parameters = reshape(Xref(:,k:k-1+model.N), [model.N*nx,1]);
%     else
%         problem.all_parameters = [reshape(Xref(:,k:kmax+1), [(kmax+2-k)*nx,1]); ...
%             repmat(Xref(:,kmax+1), k+model.N-(kmax+2), 1)];
%     end
%         
%     [solverout,exitflag,info] = FORCESNLPsolver(problem);
% 
%     if( exitflag == 1 )
%         U(:,k) = solverout.x01(1:nu);
%         solvetime(k) = info.solvetime;
%         iters(k) = info.it;
%     else
%         error('Some problem in solver');
%     end
%     
%     X(:,k+1) = model.eq( [U(:,k); X(:,k)] )';
% end

X = zeros(nx,1);
% X(1:3,1) = [0.25; -0.18; 0.037]; 
U = zeros(nu,1);
problem.x0 = zeros(model.N*model.nvar,1); % stack up problems into one N stages array
k = 1;
[~, nrid(k)] = min(pdist2(X(1:3,k)', Xref(1:3,:)'));
searchrange = 20;

while (nrid(k) ~= kmax+1)
    
    problem.xinit = X(:,k);
    
    if (nrid(k)+model.N <= kmax+2)
        problem.all_parameters = reshape(Xref(:,nrid(k):nrid(k)-1+model.N), [model.N*nx,1]);
    else
        problem.all_parameters = [reshape(Xref(:,nrid(k):kmax+1), [(kmax+2-nrid(k))*nx,1]); ...
            repmat(Xref(:,kmax+1), nrid(k)+model.N-(kmax+2), 1)];
    end
    
    [solverout,exitflag,info] = FORCESNLPsolver(problem);

    if( exitflag == 1 )
        U(:,k) = solverout.x01(1:nu);
        solvetime(k) = info.solvetime;
        iters(k) = info.it;
    else
        error('Some problem in solver');
    end
    
    X(:,k+1) = model.eq( [U(:,k); X(:,k)] )';
%     [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,:)')); % search through the whole trajectory
    if (nrid(k) <= searchrange) 
        [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,1:2*searchrange)'));
    elseif (nrid(k) >= kmax-searchrange)
        [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,kmax+1-2*searchrange:kmax+1)'));
        nrid(k+1) = nrid(k+1) + kmax + 1 - 2*searchrange - 1;
    else
        [~, nrid(k+1)] = min(pdist2(X(1:3,k+1)', Xref(1:3,nrid(k)-searchrange:nrid(k)+searchrange)'));
        nrid(k+1) = nrid(k+1) + nrid(k) - searchrange - 1;
    end % search only in a smaller search range around the previous nearest reference point
    k = k + 1;
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
plot3(Xref(1,:),Xref(2,:),Xref(3,:),'o');
axis([-0.5 1.5 -0.5 1.2 -0.1 1]);
hold on;
% quiver3(Xout(1,:),Xout(2,:),Xout(3,:),Xout(5,:),Xout(6,:),Xout(7,:));
% plot3(Xout(1,:),Xout(2,:),Xout(3,:));
quiver3(X(1,:),X(2,:),X(3,:),X(5,:),X(6,:),X(7,:));
% plot3(X(1,:),X(2,:),X(3,:),'+');
hold off;

t_new = 0:size(X,2)-1;

figure;
% plot(t,Xref(1,:),'--r',t,Xref(2,:),'--g',t,Xref(3,:),'--b');
hold on;
% plot(t,Xout(1,:),'r',t,Xout(2,:),'g',t,Xout(3,:),'b');
plot(t_new,X(1,:),'r',t_new,X(2,:),'g',t_new,X(3,:),'b');
hold off;

figure;
% plot(t,Xref(5,:),'--m',t,Xref(6,:),'--c',t,Xref(7,:),'--k');
hold on;
% plot(t,Xout(5,:),'m',t,Xout(6,:),'c',t,Xout(7,:),'k');
plot(t_new,X(5,:),'m',t_new,X(6,:),'c',t_new,X(7,:),'k');
hold off;
