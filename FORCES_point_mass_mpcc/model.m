function [ x_k ] = model_equationMPCCGimbal( z,p,dt,model,generation )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


global index
if generation
    dt =            p(index.dt_measured);
end

%% states

pos             = z(index.posStates); % position
vel             = z(index.velStates); % x y velocity

w_yaw           = z(index.yawSpeed);
Quadyaw         = z(index.QuadYaw);

rp_del          = z(index.rp_del);
%% inputs

GimbalPitch         = z(index.gimbalPitch);
GimbalYaw           = z(index.gimbalYaw);
GimbalPitchRate     = z(index.gimbalPitchRate);
GimbalYawRate       = z(index.gimbalYawRate);

roll            = z(index.accStates(1));
pitch           = z(index.accStates(2));


u_vel_z         = z(index.velzState); % z velocity

theta           = z(index.thetaState);
theta_vel       = z(index.thetaVelState);
theta_acc       = z(index.thetaAccSate);

%% setpoints

%acc = [(ax);(ay)]*9.81;
alpha  = 0.81;

rp = [roll;pitch];
rp_del_k = rp+alpha*(rp_del - rp);

acc = [tan(rp(1));tan(rp(2))]*9.81;
%% mass point model for x y
modeldrag = 0;
%if modeldrag
C_drag = [0.57;0.57];

pos_k = pos(1:2) + vel*dt + 1/2*dt^2*(acc-C_drag.*vel);
vel_k = vel + dt*(acc-C_drag.*vel);
% else
%     pos_k = pos(1:2) + vel*dt + 1/2*dt^2*acc;
%     vel_k = vel + dt*acc;
%end
%% constant vel model for z
pos_z_k = pos(3) + u_vel_z*dt;
Quadyaw_k = Quadyaw + dt*w_yaw;

theta_k     = theta + dt*theta_vel + 1/2*dt^2*theta_acc;
theta_vel_k = theta_vel + dt*theta_acc;

GimbalPitch_k   = GimbalPitch + dt*GimbalPitchRate;
GimbalYaw_k     = GimbalYaw + dt*GimbalYawRate;



x_k=[pos_k;pos_z_k;vel_k;Quadyaw_k;rp_del_k;theta_k;theta_vel_k;GimbalPitch_k;GimbalYaw_k];


end

