oc=1;
% define the Quadrotor trajectory with a polygone
%QuadrotorTray = impoly('Closed',oc);

%xy_pos=getPosition(QuadrotorTray);


height = 0.8;
height_curve = 1;


        xyz_pos=[0    0,height;
                -1.5 ,   1.5,height;
                0 ,   2,height;
                 1.5 ,   1.5,height;
                -1.5 ,   -1.5,height;
                0 ,  -2,height;
                1.5 ,  -1.5,height;
                0 ,   0,height;];

       xyz_pos=[0    0,height;
                -1.5 ,   1.5,2;
                0 ,   2,1;
                 1.5 ,   1.5,1;
                -1.5 ,   -1.5,3;
                0 ,  -2,2;
                1.5 ,  -1.5,1;
                0 ,   0,height;];
            
     xyz_pos=[  0   ,   0,  height;
               -1.5 ,  1.5, 2;
                0   ,  2,   1;
                0   ,  2,   2;
                1.5 ,  1.5, 2;
               -1.5 , -1.5, 3;
                0   , -2,   2;
                1.5 , -1.5, 1;
                0   ,  0,   height;];
            

% generate new trajectory
quad_trajectory = CTrajectory(1,xyz_pos');