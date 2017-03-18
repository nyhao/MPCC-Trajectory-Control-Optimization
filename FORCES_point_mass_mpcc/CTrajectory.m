classdef CTrajectory<handle
    %TRAJECTORY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        traj_inter
        traj_ppObj
        close_trajectory
        
        endBreak
        dim = 3;
        
        numFittingpoints = 11;
        
        model
    end

    methods
        function obj=CTrajectory(close_trajectory,points)
            % points have to be in the format:
            %points: [x1,y1,z1;
            %         x2,y2,z2;
            %         ........]
            
            if nargin<1
                close_trajectory=1;
            end
            if nargin<2
                points=[0;0;0];
            end
            
            if close_trajectory==0
                obj.traj_ppObj=cscvn(points');
                %obj.traj_ppObj=spline(t,points');
                
                %obj.traj_inter=fnplt(obj.traj_ppObj);
                obj.close_trajectory=close_trajectory;
            else
                %If the first and last point coincide (and there are no other
                %repeated points), then a periodic cubic spline curve is constructed. However, double points result in corners.
                %obj.traj_ppObj=cscvn([points,points(:,1);t,t(1)]);
                  obj.traj_ppObj=cscvn([points,points(:,1)]);
                %obj.traj_ppObj=spline(t,points);
                %obj.traj_ppObj=  csape(t,points);
     
                obj.close_trajectory=close_trajectory;
            end
            
            obj.endBreak = obj.traj_ppObj.breaks(end);
        end

        
        
        
        
        function [x,interpol] = fitpoly(obj,dt,T,givelocalinterpol)
      
            x = [];
           timestep = ((obj.numFittingpoints-1)/2*dt);
           
           A = []; y = [];
            for i=-timestep:dt:timestep
               fitTime = T+ i;
               
               poleval = obj.evaluate(fitTime);
               a_i = [  fitTime^2,fitTime,1,0,0,0,0,0,0;
                        0,0,0,fitTime^2,fitTime,1,0,0,0;
                        0,0,0,0,0,0,fitTime^2,fitTime,1];
               A = [A;a_i];
               y = [y;poleval];
               
               
            end
           obj.model= (A'*A)\A'*y;
           x = obj.model;
           
           if givelocalinterpol
                 xi = []; yi = []; zi = [];
            for i=-1:0.01:1
                Tl = (i+T);
                xi = [xi,obj.model(1)*Tl^2+obj.model(2)*Tl + obj.model(3)];
                yi = [yi,obj.model(4)*Tl^2+obj.model(5)*Tl + obj.model(6)];
                zi = [zi,obj.model(7)*Tl^2+obj.model(8)*Tl + obj.model(9)];
            end
          interpol = [xi;yi;zi];
           end
           
        end
        
        function [interpol] = evaluateinterpol(obj)
      
           
     
           
          
         end
        
        function [pos]=evaluate(obj,T)
            % INPUTS:
            % T: time where the spline has to be evaluated
            % velocityset: setpoint velocity on the spline
            %
            % OUTPUT:
            % inter: interpolated values on the trajectories [pos;pos';pos'']
            % inter: interpolated values on the trajectories [pos;pos';pos'']
            
            if nargin< 1
                T=0;
            end
            if nargin< 2
                velocityset=0;
            end
            coefs=obj.traj_ppObj.coefs; breaks=obj.traj_ppObj.breaks;  pieces=obj.traj_ppObj.pieces;
            
            seg=1;
            findT=0;
            if obj.close_trajectory==0
                % if trajectory is open
                if T<=0
                    T=0;
                end
                if T>=breaks(end)
                    T=breaks(end);
                end
                for i=1:(pieces)
                    a=breaks(i);
                    b=breaks(i+1);
                    if (T>=a && T<=b)
                        seg=i;
                        break
                    end
                    
                end
                findT=T;
            end
            
            if obj.close_trajectory==1
                % if trajectory is closed
                searchTc=mod(T,breaks(end));
                % find the right polynome segment
                for i=1:(pieces)
                    a=breaks(i);
                    b=breaks(i+1);
                    if (searchTc>=a && searchTc<=b)
                        seg=i;
                        break
                    end
                    
                end
                findT=searchTc;
            end
            T=(findT-breaks(seg));
            %% patameter "time" for evaluation
            % polynome coefs for x:
            dim = obj.dim;
            for cntDim = 1:dim
                 kx3 = coefs(dim*seg-(cntDim-1),1); kx2=coefs(dim*seg-(cntDim-1),2); kx1=coefs(dim*seg-(cntDim-1),3); kx0=coefs(dim*seg-(cntDim-1),4);
                 x(dim-(cntDim-1),1)=kx3*T^3+kx2*T^2+kx1*T+kx0;
            end

            
            pos = x;

        end
        
        
    end
    
    
    
    
end
