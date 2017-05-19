classdef OneShotMPCC < handle
    properties
        %% Trajectory properties
        T;  % discretization time-step
        keyframes;  % keyframes is a Nx4 matrix with the 3D position and yaw of the quadrotor
        keyorientations;  % keyorientations is a Nx2 matrix with the yaw and pitch angle of the camera
        %% Weights
        contour_weight;
        lag_weight;
        orientation_weight;
        progress_weight;
        jerk_weight;
        camera_weight;
        relative_timing_weight;
        absolute_timing_weight;
        rest_weight;
        %% Parameters
        mass;
        gravity;
        umin;
        umax;
        xmin;
        xmax;
        %% Properties for optimization
        nStates = 19;
        nInputs = 7;
        nVars;  % number of variables
        nPars;  % number of runtime parameters
        nStages = 160;  % horizon length
        % Input indices
        inputIndices;
        % Position and orientation state indices
        posStateIndices;
        yawStateIndex;
        velStateIndices;
        yawVelStateIndex;
        jerkStateIndices;
        yawJerkStateIndex;
        % Theta state indices
        thetaStateIndex;
        thetaVelStateIndex;
        thetaJerkStateIndex;
        % Camera state indices
        gimbalYawStateIndex;
        gimbalPitchStateIndex;
        gimbalYawVelStateIndex;
        gimbalPitchVelStateIndex;
        % Combined state indices
        allPosStateIndices;
        allVelStateIndices;
        allJerkStateIndices;
        allThetaStateIndices;
        allCameraStateIndices;
        % Order for local polynomial fitting
        poly_order = 2;  % quadratic fit
        %% Trajectory information
        pos_spline;
        vel_spline;
        positions;
        velocities;
        cy_spline;
        cp_spline;
        camera_yaw;
        camera_pitch;
        theta;
        %% Solver information
        model;
        %% User add-on request information
        relative_ind; %  to store indices which are meant to have relative timing constraint
        absolute_tk; %  to store time-steps which are meant to have absolute timing constraint
        %% Iteration breaking condition
        raw_condition;
        tuned_condition;
    end
    methods
        function obj = OneShotMPCC(T, keyframes, keyorientations, options)
            if nargin < 4
                options = struct();
            end
            
            %% Initialize state vector indices
            obj.inputIndices = 1:obj.nInputs;
            % Position and orientation state indices
            obj.posStateIndices = obj.nInputs + (1:3);
            obj.yawStateIndex = obj.posStateIndices(3) + 1;
            obj.velStateIndices = obj.posStateIndices + 4;
            obj.yawVelStateIndex = obj.velStateIndices(3) + 1;
            obj.jerkStateIndices = obj.velStateIndices + 4;
            obj.yawJerkStateIndex = obj.jerkStateIndices(3) + 1;
            % Theta state indices
            obj.thetaStateIndex = obj.yawJerkStateIndex + 1;
            obj.thetaVelStateIndex = obj.thetaStateIndex + 1;
            obj.thetaJerkStateIndex = obj.thetaVelStateIndex + 1;
            % Camera state indices
            obj.gimbalYawStateIndex = obj.thetaJerkStateIndex + 1;
            obj.gimbalPitchStateIndex = obj.gimbalYawStateIndex + 1;
            obj.gimbalYawVelStateIndex = obj.gimbalYawStateIndex + 2;
            obj.gimbalPitchVelStateIndex = obj.gimbalYawStateIndex + 3;
            
            obj.T = T;
            obj.keyframes = keyframes;
            obj.keyorientations = keyorientations;
            
            obj.nVars = obj.nStates + obj.nInputs;
            obj.nPars = (6*(obj.poly_order+1)+3*(obj.poly_order))+2;  % not fixed?
            
            obj.allPosStateIndices = [obj.posStateIndices, obj.yawStateIndex];
            obj.allVelStateIndices = [obj.velStateIndices, obj.yawVelStateIndex];
            obj.allJerkStateIndices = [obj.jerkStateIndices, obj.yawJerkStateIndex];
            obj.allThetaStateIndices = [obj.thetaStateIndex, obj.thetaVelStateIndex, obj.thetaJerkStateIndex];
            obj.allCameraStateIndices = [obj.gimbalYawStateIndex, ...
                obj.gimbalPitchStateIndex, ...
                obj.gimbalYawVelStateIndex, ...
                obj.gimbalPitchVelStateIndex];
            
            obj.relative_ind = [];
            obj.absolute_tk = [];
            
            %% Interpolate keyframes
            interpolate_keyframes(obj);
            
            %% Initialize options
            parse_options(obj, options);
        end
        
        function parse_options(obj, options)
            %% Initialize (default) parameters
            if ~isfield(options, 'contour_weight')
                options.contour_weight = 10000;
            end
            if ~isfield(options, 'lag_weight')
                options.lag_weight = 10000;
            end
            if ~isfield(options, 'orientation_weight')
                options.orientation_weight = 10000;
            end
            if ~isfield(options, 'progress_weight')
                options.progress_weight = 2;
            end
            if ~isfield(options, 'jerk_weight')
                options.jerk_weight = 2;
            end
            if ~isfield(options, 'camera_weight')
                options.camera_weight = 1;
            end
            if ~isfield(options, 'relative_timing_weight')
                options.relative_timing_weight = 10000;
            end
            if ~isfield(options, 'absolute_timing_weight')
                options.absolute_timing_weight = 10000000;
            end
            if ~isfield(options, 'rest_weight')
                options.rest_weight = 100;
            end
            if ~isfield(options, 'mass')
                options.mass = 0.5;
            end
            if ~isfield(options, 'gravity')
                options.gravity = -9.81;
            end
            if ~isfield(options, 'umax')
                options.umax = [+4;+4;+8;+4;+2;+2;+2];
            end
            if ~isfield(options, 'umin')
                options.umin = -options.umax;
                options.umin(3) = 0;  % only throttle upwards, not downwards
            end
            if ~isfield(options, 'xmax')
                options.xmax = +100 * ones(obj.nVars - obj.nInputs, 1);
            end
            if ~isfield(options, 'xmin')
                options.xmin = -options.xmax;
                options.xmin(14) = 0;  % only allows forward progress
            end
            if ~isfield(options, 'raw_condition')
                options.raw_condition = 3;
            end
            if ~isfield(options, 'tuned_condition')
                options.tuned_condition = 5;
            end
            
            %% Extract options
            obj.contour_weight = options.contour_weight;
            obj.lag_weight = options.lag_weight;
            obj.orientation_weight = options.orientation_weight;
            obj.progress_weight = options.progress_weight;
            obj.jerk_weight = options.jerk_weight;
            obj.camera_weight = options.camera_weight;
            obj.relative_timing_weight = options.relative_timing_weight;
            obj.absolute_timing_weight = options.absolute_timing_weight;
            obj.rest_weight = options.rest_weight;
            obj.mass = options.mass;
            obj.gravity = options.gravity;
            obj.umax = options.umax;
            obj.umin = options.umin;
            obj.xmax = options.xmax;
            obj.xmin = options.xmin;
            obj.raw_condition = options.raw_condition;
            obj.tuned_condition = options.tuned_condition;
        end
        
        function interpolate_keyframes(obj)
            % Get theta from keyframes (Calculate chord lengths between keyframes)
            keyframes_to_theta = cumsum([0;((diff(obj.keyframes(:,1:3)).^2)*ones(3,1)).^(1/4)]).';
            obj.theta = linspace(keyframes_to_theta(1), keyframes_to_theta(end), obj.nStages+1);
            
            % Construct spline functions for position and velocity which
            % are parameterized by theta
            obj.pos_spline = cscvn(obj.keyframes(:,1:3)');
            obj.vel_spline = fnder(obj.pos_spline);
            
            % Compute reference positions, orientations and velocities
            % with spline functions
            obj.positions = ppval(obj.pos_spline,obj.theta);
            obj.velocities = ppval(obj.vel_spline,obj.theta);
            
            % Construct spline functions for camera's yaw and pitch which
            % are parameterized by theta
            obj.cy_spline = spline(keyframes_to_theta,obj.keyorientations(:,1)');
            obj.cp_spline = spline(keyframes_to_theta,obj.keyorientations(:,2)');
            
            % Compute reference camera's yaw and pitch with spline
            % functions
            obj.camera_yaw = ppval(obj.cy_spline,obj.theta);
            obj.camera_pitch = ppval(obj.cp_spline,obj.theta);
        end
        
        function [Ad, Bd, gd] = setup_system(obj)
            %% Build quadrotor model
            [Ad, Bd, gd] = build_quad_model(obj, obj.T, obj.nStates, obj.nInputs);
            
            %% Generate FORCES solver
            generate_FORCES_solver(obj, Ad, Bd, gd);
        end
        
        function [raw_output, tuned_output] = generate_trajectory(obj)
            %% Solve for the first time
            [raw_trajectory, raw_input, raw_exitflag, raw_info] = solve_system(obj);
            
            raw_output.trajectory = raw_trajectory;
            raw_output.input = raw_input;
            raw_output.exitflag = raw_exitflag;
            raw_output.info = raw_info;
            
            %% Prompt user for tuning
            [relative_theta_vel, absolute_theta] = user_prompt(obj);
            
            %% Solve for the second time
            if isempty(obj.relative_ind) && isempty(obj.absolute_tk)
                tuned_output = struct();
            else
                [tuned_trajectory, tuned_input, tuned_exitflag, tuned_info] = ...
                    solve_system(obj, relative_theta_vel, absolute_theta);

                tuned_output.trajectory = tuned_trajectory;
                tuned_output.input = tuned_input;
                tuned_output.exitflag = tuned_exitflag;
                tuned_output.info = tuned_info;
            end
        end
        
        function [Xout, Uout, exitflag, info] = solve_system(obj, relative_theta_vel, absolute_theta)
            if nargin < 2
                relative_theta_vel = zeros(1,obj.nStages+1);
                absolute_theta = 0;
                epsilon = obj.raw_condition;
            else
                epsilon = obj.tuned_condition;
            end
            
            %% Initialize information for solver
            progress_id = ones(1,obj.nStages+1);
            solution = 5000;
            exitflag = 0;
            
            %% Solve until we get a valid trajectory
            while (exitflag ~= 1 || solution > epsilon)
                % Setup the problem
                old_progress_id = progress_id;
                Xout = zeros(obj.nStates,obj.nStages+1);
                Xout(:,1) = [obj.positions(:,1); 0; zeros(4,1); zeros(4,1); zeros(3,1); ...
                    obj.camera_yaw(1); obj.camera_pitch(1); zeros(2,1)];  % initial conditions
                Uout = zeros(obj.nInputs,obj.nStages);
                problem.x0 = zeros((obj.nStages+1)*obj.nVars,1);  % stack up problems into one N stages array
                k = 1;
                fitlength = 40;
                
                problem.xinit = Xout(:,k);
                
                % Compute polynomial coefficients and parameters for every time-step
                param = zeros(obj.nPars, obj.nStages+1);
                count = 1;
                for k = progress_id
                    if (k >= obj.nStages-fitlength)
                        fitrange = k-fitlength:obj.nStages+1;
                    else
                        fitrange = k:k+fitlength;
                    end
                    px = polyfit(obj.theta(fitrange),obj.positions(1,fitrange),obj.poly_order);
                    py = polyfit(obj.theta(fitrange),obj.positions(2,fitrange),obj.poly_order);
                    pz = polyfit(obj.theta(fitrange),obj.positions(3,fitrange),obj.poly_order);
                    dpx = polyder(px);
                    dpy = polyder(py);
                    dpz = polyder(pz);
                    if (length(dpx) ~= obj.poly_order)
                        if (find(px == 0) == 1)
                            dpx(2) = dpx(1);
                            dpx(1) = 0;
                        else
                            dpx(2) = 0;
                        end
                    end
                    if (length(dpy) ~= obj.poly_order)
                        if (find(py == 0) == 1)
                            dpy(2) = dpy(1);
                            dpy(1) = 0;
                        else
                            dpy(2) = 0;
                        end
                    end
                    if (length(dpz) ~= obj.poly_order)
                        if (find(pz == 0) == 1)
                            dpz(2) = dpz(1);
                            dpz(1) = 0;
                        else
                            dpz(2) = 0;
                        end
                    end
                    pcy = polyfit(obj.theta(fitrange),obj.camera_yaw(fitrange),obj.poly_order);
                    pcp = polyfit(obj.theta(fitrange),obj.camera_pitch(fitrange),obj.poly_order);
                    if isempty(obj.relative_ind)
                        ptv = polyfit(obj.theta(fitrange),zeros(1,length(fitrange)),obj.poly_order);
                    else
                        for ind = 1:2:numel(obj.relative_ind)
                            if (k >= obj.relative_ind(ind) && k <= obj.relative_ind(ind+1))
                                ptv = polyfit(obj.theta(obj.relative_ind(ind):obj.relative_ind(ind+1)), ...
                                    relative_theta_vel(obj.relative_ind(ind):obj.relative_ind(ind+1)),obj.poly_order);
                                break;
                            else
                                ptv = polyfit(obj.theta(fitrange),zeros(1,length(fitrange)),obj.poly_order);
                            end
                        end
                    end
                    absolute_ind = find(obj.absolute_tk == count);
                    if isempty(absolute_ind)
                        timing_theta = 0;
                    else
                        timing_theta = absolute_theta(absolute_ind);
                    end
                    param(:,count) = [px'; py'; pz'; dpx'; dpy'; dpz'; pcy'; pcp'; ptv'; timing_theta; max(obj.theta)];
                    count = count + 1;
                end
                
                problem.all_parameters = reshape(param, [(obj.nStages+1)*(obj.nPars), 1]);  % stack up parameters
                
                % Solve the problem
                [solverout,exitflag,info] = FORCESNLPsolver(problem);
                disp(['exitflag = ', num2str(exitflag)]);
                
                % One-shot MPCC output
                names = fieldnames(solverout);
                for j = 1:obj.nStages+1
                    tempout = getfield(solverout, names{j});
                    Xout(:,j) = tempout(obj.nInputs+1:obj.nInputs+obj.nStates);
                    if (j <= obj.nStages)
                        Uout(:,j) = tempout(1:obj.nInputs);
                    end
                end
                
                % Determine progress index with respect to reference theta
                % for next iteration
                for pind = 1:obj.nStages+1
                    [~,progress_id(pind)] = min(abs(repmat(Xout(13,pind),1,obj.nStages+1) - obj.theta));
                end
                
                % Compute solution to decide when to stop iterating
                progress_diff = abs(progress_id - old_progress_id);
                solution = mean(progress_diff);
                % coor_diff = abs(obj.positions - Xout(1:3,:));
                % solution = mean(mean(coor_diff));
                disp(['solution = ', num2str(solution)]);
            end
        end
        
        function [relative_theta_vel, absolute_theta] = user_prompt(obj)
            while (1)
                %% Prompt user for relative timing input
                relative_decision_prompt = 'Do you want to set relative timing to the trajectory? [Y/n]: ';
                relative_decision = input(relative_decision_prompt,'s');
                if (isempty(relative_decision) || relative_decision == 'Y' || relative_decision == 'y')
                    relative_theta_prompt = 'Enter the specific theta values: ';
                    relative_theta = input(relative_theta_prompt);
                    relative_time_prompt = 'Enter the new timings: ';
                    relative_time = input(relative_time_prompt);
                    relative_theta_vel = zeros(1,obj.nStages+1);
                    obj.relative_ind = [];
                    for the = relative_theta
                        [~, nearest_theta] = min(pdist2(obj.theta',the));
                        obj.relative_ind = [obj.relative_ind, nearest_theta];
                    end
                    sg_incr = 1;
                    for db_incr = 1:2:numel(obj.relative_ind)
                        relative_theta_vel(obj.relative_ind(db_incr):obj.relative_ind(db_incr+1)) = ...
                            (relative_theta(db_incr+1) - relative_theta(db_incr))/relative_time(sg_incr);
                        sg_incr = sg_incr + 1;
                    end
                elseif (relative_decision == 'N' || relative_decision == 'n')
                    obj.relative_ind = [];
                    relative_theta_vel = zeros(1,obj.nStages+1);
                else
                    disp('Please enter a valid answer (y/n).');
                end
                
                %% Prompt user for absolute timing input
                absolute_decision_prompt = 'Do you want to set absolute timing to the trajectory? [Y/n]: ';
                absolute_decision = input(absolute_decision_prompt,'s');
                if (isempty(absolute_decision) || absolute_decision == 'Y' || absolute_decision == 'y')
                    absolute_theta_prompt = 'Enter the specific theta values: ';
                    absolute_theta = input(absolute_theta_prompt);
                    absolute_time_prompt = 'Enter the new timings: ';
                    absolute_time = input(absolute_time_prompt);
                    obj.absolute_tk = zeros(1,numel(absolute_time));
                    for ind = 1:numel(absolute_time)
                        obj.absolute_tk(ind) = round(absolute_time(ind)/obj.T);
                    end
                    break;
                elseif (absolute_decision == 'N' || absolute_decision == 'n')
                    obj.absolute_tk = [];
                    absolute_theta = 0;
                    break;
                else
                    disp('Please enter a valid answer (y/n).');
                end
            end
        end
        
        function generate_FORCES_solver(obj, quad_Ad, quad_Bd, quad_gd)
            %% Setup model for FORCES solver
            % Dimensions
            obj.model.N = obj.nStages + 1;  % plus 1 initial stage
            obj.model.nvar = obj.nVars;
            obj.model.neq = obj.nStates;
            obj.model.npar = obj.nPars;
            
            % Short name for indices in z
            nx = obj.nInputs + 1;
            ny = obj.nInputs + 2;
            nz = obj.nInputs + 3;
            nyaw = obj.nInputs + 4;
            njerk = obj.nInputs + 9: obj.nInputs + 12;
            ntheta = obj.nInputs + 13;
            ntv = obj.nInputs + 14;
            ntj = obj.nInputs + 15;
            ncamyaw = obj.nInputs + 16;
            ncampitch = obj.nInputs + 17;
            
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
            
            % Weight functions
            jerk_weight_matrix = obj.jerk_weight*eye(4);
            theta_jerk_weight = obj.jerk_weight;
            progress_control = @(rtv,tt,end_theta) (rtv > 0)*(obj.relative_timing_weight + obj.progress_weight) - ...
                obj.progress_weight + (tt >= end_theta)*(obj.rest_weight + obj.progress_weight);
            absolute_time_control = @(abst) (abst > 0)*(obj.absolute_timing_weight);
            
            % Objectives
%             obj.model.objective = @(z,par) OneShotMPCC.objective(z,par, obj);
            obj.model.objective = @(z,par) ...
                [z(nx) - polyval(par(npx),z(ntheta)); ...
                z(ny) - polyval(par(npy),z(ntheta)); ...
                z(nz) - polyval(par(npz),z(ntheta))]'* ...
                [polyval(par(npdx),z(ntheta))/sqrt(polyval(par(npdx),z(ntheta))^2+ ... 
                polyval(par(npdy),z(ntheta))^2+polyval(par(npdz),z(ntheta))^2); ...
                polyval(par(npdy),z(ntheta))/sqrt(polyval(par(npdx),z(ntheta))^2+ ... 
                polyval(par(npdy),z(ntheta))^2+polyval(par(npdz),z(ntheta))^2); ...
                polyval(par(npdz),z(ntheta))/sqrt(polyval(par(npdx),z(ntheta))^2+ ... 
                polyval(par(npdy),z(ntheta))^2+polyval(par(npdz),z(ntheta))^2)]*obj.lag_weight* ...
                [z(nx) - polyval(par(npx),z(ntheta)); ...
                z(ny) - polyval(par(npy),z(ntheta)); ...
                z(nz) - polyval(par(npz),z(ntheta))]'* ...
                [polyval(par(npdx),z(ntheta))/sqrt(polyval(par(npdx),z(ntheta))^2+ ...
                polyval(par(npdy),z(ntheta))^2+polyval(par(npdz),z(ntheta))^2); ...
                polyval(par(npdy),z(ntheta))/sqrt(polyval(par(npdx),z(ntheta))^2+ ...
                polyval(par(npdy),z(ntheta))^2+polyval(par(npdz),z(ntheta))^2); ...
                polyval(par(npdz),z(ntheta))/sqrt(polyval(par(npdx),z(ntheta))^2+ ...
                polyval(par(npdy),z(ntheta))^2+polyval(par(npdz),z(ntheta))^2)] + ...
                sqrt((z(nx) - polyval(par(npx),z(ntheta)))^2 + ...
                (z(ny) - polyval(par(npy),z(ntheta)))^2 + ...
                (z(nz) - polyval(par(npz),z(ntheta)))^2 - ...
                ([z(nx) - polyval(par(npx),z(ntheta)); ...
                z(ny) - polyval(par(npy),z(ntheta)); ...
                z(nz) - polyval(par(npz),z(ntheta))]'* ...
                [polyval(par(npdx),z(ntheta))/sqrt(polyval(par(npdx),z(ntheta))^2+ ...
                polyval(par(npdy),z(ntheta))^2+polyval(par(npdz),z(ntheta))^2); ...
                polyval(par(npdy),z(ntheta))/sqrt(polyval(par(npdx),z(ntheta))^2+ ...
                polyval(par(npdy),z(ntheta))^2+polyval(par(npdz),z(ntheta))^2); ...
                polyval(par(npdz),z(ntheta))/sqrt(polyval(par(npdx),z(ntheta))^2+ ...
                polyval(par(npdy),z(ntheta))^2+polyval(par(npdz),z(ntheta))^2)])^2)*obj.contour_weight* ...
                sqrt((z(nx) - polyval(par(npx),z(ntheta)))^2 + ...
                (z(ny) - polyval(par(npy),z(ntheta)))^2 + ...
                (z(nz) - polyval(par(npz),z(ntheta)))^2 - ...
                ([z(nx) - polyval(par(npx),z(ntheta)); ...
                z(ny) - polyval(par(npy),z(ntheta)); ...
                z(nz) - polyval(par(npz),z(ntheta))]'* ...
                [polyval(par(npdx),z(ntheta))/sqrt(polyval(par(npdx),z(ntheta))^2+ ...
                polyval(par(npdy),z(ntheta))^2+polyval(par(npdz),z(ntheta))^2); ...
                polyval(par(npdy),z(ntheta))/sqrt(polyval(par(npdx),z(ntheta))^2+ ...
                polyval(par(npdy),z(ntheta))^2+polyval(par(npdz),z(ntheta))^2); ...
                polyval(par(npdz),z(ntheta))/sqrt(polyval(par(npdx),z(ntheta))^2+ ...
                polyval(par(npdy),z(ntheta))^2+polyval(par(npdz),z(ntheta))^2)])^2) + ...
                (z(nyaw) + z(ncamyaw) - polyval(par(npcy),z(ntheta)))'*obj.orientation_weight* ...
                (z(nyaw) + z(ncamyaw) - polyval(par(npcy),z(ntheta))) + ...
                (z(ncampitch) - polyval(par(npcp),z(ntheta)))'*obj.orientation_weight* ...
                (z(ncampitch) - polyval(par(npcp),z(ntheta))) + ...
                (z(ntv) - polyval(par(nptv),z(ntheta)))'* ...
                progress_control(polyval(par(nptv),z(ntheta)),z(ntheta),par(npend))* ...
                (z(ntv) - polyval(par(nptv),z(ntheta))) + ...
                (z(njerk))'*jerk_weight_matrix*(z(njerk)) + ...
                (z(ntj))'*theta_jerk_weight*(z(ntj)) + ...
                (z(ntheta) - par(npabs))'*absolute_time_control(par(npabs))*(z(ntheta) - par(npabs));
            
            % Equality constraints
            obj.model.eq = @(z) quad_Ad*z(obj.nInputs+1:obj.nInputs+obj.nStates) + quad_Bd*z(1:obj.nInputs) + quad_gd;
            
            % LHS matrix of equality constraints
            obj.model.E = [[zeros(8,obj.nInputs); [(-1/(obj.T*obj.mass))*eye(4), zeros(4,3)]; ...
                [zeros(7,4), (-1/obj.T)*[0;0;1;0;0;0;0], zeros(7,2)]], eye(obj.nStates)];
            
            % Initial states
            obj.model.xinitidx = obj.nInputs+1:obj.nInputs+obj.nStates;
            
            % Inequality constraints
            obj.model.lb = [ obj.umin',    obj.xmin'  ];
            obj.model.ub = [ obj.umax',    obj.xmax'  ];
            
            %% Get options for solver
            codeoptions = getOptions('FORCESNLPsolver');
            codeoptions.printlevel = 2;
            codeoptions.maxit = 5000;

            %% Generate code
            FORCES_NLP(obj.model, codeoptions);
        end
        
        function [Ad, Bd, gd] = build_quad_model(obj, T, nStates, nInputs)
            nx = nStates;
            nu = nInputs;
            m = obj.mass;
            g = obj.gravity;
            
            % Continuous state space matrices
            % dot.X = Ac*X + Bc*U
            % dot.X = [vx; vy; vz; wyaw; ax; ay; az; ayaw]
            % X = [x; y; z; yaw; vx; vy; vz; wyaw]
            % U = [Fx; Fy; Fz; Tyaw]
            Ac = zeros(nx,nx);
            Ac(obj.allPosStateIndices(1) - nInputs, obj.allVelStateIndices(1) - nInputs) = 1;
            Ac(obj.allPosStateIndices(2) - nInputs, obj.allVelStateIndices(2) - nInputs) = 1;
            Ac(obj.allPosStateIndices(3) - nInputs, obj.allVelStateIndices(3) - nInputs) = 1;
            Ac(obj.allPosStateIndices(4) - nInputs, obj.allVelStateIndices(4) - nInputs) = 1;
            
            Bc = zeros(nx,nu);
            Bc(obj.allVelStateIndices(1) - nInputs, 1) = 1/m;
            Bc(obj.allVelStateIndices(2) - nInputs, 2) = 1/m;
            Bc(obj.allVelStateIndices(3) - nInputs, 3) = 1/m;
            Bc(obj.allVelStateIndices(4) - nInputs, 4) = 1/m;
            
            Ac(obj.thetaStateIndex - nInputs, obj.thetaVelStateIndex - nInputs) = 1;
            Bc(obj.thetaVelStateIndex - nInputs, 5) = 1;
            
            Ac(obj.gimbalYawStateIndex - nInputs, obj.gimbalYawVelStateIndex - nInputs) = 1;
            Ac(obj.gimbalPitchStateIndex - nInputs, obj.gimbalPitchVelStateIndex - nInputs) = 1;
            Bc(obj.gimbalYawVelStateIndex - nInputs, 6) = 1;
            Bc(obj.gimbalPitchVelStateIndex - nInputs, 7) = 1;
            
            gc = zeros(nx,1);
            gc(7,1) = g;
            
            I = eye(nx);
            
            % Discretization
            A_tilde = [Ac, Bc, I; zeros(nx + nu, nx + nu + nx)];
            A_tilde_d = expm(A_tilde*T);
            
            Ad = A_tilde_d(1:nx,1:nx);
            Bd = A_tilde_d(1:nx,nx+1:nx+nu);
            gd = A_tilde_d(1:nx,nx+nu+1:2*nx+nu) * gc;
            
            % Jerk / Change in acceleration (input)
            % j(i+1) = a(i+1) - a(i)
            Ad(9,9) = 0;
            Ad(10,10) = 0;
            Ad(11,11) = 0;
            Ad(12,12) = 0;
            Bd(9,1) = -1/(T*m);
            Bd(10,2) = -1/(T*m);
            Bd(11,3) = -1/(T*m);
            Bd(12,4) = -1/(T*m);
            
            Ad(15,15) = 0;
            Bd(15,5) = -1/T;
            
            % Remove gimbal quantities which are controlled by inputs
            Ad(obj.gimbalYawVelStateIndex - nInputs, obj.gimbalYawVelStateIndex - nInputs) = 0;
            Ad(obj.gimbalPitchVelStateIndex - nInputs, obj.gimbalPitchVelStateIndex - nInputs) = 0;
        end
    end
    
%     methods (Static)
%         function f = objective(z, par, obj) %...
%         end
%     end
end