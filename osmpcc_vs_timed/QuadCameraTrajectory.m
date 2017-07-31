classdef QuadCameraTrajectory < handle
    properties
        %% High-level trajectory constraints
        T;
        keytimes;  % keytimes contains the timepoints for the N keyframes
        keyframes;  % keyframes is a Nx4 matrix with the 3D position and yaw of the quadrotor for each timepoint
        keyorientations;  % keyorientations is a Nx2 matrix with the yaw and pitch angle of the camera for each timepoint
        %% Weights and constraints
        position_weight;
        derivative_weights;
        camera_weight;
        corridor_constraints;
        velocity_eq_constraints;
        acceleration_eq_constraints;
        yaw_vel_eq_constraints;
        yaw_acc_eq_constraints;
        %% Parameters
        loop_flag;
        TIME_TOLERANCE;
        mass;
        gravity;
        umax;
        umin;
        xmax;
        xmin;
        %% Properties for optimization
        times;
        nStates = 28;
        nInputs = 6;
        nVars;
        nStages;
        % Input indices
        inputIndices;
        % Position state indices
        posStateIndices;
        velStateIndices;
        accStateIndices;
        jerkStateIndices;
        snapStateIndices;
        % Yaw state indices
        yawStateIndex;
        yawVelStateIndex;
        yawAccStateIndex;
        yawJerkStateIndex;
        yawSnapStateIndex;
        % Camera state indices
        gimbalYawStateIndex;
        gimbalPitchStateIndex;
        gimbalYawVelStateIndex;
        gimbalPitchVelStateIndex;
        gimbalYawAccStateIndex;
        gimbalPitchAccStateIndex;
        gimbalYawJerkStateIndex;
        gimbalPitchJerkStateIndex;
        % Combined state indices
        allPosStateIndices;
        allVelStateIndices;
        allAccStateIndices;
        allJerkStateIndices;
        allSnapStateIndices;
        allCameraStateIndices;
        % Running index for equality constraints
        idx_eq_constraint = 0;
        %% Problem matrices and vectors
        H_all;
        f_all;
        A_all;
        b_all;
        Aeq_all;
        beq_all;
        lb_all;
        ub_all;
    end
    methods
        function obj = QuadCameraTrajectory(T, keytimes, keyframes, keyorientations, options)
            if nargin < 4
                options = struct();
            end

            %% Initialize state vector indices
            obj.inputIndices = 1:obj.nInputs;
            % Position state indices
            obj.posStateIndices = obj.nInputs + (1:3);
            obj.velStateIndices = obj.posStateIndices + 4;
            obj.accStateIndices = obj.velStateIndices + 4;
            obj.jerkStateIndices = obj.accStateIndices + 4;
            obj.snapStateIndices = obj.jerkStateIndices + 4;
            % Yaw state indices
            obj.yawStateIndex = obj.posStateIndices(3) + 1;
            obj.yawVelStateIndex = obj.velStateIndices(3) + 1;
            obj.yawAccStateIndex = obj.accStateIndices(3) + 1;
            obj.yawJerkStateIndex = obj.jerkStateIndices(3) + 1;
            obj.yawSnapStateIndex = obj.snapStateIndices(3) + 1;
            % Camera state indices
            obj.gimbalYawStateIndex = obj.yawSnapStateIndex + 1;
            obj.gimbalPitchStateIndex = obj.gimbalYawStateIndex + 1;
            obj.gimbalYawVelStateIndex = obj.gimbalYawStateIndex + 2;
            obj.gimbalPitchVelStateIndex = obj.gimbalYawStateIndex + 3;
            obj.gimbalYawAccStateIndex = obj.gimbalYawStateIndex + 4;
            obj.gimbalPitchAccStateIndex = obj.gimbalYawStateIndex + 5;
            obj.gimbalYawJerkStateIndex = obj.gimbalYawStateIndex + 6;
            obj.gimbalPitchJerkStateIndex = obj.gimbalYawStateIndex + 7;

            obj.T = T;
            obj.keytimes = keytimes;
            obj.keyframes = keyframes;
            obj.keyorientations = keyorientations;

            obj.times = keytimes(1):T:keytimes(end);
            obj.nVars = obj.nStates + obj.nInputs;
            obj.nStages = numel(obj.times);

            obj.allPosStateIndices = [obj.posStateIndices, obj.yawStateIndex];
            obj.allVelStateIndices = [obj.velStateIndices, obj.yawVelStateIndex];
            obj.allAccStateIndices = [obj.accStateIndices, obj.yawAccStateIndex];
            obj.allJerkStateIndices = [obj.jerkStateIndices, obj.yawJerkStateIndex];
            obj.allSnapStateIndices = [obj.snapStateIndices, obj.yawSnapStateIndex];
            obj.allCameraStateIndices = [obj.gimbalYawStateIndex, ...
                obj.gimbalPitchStateIndex, ...
                obj.gimbalYawVelStateIndex, ...
                obj.gimbalPitchVelStateIndex, ...
                obj.gimbalYawAccStateIndex, ...
                obj.gimbalPitchAccStateIndex, ...
                obj.gimbalYawJerkStateIndex, ...
                obj.gimbalPitchJerkStateIndex];

            parse_options(obj, options);
        end

        function parse_options(obj, options)
            %% Initialize (default) parameters
            if ~isfield(options, 'camera_weight')
                options.camera_weight = 1;
            end
            if ~isfield(options, 'derivative_weights')
                assert(size(options.derivative_weights, 2) == 6);
                options.derivative_weights = zeros(1, 6);
                options.derivative_weights(options.optimization_on_derivative) = options.derivative_weight;
            end
            if ~isfield(options, 'corridor_constraints')
                options.corridor_constraints = [];
            end
            if ~isfield(options, 'velocity_eq_constraints')
                options.velocity_eq_constraints = [];
            end
            if ~isfield(options, 'acceleration_constraints')
                options.acceleration_eq_constraints = [];
            end
            if ~isfield(options, 'yaw_vel_eq_constraints')
                options.yaw_vel_eq_constraints = [];
            end
            if ~isfield(options, 'yaw_acc_eq_constraints')
                options.yaw_acc_eq_constraints = [];
            end
            if ~isfield(options, 'loop_flag')
                options.loop_flag = false;
            end
            if ~isfield(options, 'TIME_TOLERANCE')
                options.TIME_TOLERANCE = obj.T / 10;
            end
            if ~isfield(options, 'mass')
                options.mass = 0.439;
            end
            if ~isfield(options, 'gravity')
                options.gravity = 9.81;
            end
            if ~isfield(options, 'umax')
                options.umax = [+4;+4;+8;+4;+2;+2];
            end
            if ~isfield(options, 'umin')
                options.umin = -options.umax;
            end
            if ~isfield(options, 'xmax')
                options.xmax = +100 * ones(obj.nVars - obj.nInputs, 1);
            end
            if ~isfield(options, 'xmin')
                options.xmin = -options.xmax;
            end

            %% Extract options
            obj.position_weight = options.position_weight;
            obj.derivative_weights = options.derivative_weights;
            obj.camera_weight = options.camera_weight;
            obj.corridor_constraints = options.corridor_constraints;
            obj.velocity_eq_constraints = options.velocity_eq_constraints;
            obj.acceleration_eq_constraints = options.acceleration_eq_constraints;
            obj.yaw_vel_eq_constraints = options.yaw_vel_eq_constraints;
            obj.yaw_acc_eq_constraints = options.yaw_acc_eq_constraints;
            obj.loop_flag = options.loop_flag;
            obj.TIME_TOLERANCE = options.TIME_TOLERANCE;
            obj.mass = options.mass;
            obj.gravity = options.gravity;
            obj.umax = options.umax;
            obj.umin = options.umin;
            obj.xmax = options.xmax;
            obj.xmin = options.xmin;
        end

        function [X, fval] = generate(obj)
            %% Build the quadrotor model
            [ui_xi, gd] = build_model(obj, obj.T, obj.nVars, obj.nInputs);

            %% Define lower and upper bounds of inputs and states
            % input (u) limits based on drones max force = [4,4,8]
            lb = [obj.umin; obj.xmin];
            ub = [obj.umax; obj.xmax];

            %% Single-stage quadratic cost matrix
            H = zeros(obj.nVars, obj.nVars);
            % on velocity
            H(obj.allVelStateIndices, obj.allVelStateIndices) = obj.derivative_weights(1) * eye(4);
            % on acceleration
            H(obj.allAccStateIndices, obj.allAccStateIndices) = obj.derivative_weights(2) * eye(4);
            % on jerk
            H(obj.allJerkStateIndices, obj.allJerkStateIndices) = obj.derivative_weights(3) * eye(4);
            % on snap
            H(obj.allSnapStateIndices, obj.allSnapStateIndices) = obj.derivative_weights(4) * eye(4);
            % on camera yaw velocity
            H(obj.gimbalYawJerkStateIndex, obj.gimbalYawJerkStateIndex) = obj.derivative_weights(5);
            % on camera pitch velocity
            H(obj.gimbalPitchJerkStateIndex, obj.gimbalPitchJerkStateIndex) = obj.derivative_weights(6);

            %% Create system matrices
            %for normal trajectories + 8 constraints for hard start and end point (yaw
            %= 0)
            additional_eq_constraints = 8;
            %if start and end keyframe equal + 8 constraints
            if obj.loop_flag
                additional_eq_constraints = additional_eq_constraints + 8;
            else %if start and end keyframe different + 16 constraints (2 * 4 velocities + 2 * 4 accelerations)
                additional_eq_constraints = additional_eq_constraints + 16;
            end

            % Compute number of equality onstraints
            num_of_eq_constraints = (obj.nVars - obj.nInputs) * (obj.nStages-1) + additional_eq_constraints;

            % Build full system matrices
            create_system_matrices(obj, num_of_eq_constraints);

            %% Set upper and lower bounds for all stages
            for k=1:obj.nStages
                % Define upper and lower bounds
                set_stage_bounds(obj, k, lb, ub);
            end

            %% Set start and end position as hard constraint
            position_eq_constraints = [
                obj.keytimes(1), obj.keyframes(1, 1:3);
                obj.keytimes(end), obj.keyframes(end, 1:3)
            ];
            yaw_eq_constraints = [
                obj.keytimes(1), obj.keyframes(1, 4);
                obj.keytimes(end), obj.keyframes(end, 4)
            ];

            %% Define energy terms for derivatives (L2-norm)
            for stage=1:obj.nStages
                quadratic_weights = H;
                linear_weights = zeros(size(H, 1), 1);
                set_stage_energy_weights(obj, stage, 1:obj.nVars, quadratic_weights, linear_weights);
            end

            %% Add inter-stage constraints for the quadcopter model
            for k=1:obj.nStages
                % This links stage k to stage k+1)
                if k ~= obj.nStages
                    add_stage_eq_constraint(obj, k, ui_xi, gd);
                end
            end

            %% Define soft constraints for keyframes (L2-norm) (exclude first and last
            % keyframe as they already have hard constraints)
            for k=2:size(obj.keyframes, 1) - 1
                stage = find_stage(obj, obj.keytimes(k));
                %% (position - desired_position) .^ 2 = position .^2 - 2 * position * desired_position + const_term
                quadratic_weights = obj.position_weight * eye(3);
                linear_weights = -2 * obj.position_weight * obj.keyframes(k, 1:3)';
                set_stage_energy_weights(obj, stage, obj.posStateIndices, quadratic_weights, linear_weights);
            end

            %% Define soft constraints on camera direction for keyframes (L2-norm)
            for k=1:size(obj.keyframes, 1)
                stage = find_stage(obj, obj.keytimes(k));
                %% (position - desired_position) .^ 2 = position .^2 - 2 * position * desired_position + const_term
                quadratic_weights = obj.camera_weight * eye(3);
                quadratic_weights(1, 2) = obj.camera_weight;
                quadratic_weights(2, 1) = obj.camera_weight;
                linear_weights = [-2 * obj.camera_weight * obj.keyorientations(k, 1);
                    -2 * obj.camera_weight * obj.keyorientations(k, 1);
                    -2 * obj.camera_weight * obj.keyorientations(k, 2)];
                state_indices = [obj.yawStateIndex, obj.gimbalYawStateIndex, obj.gimbalPitchStateIndex];
                add_stage_energy_weights(obj, stage, state_indices, quadratic_weights, linear_weights);
            end

            %% Set constraints for start and end points
            if obj.loop_flag
                stage1 = 1;
                stage2 = obj.nStages;

                state_indices = obj.allVelStateIndices;
                [Aeq_rows, beq_rows] = build_stage_link_eq_constraint(obj, stage1, stage2, state_indices);
                add_eq_constraint(obj, Aeq_rows, beq_rows);

                state_indices = obj.allAccStateIndices;
                [Aeq_rows, beq_rows] = build_stage_link_eq_constraint(obj, stage1, stage2, state_indices);
                add_eq_constraint(obj, Aeq_rows, beq_rows);
            else
                % Add zero velocity and acceleration constraints for first and last
                % stage.
                obj.velocity_eq_constraints = [obj.velocity_eq_constraints; [obj.keytimes(1), zeros(1, 3)]];
                obj.velocity_eq_constraints = [obj.velocity_eq_constraints; [obj.keytimes(end), zeros(1, 3)]];
                obj.yaw_vel_eq_constraints = [obj.yaw_vel_eq_constraints; [obj.keytimes(1), 0]];
                obj.yaw_vel_eq_constraints = [obj.yaw_vel_eq_constraints; [obj.keytimes(end), 0]];
                obj.acceleration_eq_constraints = [obj.acceleration_eq_constraints; [obj.keytimes(1), zeros(1, 3)]];
                obj.acceleration_eq_constraints = [obj.acceleration_eq_constraints; [obj.keytimes(end), zeros(1, 3)]];
                obj.yaw_acc_eq_constraints = [obj.yaw_acc_eq_constraints; [obj.keytimes(1), 0]];
                obj.yaw_acc_eq_constraints = [obj.yaw_acc_eq_constraints; [obj.keytimes(end), 0]];
            end

            %% Add position equality constraints
            for i=1:size(position_eq_constraints, 1)
                constraint_time = position_eq_constraints(i, 1);
                constraint_position = position_eq_constraints(i, 2:4);
                [Aeq_rows, beq_rows] = build_position_eq_constraint(obj, constraint_time, constraint_position);
                add_eq_constraint(obj, Aeq_rows, beq_rows);
            end

            %% Add velocity equality constraints
            for i=1:size(obj.velocity_eq_constraints, 1)
                constraint_time = obj.velocity_eq_constraints(i, 1);
                constraint_velocity = obj.velocity_eq_constraints(i, 2:4);
                [Aeq_rows, beq_rows] = build_velocity_eq_constraint(obj, constraint_time, constraint_velocity);
                add_eq_constraint(obj, Aeq_rows, beq_rows);
            end

            %% Add acceleration equality constraints
            for i=1:size(obj.acceleration_eq_constraints, 1)
                constraint_time = obj.acceleration_eq_constraints(i, 1);
                constraint_acceleration = obj.acceleration_eq_constraints(i, 2:4);
                [Aeq_rows, beq_rows] = build_acceleration_eq_constraint(obj, constraint_time, constraint_acceleration);
                add_eq_constraint(obj, Aeq_rows, beq_rows);
            end

            %% Add yaw equality constraints
            for i=1:size(yaw_eq_constraints, 1)
                constraint_time = yaw_eq_constraints(i, 1);
                constraint_yaw = yaw_eq_constraints(i, 2);
                [Aeq_rows, beq_rows] = build_yaw_eq_constraint(obj, constraint_time, constraint_yaw);
                add_eq_constraint(obj, Aeq_rows, beq_rows);
            end

            %% Add yaw velocity equality constraints
            for i=1:size(obj.yaw_vel_eq_constraints, 1)
                constraint_time = obj.yaw_vel_eq_constraints(i, 1);
                constraint_yaw_vel = obj.yaw_vel_eq_constraints(i, 2);
                [Aeq_rows, beq_rows] = build_yaw_vel_eq_constraint(obj, constraint_time, constraint_yaw_vel);
                add_eq_constraint(obj, Aeq_rows, beq_rows);
            end

            %% Add yaw acceleration equality constraints
            for i=1:size(obj.yaw_acc_eq_constraints, 1)
                constraint_time = obj.yaw_acc_eq_constraints(i, 1);
                constraint_yaw_acc = obj.yaw_acc_eq_constraints(i, 2);
                [Aeq_rows, beq_rows] = build_yaw_acc_eq_constraint(obj, constraint_time, constraint_yaw_acc);
                add_eq_constraint(obj, Aeq_rows, beq_rows);
            end

            %% TODO: Define corridor constraints
            [A_corr, b_corr] = define_corridor_constraints(obj, obj.nStates, obj.nInputs, obj.nStages, obj.keytimes, obj.T, obj.corridor_constraints, obj.keyframes);
            obj.A_all = A_corr;
            obj.b_all = b_corr;
            % obj.A_all = [obj.A_all;A_corr];
            % obj.b_all = [obj.b_all;b_corr];

            [X, fval] = solve_system(obj);
        end

        function [X, fval] = solve_system(obj)
            quad_options = optimoptions('quadprog','Algorithm','interior-point-convex','Display','iter','TolFun',1e-5,'TolX',1e-5,'MaxIter',1000);
            [x,fval,exitflag] = quadprog(obj.H_all, obj.f_all, obj.A_all, obj.b_all, obj.Aeq_all, obj.beq_all, obj.lb_all, obj.ub_all, [], quad_options);

            if isempty(exitflag ~= 1)
                if exitflag == -2
                    error('Problem is infeasible.');
                else
                    error('Unable to find a suitable trajectory.');
                end
            end
            X = reshape(x, [obj.nVars, obj.nStages]);
        end

        function create_system_matrices(obj, num_of_eq_constraints)
            % Build full problem matrix
            obj.H_all = zeros(obj.nVars * obj.nStages, obj.nVars * obj.nStages);
            obj.f_all = zeros(obj.nVars * obj.nStages, 1);

            % Create equality constraint matrices
            obj.Aeq_all = zeros(num_of_eq_constraints, obj.nVars * obj.nStages);
            obj.beq_all = zeros(num_of_eq_constraints, 1);

            % Lower and upper bounds on input and states
            obj.lb_all = zeros(obj.nVars * obj.nStages, 1);
            obj.ub_all = zeros(obj.nVars * obj.nStages, 1);

        end

        function stage = find_stage(obj, time)
            stage = find(obj.times <= time + 0.001, 1, 'last');
            if isempty(stage)
                error(['Could not find a stage matching the specified time: ', num2str(time)]);
            end
            time_diff = time - obj.times(stage);
            if time_diff > obj.TIME_TOLERANCE
                error(['Could not find a stage matching the specified time: ', num2str(time), ...
                    '. Closest previous stage has time: ', num2str(obj.times(stage))]);
            end
        end


        function add_stage_energy_weights(obj, stage, state_indices, quadratic_weights, linear_weights)
            stage_indices = get_stage_indices(obj, stage);
            indices = stage_indices(state_indices);

            obj.H_all(indices, indices) = obj.H_all(indices, indices) + quadratic_weights;
            % We multiply with 0.5 here because Q in quadprog is also
            % multiplied with 0.5.
            obj.f_all(indices, 1) = obj.f_all(indices, 1) + 0.5 * linear_weights;
        end


        function set_stage_energy_weights(obj, stage, state_indices, quadratic_weights, linear_weights)
            stage_indices = get_stage_indices(obj, stage);
            indices = stage_indices(state_indices);

            obj.H_all(indices, indices) = quadratic_weights;
            % We multiply with 0.5 here because Q in quadprog is also
            % multiplied with 0.5.
            obj.f_all(indices, 1) = 0.5 * linear_weights;
        end


        function set_stage_bounds(obj, stage, lb, ub)
            indices = get_stage_indices(obj, stage);
            obj.lb_all(indices) = lb;
            obj.ub_all(indices) = ub;
        end


        function [Aeq_rows, beq_rows] = build_stage_link_eq_constraint(obj, stage1, stage2, state_indices)
            stage_indices1 = get_stage_indices(obj, stage1);
            indices1 = stage_indices1(state_indices);
            stage_indices2 = get_stage_indices(obj, stage2);
            indices2 = stage_indices2(state_indices);
            Aeq_rows = zeros(2 * numel(state_indices), obj.nVars * obj.nStages);
            Aeq_rows(1:numel(state_indices), indices1) = eye(numel(state_indices));
            Aeq_rows(numel(state_indices)+1:end, indices2) = -eye(numel(state_indices));
            beq_rows = zeros(numel(state_indices), 1);
        end

        function [Aeq_rows, beq_rows] = build_stage_eq_constraint(obj, stage, state_indices, values)
            stage_indices = get_stage_indices(obj, stage);
            indices = stage_indices(state_indices);
            Aeq_rows = zeros(numel(state_indices), obj.nVars * obj.nStages);
            Aeq_rows(:, indices) = eye(numel(state_indices));
            beq_rows = reshape(values, [numel(state_indices), 1]);
        end

        function [Aeq_rows, beq_rows] = build_position_eq_constraint(obj, constraint_time, constraint_position)
            stage = find_stage(obj, constraint_time);
            [Aeq_rows, beq_rows] = build_stage_eq_constraint(obj, stage, obj.posStateIndices, constraint_position);
        end

        function [Aeq_rows, beq_rows] = build_velocity_eq_constraint(obj, constraint_time, constraint_velocity)
            stage = find_stage(obj, constraint_time);
            [Aeq_rows, beq_rows] = build_stage_eq_constraint(obj, stage, obj.velStateIndices, constraint_velocity);
        end

        function [Aeq_rows, beq_rows] = build_acceleration_eq_constraint(obj, constraint_time, constraint_acceleration)
            stage = find_stage(obj, constraint_time);
            [Aeq_rows, beq_rows] = build_stage_eq_constraint(obj, stage, obj.accStateIndices, constraint_acceleration);
        end


        function [Aeq_rows, beq_rows] = build_yaw_eq_constraint(obj, constraint_time, constraint_yaw)
            stage = find_stage(obj, constraint_time);
            [Aeq_rows, beq_rows] = build_stage_eq_constraint(obj, stage, obj.yawStateIndex, constraint_yaw);
        end

        function [Aeq_rows, beq_rows] = build_yaw_vel_eq_constraint(obj, constraint_time, constraint_yaw_vel)
            stage = find_stage(obj, constraint_time);
            [Aeq_rows, beq_rows] = build_stage_eq_constraint(obj, stage, obj.yawVelStateIndex, constraint_yaw_vel);
        end

        function [Aeq_rows, beq_rows] = build_yaw_acc_eq_constraint(obj, constraint_time, constraint_yaw_acc)
            stage = find_stage(obj, constraint_time);
            [Aeq_rows, beq_rows] = build_stage_eq_constraint(obj, stage, obj.yawAccStateIndex, constraint_yaw_acc);
        end


        function add_eq_constraint(obj, Aeq_rows, beq_rows)
            constraint_indices = (obj.idx_eq_constraint + 1) : (obj.idx_eq_constraint + size(Aeq_rows, 1));
            obj.Aeq_all(constraint_indices, :) = Aeq_rows;
            obj.beq_all(constraint_indices, :) = beq_rows;
            obj.idx_eq_constraint = obj.idx_eq_constraint + size(Aeq_rows, 1);
        end

        function add_stage_eq_constraint(obj, stage, Aeq_rows, beq_rows)
            stage_indices = [get_stage_indices(obj, stage), get_stage_indices(obj, stage + 1)];
            constraint_indices = (obj.idx_eq_constraint + 1) : (obj.idx_eq_constraint + size(Aeq_rows, 1));
            obj.Aeq_all(constraint_indices, stage_indices) = Aeq_rows;
            obj.beq_all(constraint_indices, :) = beq_rows;
            obj.idx_eq_constraint = obj.idx_eq_constraint + size(Aeq_rows, 1);
        end

        function indices = get_stage_indices(obj, stage)
            indices = (stage - 1) * obj.nVars + (1:obj.nVars);
        end

        function [ui_xi, gd] = build_model(obj, T, nStates, nInputs)
            nx = nStates - nInputs;
            nu = nInputs;
            m = obj.mass;
            g = obj.gravity;

            Ac = zeros(nx, nx);
            Ac(obj.allPosStateIndices(1) - nInputs, obj.allVelStateIndices(1) - nInputs) = 1;
            Ac(obj.allPosStateIndices(2) - nInputs, obj.allVelStateIndices(2) - nInputs) = 1;
            Ac(obj.allPosStateIndices(3) - nInputs, obj.allVelStateIndices(3) - nInputs) = 1;
            Ac(obj.allPosStateIndices(4) - nInputs, obj.allVelStateIndices(4) - nInputs) = 1;

            Bc = zeros(nx,nu);
            Bc(obj.allVelStateIndices(1) - nInputs, 1) = 1/m;
            Bc(obj.allVelStateIndices(2) - nInputs, 2) = 1/m;
            Bc(obj.allVelStateIndices(3) - nInputs, 3) = 1/m;
            Bc(obj.allVelStateIndices(4) - nInputs, 4) = 1/m;

            Ac(obj.gimbalYawStateIndex - nInputs, obj.gimbalYawVelStateIndex - nInputs) = 1;
            Ac(obj.gimbalPitchStateIndex - nInputs, obj.gimbalPitchVelStateIndex - nInputs) = 1;
            Bc(obj.gimbalYawVelStateIndex - nInputs, 5) = 1;
            Bc(obj.gimbalPitchVelStateIndex - nInputs, 6) = 1;

            gc = zeros(nx,1);
            gc(7,1) = g;

            I = eye(nx);

            A_tilde = [Ac, Bc, I; zeros(nx + nu, nx + nu + nx)];
            A_tilde_d = expm(A_tilde*T);

            Ad = A_tilde_d(1:nx,1:nx);
            Bd = A_tilde_d(1:nx,nx+1:nx+nu);
            gd = A_tilde_d(1:nx,nx+nu+1:2*nx+nu) * gc;

            I_minus = -I;

            %% Remove identity entries for quantities controlled by the inputs
            Ad(12,12) = 0;
            Ad(11,11) = 0;
            Ad(10,10) = 0;
            Ad(9,9) = 0;

            Ad(obj.gimbalYawVelStateIndex - nInputs, obj.gimbalYawVelStateIndex - nInputs) = 0;
            Ad(obj.gimbalPitchVelStateIndex - nInputs, obj.gimbalPitchVelStateIndex - nInputs) = 0;

            %% Finite differentation
            %a(i+1) = v(i+1) - v(i)
            Ad(12,8) = -1/T;
            Ad(11,7) = -1/T;
            Ad(10,6) = -1/T;
            Ad(9,5) = -1/T;

            I_minus(12,8) = 1/T;
            I_minus(11,7) = 1/T;
            I_minus(10,6) = 1/T;
            I_minus(9,5) = 1/T;

            %j(i+1) = a(i+1) - a(i) 
            Ad(16,16) = 0;
            Ad(15,15) = 0;
            Ad(14,14) = 0;
            Ad(13,13) = 0;

            Ad(16,12) = -1/T;
            Ad(15,11) = -1/T;
            Ad(14,10) = -1/T;
            Ad(13,9) = -1/T;

            I_minus(16,12) = 1/T;
            I_minus(15,11) = 1/T;
            I_minus(14,10) = 1/T;
            I_minus(13,9) = 1/T;

            %s(i+1) = j(i+1) - j(i)
            Ad(20,20) = 0;
            Ad(19,19) = 0;
            Ad(18,18) = 0;
            Ad(17,17) = 0;

            Ad(20,16) = -1/T;
            Ad(19,15) = -1/T;
            Ad(18,14) = -1/T;
            Ad(17,13) = -1/T;

            I_minus(20,16) = 1/T;
            I_minus(19,15) = 1/T;
            I_minus(18,14) = 1/T;
            I_minus(17,13) = 1/T;
            
            %for gimbal acceleration
            Ad(25,25) = 0;
            Ad(26,26) = 0;
            Ad(25,23) = -1/T;
            Ad(26,24) = -1/T;
            
            I_minus(25,23) = 1/T;
            I_minus(26,24) = 1/T;

            %for gimbal jerk
            Ad(27,27) = 0;
            Ad(28,28) = 0;
            Ad(27,25) = -1/T;
            Ad(28,26) = -1/T;
            
            I_minus(27,25) = 1/T;
            I_minus(28,26) = 1/T;

            ui_xi = [Bd, Ad, zeros(nx,nu), I_minus];
        end

        % TODO: this needs cleanup
        function [A, b] = define_corridor_constraints(obj, nx,nu,kmax,t,T,corridor_constraints, keyframes)
            corrcons_size = size(corridor_constraints,1);
            size_c = 0;
            for i = 1:corrcons_size   
                tx1 = corridor_constraints(i,1);
                tx2 = corridor_constraints(i,2);
                size_c = size_c + 6 * (tx2-tx1)/T;
            end

            A = zeros(size_c, (nx+nu) * kmax);
            b = zeros(size_c, 1);
            row_ix = 0;
            %corridor constraints
            for i = 1:corrcons_size   
                tx1 = corridor_constraints(i,1);
                tx2 = corridor_constraints(i,2);
                delta = corridor_constraints(i,3);

                for k = t(1,tx1)/T:t(1,tx2)/T
                    unit_vektor = (keyframes(tx2,1:3)' - keyframes(tx1,1:3)') / norm(keyframes(tx2,1:3)' - keyframes(tx1,1:3)');
                    start_vektor = keyframes(tx1,1:3)';

                    %x-corridor x <= delta
                    A(row_ix + 1,(nx+nu) * k + 5) = (-1 + unit_vektor(1,1)^2);
                    A(row_ix + 1,(nx+nu) * k + 6) = unit_vektor(1,1) * unit_vektor(2,1);
                    A(row_ix + 1,(nx+nu) * k + 7) = unit_vektor(1,1) * unit_vektor(3,1);
                    b(row_ix + 1,1) = delta + (unit_vektor(1,1)^2-1)*start_vektor(1,1) + unit_vektor(1,1)*unit_vektor(2,1)*start_vektor(2,1) + unit_vektor(1,1)*unit_vektor(3,1)*start_vektor(3,1);

                    %y-corridor y <= delta
                    A(row_ix + 2,(nx+nu) * k + 5) = unit_vektor(1,1) * unit_vektor(2,1);
                    A(row_ix + 2,(nx+nu) * k + 6) = (-1 + unit_vektor(2,1)^2);
                    A(row_ix + 2,(nx+nu) * k + 7) = unit_vektor(2,1) * unit_vektor(3,1);
                    b(row_ix + 2,1) = delta + (unit_vektor(2,1)^2-1)*start_vektor(2,1) + unit_vektor(1,1)*unit_vektor(2,1)*start_vektor(1,1) + unit_vektor(2,1)*unit_vektor(3,1)*start_vektor(3,1);

                    %z-corridor z <= delta
                    A(row_ix + 3,(nx+nu) * k + 5) = unit_vektor(1,1) * unit_vektor(3,1);
                    A(row_ix + 3,(nx+nu) * k + 6) = unit_vektor(3,1)  *unit_vektor(2,1);
                    A(row_ix + 3,(nx+nu) * k + 7) = -1+unit_vektor(3,1)^2;
                    b(row_ix + 3,1) = delta + (unit_vektor(3,1)^2-1)*start_vektor(3,1) + unit_vektor(3,1)*unit_vektor(2,1)*start_vektor(2,1) + unit_vektor(3,1)*unit_vektor(1,1)*start_vektor(1,1); 

                    %x-corridor -x <= delta
                    A(row_ix + 4,(nx+nu) * k + 5) = (1 - unit_vektor(1,1)^2);
                    A(row_ix + 4,(nx+nu) * k + 6) = -unit_vektor(1,1) * unit_vektor(2,1);
                    A(row_ix + 4,(nx+nu) * k + 7) = -unit_vektor(1,1) * unit_vektor(3,1);
                    b(row_ix + 4,1) = delta - (unit_vektor(1,1)^2-1)*start_vektor(1,1) - unit_vektor(1,1)*unit_vektor(2,1)*start_vektor(2,1) - unit_vektor(1,1)*unit_vektor(3,1)*start_vektor(3,1);

                    %y-corridor -y <= delta
                    A(row_ix + 5,(nx+nu) * k + 5) = -unit_vektor(1,1) * unit_vektor(2,1);
                    A(row_ix + 5,(nx+nu) * k + 6) = (1 - unit_vektor(2,1)^2);
                    A(row_ix + 5,(nx+nu) * k + 7) = -unit_vektor(2,1) * unit_vektor(3,1);
                    b(row_ix + 5,1) = delta - (unit_vektor(2,1)^2-1)*start_vektor(2,1) - unit_vektor(1,1)*unit_vektor(2,1)*start_vektor(1,1) - unit_vektor(2,1)*unit_vektor(3,1)*start_vektor(3,1);

                    %z-corridor -z <= delta
                    A(row_ix + 6,(nx+nu) * k + 5) = -unit_vektor(1,1) * unit_vektor(3,1);
                    A(row_ix + 6,(nx+nu) * k + 6) = -unit_vektor(3,1)  *unit_vektor(2,1);
                    A(row_ix + 6,(nx+nu) * k + 7) = 1-unit_vektor(3,1)^2;
                    b(row_ix + 6,1) = delta - (unit_vektor(3,1)^2-1)*start_vektor(3,1) - unit_vektor(3,1)*unit_vektor(2,1)*start_vektor(2,1) - unit_vektor(3,1)*unit_vektor(1,1)*start_vektor(1,1); 

                    row_ix = row_ix + 6;
                end
            end
        end
    end
end