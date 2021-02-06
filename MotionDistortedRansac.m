classdef MotionDistortedRansac < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        landmarks1
        landmarks2
        tolerance = 0.05
        inlier_ratio = 0.9
        iterations = 40
        p1bar
        p2bar
        num_transforms = 21
        delta_ts
        delta_vec
        w_best = [0; 0; 0; 0; 0; 0]
        max_gn_iterations = 10
        epsilon_converge = 0.0001
        error_converge = 0.01
    end
    
    methods
        function obj = MotionDistortedRansac(landmarks1_, landmarks2_, tolerance_, inlier_ratio_, iterations_)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.landmarks1 = landmarks1_;
            obj.landmarks2 = landmarks2_;
            obj.p1bar = [landmarks1_.LO; zeros(1, size(landmarks1_.LO, 2)); ones(1, size(landmarks1_.LO, 2))];  %[x, y, 0, 1]
            obj.p2bar = [landmarks2_.LO; zeros(1, size(landmarks2_.LO, 2)); ones(1, size(landmarks2_.LO, 2))];
            obj.delta_ts = double(obj.landmarks2.t - obj.landmarks1.t) / 1e6;
            obj.delta_vec = linspace(min(obj.delta_ts), max(obj.delta_ts), obj.num_transforms);
            if nargin == 5
                obj.tolerance = tolerance_;
                obj.inlier_ratio = inlier_ratio_;
                obj.iterations = iterations_;
            end
        end
        
        function percentage = computeModel(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            max_inliers = 0;
            subset_size = 2;
            % loop until the ratio of inliers exceeds a certain threshold
            for i = 1 : obj.iterations
                wbar = zeros(6, 1);
                subsetIdx = randperm(size(obj.p1bar, 2), subset_size);
                % To do: change wbar to obj.wbar assuming constant velocity
                wbar = obj.get_motion_parameters(subsetIdx, wbar);
                inliers = obj.getNumInliers(wbar);
                if(inliers > max_inliers)
                    max_inliers = inliers;
                         obj.w_best = wbar;  
                end
                if(max_inliers / size(obj.p1bar, 2) > obj.inlier_ratio)
                    break;
                end
            end
            % Do motion estimation using inlier Idx
            best_inliersIndices = obj.getInliers(obj.w_best);
            obj.w_best = obj.get_motion_parameters(best_inliersIndices, obj.w_best);
            percentage = length(best_inliersIndices) / size(obj.p1bar, 2);
        end

        function T = getTransform(obj, delta_t)
            T = MotionDistortedRansac.se3ToSE3(delta_t * obj.w_best);
        end

        function inlierIndices = getInliers(obj, wbar)
        %myFun - Description
        %
        % Syntax: inlierIndices = getInliers(input)
        %
        % Long description
            inlierIndices = [];
            transforms = zeros(4, 4, length(obj.delta_vec));
            for i = 1 : length(obj.delta_vec)
                transforms(:,:, i) = MotionDistortedRansac.se3ToSE3(wbar * obj.delta_vec(i));
            end
            for i = 1 : size(obj.p1bar, 2)
                p2 = obj.p2bar(:, i);
                p1 = obj.p1bar(:, i);
                error_ = p2 - squeeze(transforms(:, :, MotionDistortedRansac.get_closest(obj.delta_ts(i), obj.delta_vec))) * p1;
                if(norm(error_.^2) < obj. tolerance)
                    inlierIndices(:, end + 1) = i;
                end
            
            end

        end

        function numInliers = getNumInliers(obj, wbar)
            transforms = zeros(4, 4, length(obj.delta_vec));
            for i = 1 : length(obj.delta_vec)
                transforms(:,:, i) = MotionDistortedRansac.se3ToSE3(wbar * obj.delta_vec(i));
            end
            numInliers = 0;
            for i = 1 : size(obj.p1bar, 2)
                p2 = obj.p2bar(:, i);
                p1 = obj.p1bar(:, i);
                error_ = p2 - squeeze(transforms(:, :, MotionDistortedRansac.get_closest(obj.delta_ts(i), obj.delta_vec))) * p1;
                if(norm(error_.^2) < obj.tolerance)
                    numInliers = numInliers + 1;
                end
            end

        end

        function wbar = get_motion_parameters(obj, subsetIdx, wbar)
        %get_motion_parameters subsetIdx, wbarcription
        %
        % Syntax: wbar = get_motion_parameters(subsetIdx, wbar)
        %
        % Long description
            lastError = 0;
            for it = 1 : obj.max_gn_iterations
                A = zeros(6, 6);
                b = zeros(6, 1);
                for m = 1 : length(subsetIdx)
                    Tbar = obj.se3ToSE3(obj.delta_ts(subsetIdx(m)) * wbar);
                    p1   = obj.p1bar(:, subsetIdx(m));
                    p2   = obj.p2bar(:, subsetIdx(m));
                    gbar = Tbar * p1;
                    G    = obj.delta_ts(subsetIdx(m)) * obj.circledot(gbar);
                    ebar = p2 - gbar;
                    A    = A + G.' * G;
                    b    = b + G.' * ebar;
                end
                
                delta_w = A \ b;

                % Line search for best update
                minError = 10000;
                bestAlpha = 1;
                for alpha = 0.1 : 0.1 : 1
                    e = 0;
                    wbar_tmp = wbar + alpha * delta_w;
                    for m = 1 : length(subsetIdx)
                        p1   = obj.p1bar(:, subsetIdx(m));
                        p2   = obj.p2bar(:, subsetIdx(m));
                        Tbar = MotionDistortedRansac.se3ToSE3(obj.delta_ts(subsetIdx(m)) * wbar_tmp);
                        ebar = p2 - Tbar * p1;
                        e    = e + norm(ebar);
                    end                   
                    if (e < minError)
                        minError = e;
                       bestAlpha = alpha;
                    
                    end

                end

                wbar = wbar + bestAlpha * delta_w;

                if(norm(delta_w.^2) < obj.epsilon_converge)
                    break;
                end
                if (it > 1 && abs(lastError - minError) / lastError < obj.error_converge)
                    break;
                end
                lastError = minError;
            end


        end


    end

    methods(Static)
        function idx = get_closest(value, array)
            [~, idx] = min((value - array).^2);
        end
    
        function mtx = cross_(x)
            mtx = [0, -x(3), x(2);...
                   x(3), 0, -x(1);...
                   -x(2), x(1), 0];
        end

        function T = se3ToSE3(xi)
            T = eye(4);
            rho = xi(1 : 3);
            phibar = xi(4 : 6);
            phi = norm(phibar);
            C = eye(3);
            if(phi ~= 0)
                phibar = phibar / phi;
                     C = cos(phi) * eye(3) + (1 - cos(phi)) * (phibar * phibar.') + sin(phi) * MotionDistortedRansac.cross_(phibar);
                % To do: enforce orthogonality
                     J = eye(3) * sin(phi) / phi + (1 - sin(phi) / phi) * (phibar * phibar.') + MotionDistortedRansac.cross_(phibar) * (1 - cos(phi)) / phi;
                     rho = J * rho;
            end
            T(1:3, 1:3) = C;
            T(1:3, 4)   = rho;
        end

        function mtx = circledot(x)
            rho = x(1:3);
            eta = x(4);
            mtx = zeros(4, 6);
            mtx(1:3, 1:3) = eye(3) * eta;
            mtx(1:3, 4:6) = - 1 * MotionDistortedRansac.cross_(rho) ;
        end

    end


end

