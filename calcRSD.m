function [descriptor] = calcRSD(LO, LI, M, r, isDebug)
    % DataAssociation - convert a UNIX timestamp to a human-readable string
    %
    % [date_time] = TimestampToString(t)
    %
    % INPUTS:
    %   LO: size(2 * N), each point is represented as [r; rho]
    %   LI: size(2 * N), each point is represented as [x; y]
    % OUTPUTS:
    %   match_set: string representing t in the format yyyy/mm/dd HH:MM:SS.FFF
    N = size(LO, 2);
    %RSDs = zeros(5, M, N);
    rng_res = 1;
    descriptor = zeros(ceil(r / rng_res), N);
    % calculate RSD for each point
    for i = 1 : N
        % debug purpose
        if isDebug
            figure(1);
            polarplot(LO(2,:), LO(1,:), '.');
        end
        
        % current point 
        current_point = LO(:, i);
        % select neighboring points 
        % making their origin at (current_point(1), current_point(2))
        neighboring_points = LI - current_point;
        dist2current = sum(neighboring_points.^2, 1); 
        for ii = 1 : size(dist2current, 2)
            if(dist2current(ii) < r^2)
                distance = sqrt(dist2current(ii));
                rngbin = round(distance - rng_res / 2) + 1;
                descriptor(rngbin, i) = descriptor(rngbin, i) + 1; 
            end
        end
        if isDebug
            assert(ref_az <= 2 * pi && ref_az >= 0);
        end

        % debug purpose
        if isDebug
            figure(2);
            polarplot(angles, LO(1,:), '.');
        end
        %       
       
        % for j = 1 : M
        %     idx = (angles >= (j - 1) * az_res & angles < j * az_res);           % idx of points falls with azimuth bin
        %     if(sum(idx) > 0)
        %         RSD(:, j) = [sum(idx); mean(neighboring_points(:, idx), 2); harmmean(neighboring_points(:, idx), 2)];
        %     else
        %         RSD(:, j) = [0; 0; 0; 0; 0];
        %     end
        % end

        %if isDebug
        %  assert(sum(RSD(1, :)) == size(neighboring_points, 2), 'sum of the numbers of points in all partition should equal to N');
        %end

        % normalize all rows
        %for j = 1 : 5
        descriptor(:, i) = rescale(descriptor(:, i));
        %end


        % make sure highest-density slice is the first
        % [~, I] = sort(RSD(1, :), 'descend');
        % RSD = RSD(:, I);

        %RSDs(:,:, i) = RSD;
    end
    %fprintf('RSD calculation finished\n');