function [match_set] = DataAssociation(LO_1,  LO_2, unary_matches, alpha)

    % DataAssociation - convert a UNIX timestamp to a human-readable string
    %
    % [date_time] = TimestampToString(t)
    %
    % INPUTS:
    %   LO_1: detected points in polar, size(2 * N), each point  
    %   LI_1: size(2 * N),
    %   LO_2: size(2 * M),
    %   LI_2: size(2 * M),
    % OUTPUTS:
    %   unary_matches: string representing t in the format yyyy/mm/dd HH:MM:SS.FFF
    %   match_set: 2 *  
    N = size(LO_1, 2);
    M = size(LO_2, 2);
    dist = inf * ones(N);

       

    w = length(unary_matches);
    c = calcPwCompMatrix(LO_1, LO_2, unary_matches);
    [V,D] = eig(c);
    [d, ind] = sort(diag(D));
    maxEigenVector = V(:, ind(end));
    
    % normalizedVector range [0, 1]
    normalizedVector = maxEigenVector / norm(maxEigenVector);
     if(max(normalizedVector) < 0)
         normalizedVector = -normalizedVector;
     end

    unsearched_idx = 1 : w;
    match_set = [];
    

    while(length(unsearched_idx) > 0 && (length(match_set) < alpha * w))
        [v, ind] = sort(normalizedVector(unsearched_idx), 'descend');
        if(v(1) * w < 1)
            break;
        end
        sel_idx = unsearched_idx(ind(1));
        match_set(:, end + 1) = unary_matches(:, sel_idx);

        %searched_idx = find(cellfun(@(x) (x(1) == match_set(1, end)) || (x(2) == match_set(2, end)), unary_matches));
        searched_idx = find((unary_matches(1, :) == match_set(1, end)) | (unary_matches(2, :) == match_set(2, end)));
        unsearched_idx = setdiff(unsearched_idx, searched_idx);
    end
    