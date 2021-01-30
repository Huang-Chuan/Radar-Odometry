function [C] = calcPwCompMatrix(LO_1, LO_2, B)
    % DataAssociation - convert a UNIX timestamp to a human-readable string
    %
    % [date_time] = TimestampToString(t)
    %
    % INPUTS:
    %   LO: detected points in cartesian, size(2 * N), each point is represented as [x; y]
    % OUTPUTS:
    %   match_set: string representing t in the format yyyy/mm/dd HH:MM:SS.FFF
    M = size(LO_1, 2);
    C = diag(M * ones(1, M));
    for i = 1 : M
        for j = i + 1 : M
            l1_i = LO_1(:, B(1,i));
            l2_i = LO_2(:, B(2,i));
            l1_j = LO_1(:, B(1,j));
            l2_j = LO_2(:, B(2,j));
            d1 = sum((l1_i - l1_j).^2);
            d2 = sum((l2_i - l2_j).^2);
            C(i,j) = 1/(1 + abs(d1 - d2));
            C(j,i) = C(i, j);
        end
    end