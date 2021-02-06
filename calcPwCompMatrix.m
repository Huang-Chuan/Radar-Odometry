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
    %C = diag(M * ones(1, M));
    % for i = 1 : M
    %     for j = i + 1 : M
    %         l1_i = LO_1(:, B(1,i));
    %         l2_i = LO_2(:, B(2,i));
    %         l1_j = LO_1(:, B(1,j));
    %         l2_j = LO_2(:, B(2,j));
    %         d1 = sum((l1_i - l1_j).^2);
    %         d2 = sum((l2_i - l2_j).^2);
    %         C(i,j) = 1/(1 + abs(d1 - d2));
    %         C(j,i) = C(i, j);
    %     end
    % end


    % vectorized code
    idx_LO_1 = B(1, :);
    idx_LO_2 = B(2, :);
    LO_1_x = LO_1(1, idx_LO_1);
    [xii, xjj] = meshgrid(LO_1_x, LO_1_x);
    LO_1_y = LO_1(2, idx_LO_1);
    [yii, yjj] = meshgrid(LO_1_y, LO_1_y);
    D1 = (xii - xjj).^2 + (yii - yjj).^2; 

    LO_2_x = LO_2(1, idx_LO_2);
    [xii, xjj] = meshgrid(LO_2_x, LO_2_x);
    LO_2_y = LO_2(2, idx_LO_2);
    [yii, yjj] = meshgrid(LO_2_y, LO_2_y);
    D2 = (xii - xjj).^2 + (yii - yjj).^2; 

    C = 1 ./ (1 + abs(D1 - D2));
    C(1:M+1:end) = M;
    %assert(all(r==C, 'all'));
     
