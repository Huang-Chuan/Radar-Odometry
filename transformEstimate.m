function [SE2] = transformEstimate(data)
    % transformEstimate - Estimate R and v such that R * X + v ~ Y
    %
    % [R, v] = transformEstimate(X, Y)
    %
    % INPUTS:
    %   data: m x 4 matrix
    % OUTPUTS:
    %   R: rotation matrix
    %   v: traslation vector  

    X = data(:, 1:2).';
    Y = data(:, 3:4).';

    x_bar = mean(X, 2);
    y_bar = mean(Y, 2);

    N = length(x_bar);

    C = zeros(N, N);
    for i = 1 : size(X, 2)
        C = C + (Y(:, i) - y_bar) * (X(:, i) - x_bar).';
    end

    C = 1/N * C;

    [U, W, V] = svd(C);
    if (det(U) * det(V) > 0)
        R = U * V.';
    else
        R = U * diag([ones(N - 1, 1); det(U) * det(V)])  * V.';
    end

    v = y_bar - R * x_bar;
    
    % se2 model
    SE2 = [R v;...
           zeros(1, 2) 1]; 