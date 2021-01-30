function [unary_matches] = GenerateUnaryMatch(LO_1, LI_1, LO_2, LI_2, num_az_bins, criterion)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    N = size(LO_1, 2);
    unary_matches = zeros(2, N);
    if strcmp(criterion, 'RSD')
        % RSD1, RSD2: 3 x num_az_bins x number of detected points 
        RSD1 = calcRSD(LO_1, LI_1, num_az_bins, 70, false);
        RSD2 = calcRSD(LO_2, LI_2, num_az_bins, 70, false);
        %fprintf('Generating unary matches by RSD......\n');
        for i = 1 : N
            [~, j] = min(vecnorm(RSD1(:, i) - RSD2));
            unary_matches(:, i) = [i; j];
        end

    elseif strcmp(criterion, 'NN')
        %fprintf('Generating unary matches by NN......\n');
        for i = 1 : N
            [~, j] = min(vecnorm(LO_1(:, i) - LO_2));
            unary_matches(:, i) = [i; j];
        end
    else
        error('wrong criterion!!\n');
    end
    
    %fprintf('Generating unary matches complete!!\n');

