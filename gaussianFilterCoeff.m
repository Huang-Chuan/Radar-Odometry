function [coeff] = gaussianFilterCoeff(sigma_q)
    assert(mod(sigma_q, 2) == 1);
    filterSize = sigma_q * 3;
    mu = filterSize / 2;
    sigma_q_2 = sigma_q ^ 2;

    idx = 0 : filterSize - 1;
    coeff = exp(-0.5 * (idx - mu).^2 / sigma_q_2);
    % normalized coefficient    
    coeff = coeff / sum(coeff);

