function [L_O, L_I, t] = landmarkExtract(azimuths, timestamps, valid, fft_data, binomialFilterCoeff, z_q, radar_resolution, cart_resolution, cart_pixel_width, isDebug, isMR)

    % DataAssociation - convert a UNIX timestamp to a human-readable string
    %
    % [date_time] = TimestampToString(t)
    %
    % INPUTS:
    %   LO_1: size(2 * N), each point  
    %   LI_1: size(2 * N),
    %   LO_2: size(2 * M),
    %   LI_2: size(2 * M),
    % OUTPUTS:
    %   unary_matches: string representing t in the format yyyy/mm/dd HH:MM:SS.FFF
    %   match_set: 2 *  

    num_az_bins = length(azimuths);
    num_rng_bins = size(fft_data, 2);
    L_O = [];
    L_I = [];
      t = [];

    polar_binary = zeros(size(fft_data));
    

    
    start_rng_bin = floor(1 / radar_resolution);
    for i = 1 : num_az_bins
        rangePower = fft_data(i, :);
        %medFilteredSignal = medfilt1(rangePower, medFilter_w);
        mean_power = mean(rangePower);
        q = rangePower - mean_power;
        p = conv(rangePower, binomialFilterCoeff, 'same');
        Q = q(q < 0);
        if (length(Q) > 0)
            sigma_q_2 = mean(2 * Q.^2);
        else
            sigma_q_2 = 0.034^2;
        end
        y_hat = zeros(num_rng_bins, 1);
        for ii = 1 : num_rng_bins
            if (q(ii) > 0)
                %y = p(ii) * (1 - pdf('Normal', p(ii), 0, sqrt(sigma_q_2))/pdf('Normal', 0, 0, sqrt(sigma_q_2)));
                y = p(ii) * (1 - exp(-p(ii)^2/sigma_q_2));
                y = y + (q(ii) - p(ii)) * (1 - exp(-(q(ii) - p(ii))^2/sigma_q_2));
                if (y > z_q * sqrt(sigma_q_2))
                    y_hat(ii) = y;
                end
            end
        end

        if(isDebug)
            figure(1);
            plot(rangePower);
            figure(2);
            plot(q);
            figure(3);
            plot(p);
            figure(4);
            plot(y_hat);
        end

        for ii = start_rng_bin : num_rng_bins - 1
            if(y_hat(ii - 1) == 0) && (y_hat(ii) > 0) && (y_hat(ii + 1) > 0)
                a = azimuths(i);
                r = (ii - 0.5) * radar_resolution;
                L_O(:, end + 1) = [r * cos(a); r * sin(a)];
                  t(:, end + 1) = timestamps(i);
                polar_binary(i, ii) = 1;
            end
        end

        if isMR
            x=1;% multi-path removal
        end

    end


    [cart_binary, Y, X] = PolarBinary2CartBinary(azimuths, polar_binary, ...
    radar_resolution, cart_resolution, cart_pixel_width, true);


    k = find(cart_binary(:) > 0);
    L_I = [X(k) Y(k)].';
    
%     plot(L_O_c(2,:).*cos(L_O(1,:)),L_O(2,:).*sin(L_O(1,:)),'.r');
%     hold on;
%     plot(L_I(1,:),L_I(2,:),'.g');
%     x = 1;
    
    
    
    