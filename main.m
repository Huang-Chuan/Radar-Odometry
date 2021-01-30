clear all;
addpath('robotcar-dataset-sdk-master\');
dataDir = 'oxford_robotcar-dataset_tiny';
timestampFile = fopen(sprintf('%s/radar.timestamps', dataDir), 'r');
%
formatSpec = '%s %s';
filenames = textscan(timestampFile, formatSpec);
fclose(timestampFile);

num_of_frames = length(filenames{1});

prev_landmark = struct('L_O', [], 'L_I', []);
% binomial filter coefficient
coeff=gaussianFilterCoeff(17);
cart_resolution = 0.3456;
cart_pixel_width = 722;
medianFilter_w = 200;
z_q = 4.0;
alpha = 0.5;
beta = 0.25;

X = struct('R',eye(2),'t', zeros(2, 1));

% ransac parameters
fitFcn = @transformEstimate;
distFcn = @(model, data) (mean(((model(1:2, 1:2) * data(:, 1:2).' + ...
    model(1:2, 3)).' - data(:, 3:4)).^2, 2));
sampleSize = 3;
maxDistance = 0.2;

fout = fopen('accuracy.csv','w');
fprintf(fout, 'x,y,yaw,t1,t2,x_hat,y_hat,yaw_hat\n');
% iterate to the end
for frame = 1 : num_of_frames
    fprintf('processing %d frame!!\n', frame);
    timestamp = filenames{1}{frame};
    [timestamps, azimuths, valid, fft_data, radar_resolution] = LoadRadar(sprintf('%s/radar/%s', dataDir), timestamp); 
    % landmark extraction
    % L_O: detected landmark in polor form, 2D matrix of shape (2 x N)
    % L_I: detected landmark in cartesian form, 2D matrix of shape (2 x N)
    [LO, LI] = landmarkExtract(azimuths, valid, fft_data, coeff, medianFilter_w, z_q, radar_resolution, cart_resolution, cart_pixel_width, false, false);
    % load(sprintf('landmarks/%s.mat', timestamp));
    if frame == 1
        prev_landmark.LO = LO;
        prev_landmark.LI = LI; 
    else
        LO_1 = prev_landmark.LO;
        LO_2 = LO;
        LI_1 = prev_landmark.LI;
        LI_2 = LI;
        %fprintf("%d landmarks in scan 1,\n%d landmarks in scan 2\n", size(LO_1, 2), size(LO_2, 2));
        % step 1: Data assoc. with RSD on input pointclouds.
        [unary_matches] = GenerateUnaryMatch(LO_1, LI_1, LO_2, LI_2, 384, 'RSD');
        [match_set] = DataAssociation(LO_1,  LO_2, unary_matches, alpha);
        % step 2: align using data assoc. from step 1.
        % R: LO_1 -> LO_2
        % t: LO_2 relative to LO_1 expressed in LO_2
        data = [LO_2(:, match_set(2, :)).' LO_1(:, match_set(1, :)).'];
        [model, inlierIdx] = ransac(data, fitFcn, distFcn, sampleSize, maxDistance);
        %[R1, t1] = transformEstimate(LO_1(:, match_set(1, :)), LO_2(:, match_set(2, :)));
        %[LO_1_] = R1 * LO_1 + t1;
        % step 3: Data assoc. with NN on input pointclouds.
        %[unary_matches] = GenerateUnaryMatch(LO_1_, [], LO_2, [], 0, 'NN');
        %[match_set] = DataAssociation(LO_1_, LO_2, unary_matches, alpha);
        % step 4: Align using data assoc. from step 3.
        % R: LO_1_ -> LO_2
        % t: LO_2 relative to LO_1_ expressed in LO_2_
        %[R2, t2] = transformEstimate(LO_1(:, match_set(1, :)), LO_2(:, match_set(2, :)));
        X(frame).R = model(1:2,1:2);
        X(frame).t = model(1:2,3);
        % prev_landmark update
        prev_landmark.LO = LO;
        prev_landmark.LI = LI;

        % write groundtruth and odometry result to file        
        t1 = filenames{1}{frame - 1};
        t2 = filenames{1}{frame};
        [gt_x, gt_y, gt_yaw, isFound] = get_groundtruth_odometry(dataDir, t1, t2);
        if ~isFound
            break;
        end
        fprintf(fout,'%f,%f,%f,%s,%s,%f,%f,%f\n', gt_x, gt_y, gt_yaw, t1, t2, X(frame).t(1), X(frame).t(2), asin(X(frame).R(2, 1)));
    
    end
    %
end
fclose(fout);