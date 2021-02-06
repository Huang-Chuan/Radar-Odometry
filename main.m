clear all;
addpath('robotcar-dataset-sdk-master\');
dataDir = 'G:\dataSets\2019-01-10-14-36-48-radar-oxford-10k-partial_Navtech_CTS350-X_Radar\2019-01-10-14-36-48-radar-oxford-10k-partial';
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

% ransac setup
fitFcn = @transformEstimate;
distFcn = @(model, data) (mean(((model(1:2, 1:2) * data(:, 1:2).' + ...
    model(1:2, 3)).' - data(:, 3:4)).^2, 2));
sampleSize = 3;
maxDistance = 0.2;
% 

fout = fopen('accuracy.csv','w');
fprintf(fout, 'x,y,yaw,t1,t2,x_hat,y_hat,yaw_hat\n');
% iterate to the end
for frame = 1 : num_of_frames
    fprintf('processing %d frame!!\n', frame);
    timestamp = filenames{1}{frame};
    [timestamps, azimuths, valid, fft_data, radar_resolution] = LoadRadar(sprintf('%s/radar/%s', dataDir), timestamp); 
    % landmark extraction
    % LO: detected landmark in polor form, 2D matrix of shape (2 x N)
    % LI: detected landmark in cartesian form, 2D matrix of shape (2 x N)
    [LO, LI, t] = landmarkExtract(azimuths, timestamps, valid, fft_data, coeff, z_q, radar_resolution, cart_resolution, cart_pixel_width, false, false);
    RSD = calcRSD(LO, LI, 50);
    % load(sprintf('landmarks/%s.mat', timestamp));
    if frame == 1
        % load(sprintf('landmarks/%d.mat', frame));
        landmarks1.LO = LO;
        landmarks1.t  =  t;
        landmarks1.LI = LI;
        landmarks1.RSD = RSD;
        save(sprintf('landmarks/%d.mat', frame), 'landmarks1');
    else

        % load(sprintf('landmarks/%d.mat', frame));
        landmarks2.LO  = LO;
        landmarks2.LI  = LI;
        landmarks2.t   = t;
        landmarks2.RSD = RSD;
        save(sprintf('landmarks/%d.mat', frame), 'landmarks2');
        t1 = filenames{1}{frame - 1};
        t2 = filenames{1}{frame};
        % step 1: Data assoc. with RSD on input pointclouds.
        % To do: RSD calculation result can be reused inside GenerateUnaryMatch
        %[unary_matches] = GenerateUnaryMatch(LO_1, LI_1, LO_2, LI_2, 384, 'RSD');
        [unary_matches] = GenerateUnaryMatch(landmarks1.RSD, landmarks2.RSD);
        [match_set] = DataAssociation(landmarks1.LO,  landmarks2.LO, unary_matches, alpha);
        % match_set = matchFeatures(landmarks1.RSD.', landmarks2.RSD.', 'MatchThreshold', 95);
        % match_set = match_set.';
        % step 2: align using data assoc. from step 1.
        % R: LO_1 -> LO_2
        % t: LO_2 relative to LO_1 expressed in LO_2


        landmarksA.LO = landmarks1.LO(:, match_set(1, :));
        landmarksA.t  = landmarks1.t(:, match_set(1, :));
        landmarksB.LO = landmarks2.LO(:, match_set(2, :));
        landmarksB.t  = landmarks2.t(:, match_set(2, :));
        
        
        model = MotionDistortedRansac(landmarksA, landmarksB);
        percent = model.computeModel();
        fprintf('%f of %d inliers.\n', percent, size(landmarksA.LO, 2));
        SE3 = model.getTransform((str2double(t2) - str2double(t1))/ 1e6);        
        %data = [LO_2(:, match_set(2, :)).' LO_1(:, match_set(1, :)).'];
        %[model, inlierIdx] = ransac(data, fitFcn, distFcn, sampleSize, maxDistance);
        %[R1, t1] = transformEstimate(LO_1(:, match_set(1, :)), LO_2(:, match_set(2, :)));
        %[LO_1_] = R1 * LO_1 + t1;
        % step 3: Data assoc. with NN on input pointclouds.
        %[unary_matches] = GenerateUnaryMatch(LO_1_, [], LO_2, [], 0, 'NN');
        %[match_set] = DataAssociation(LO_1_, LO_2, unary_matches, alpha);
        % step 4: Align using data assoc. from step 3.
        % R: LO_1_ -> LO_2
        % t: LO_2 relative to LO_1_ expressed in LO_2_
        %[R2, t2] = transformEstimate(LO_1(:, match_set(1, :)), LO_2(:, match_set(2, :)));
        %X(frame).R = model(1:2,1:2);
        %X(frame).t = model(1:2,3);
        % prev_landmark update
        landmarks1 = landmarks2;
        
        
        
        % prev_landmark.LO = LO;
        % prev_landmark.t  = t;
        % prev_landmark.LI = LI;
    
        % write groundtruth and odometry result to file        
        [gt_x, gt_y, gt_yaw, isFound] = get_groundtruth_odometry(dataDir, t1, t2);
        if ~isFound
            break;
        end
        T = inv(SE3);
        fprintf(fout,'%f,%f,%f,%s,%s,%f,%f,%f\n', gt_x, gt_y, gt_yaw, t1, t2, T(1, 4), T(2, 4), -asin(T(1,2)));
    
    end
    %
end
fclose(fout);