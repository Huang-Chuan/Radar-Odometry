addpath('robotcar-dataset-sdk-master\');
dataDir = 'G:\dataSets\2019-01-10-14-36-48-radar-oxford-10k-partial_Navtech_CTS350-X_Radar\2019-01-10-14-36-48-radar-oxford-10k-partial';
timestampFile = fopen(sprintf('%s/radar.timestamps', dataDir), 'r');
formatSpec = '%u64 %s';
filenames = textscan(timestampFile, formatSpec);
fclose(timestampFile);


cart_resolution = 0.3456;
cart_pixel_width = 722;

num_of_frames = length(filenames{1});

frame = 82;
timestamp1 = filenames{1}(frame);
[~, azimuths, valid, fft_data1, radar_resolution] = LoadRadar(sprintf('%s/radar', dataDir), timestamp1);
det1 = load(sprintf('landmarks/%d.mat', frame));
cart_img1 = RadarPolarToCartesian(azimuths, fft_data1, radar_resolution, cart_resolution, cart_pixel_width, 0);
subplot(1,2,1);
imshow(cart_img1);
hold on;
X = floor(det1.landmarks2.LO(2, :) / cart_resolution) + 361;
Y =  -floor(det1.landmarks2.LO(1, :) / cart_resolution) + 361;
plot(X, Y, '.r');
idx1 = find(X==351&Y==516);

timestamp2 = filenames{1}(frame + 1);
[~, azimuths, ~, fft_data2, radar_resolution] = LoadRadar(sprintf('%s/radar', dataDir), timestamp2);
det2 = load(sprintf('landmarks/%d.mat', frame + 1));
cart_img2 = RadarPolarToCartesian(azimuths, fft_data2, radar_resolution, cart_resolution, cart_pixel_width, 0);
subplot(1,2,2);
imshow(cart_img2);
hold on;
X = floor(det2.landmarks2.LO(2, :) / cart_resolution) + 361;
Y =  -floor(det2.landmarks2.LO(1, :) / cart_resolution) + 361;
plot(X, Y, '.r');
plot(X, Y, '.r');
idx2 = find(X==350&Y==523);

figure(2);
subplot(2,1,1);
plot(det1.landmarks2.RSD(:, idx1));
subplot(2,1,2);
plot(det2.landmarks2.RSD(:, idx2));
