function [gt_x, gt_y, gt_yaw, isFound] = get_groundtruth_odometry(dataDir, t1, t2)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    %dataDir = 'D:/oxford_radar_robotcar_dataset_sample_tiny';
    gt_x = 0;
    gt_y = 0;
    gt_yaw = 0;
    isFound = false;
    f = fopen(sprintf('%s/gt/radar_odometry.csv', dataDir), 'r');
    %f = fopen(gtFile, 'r');
    if(f==-1)
        error('Cannot open file\n');
    end
    tline = fgetl(f);
    while(~feof(f))
        tline = split(fgetl(f), ',');
        if(strcmp(tline{10}, t1) && strcmp(tline{9}, t2))
            gt_x = str2num(tline{3});
            gt_y = str2num(tline{4});
            gt_yaw = str2num(tline{8});
            isFound = true;
            break;
        end
    end
    fclose(f);
    
end

