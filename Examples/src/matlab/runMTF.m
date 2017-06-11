close all;
clear all;
if isunix
%     addpath('/home/abhineet/E/UofA/Thesis/Code/TrackingFramework/Matlab/');
%     addpath('/home/abhineet/E/UofA/Thesis/Code/TrackingFramework/C++/MTF/Build/Release/');
else
    addpath('E:\UofA\Thesis\Code\TrackingFramework\Matlab');
%     addpath('E:\UofA\Thesis\Code\TrackingFramework\C++\MTF\Build\Release');
end
datasets;
actor_id = 0;
seq_id = 0;
img_source = 'j';
seq_fmt = 'jpg';
db_root_path = '/home/abhineet/E/UofA/Thesis/Code/Datasets';
init_frame_id = 1;
actor = actors{actor_id+1};
seq_name = sequences{actor_id + 1}{seq_id + 1};
seq_path = sprintf('%s/%s/%s', db_root_path, actor, seq_name);
img_files = dir([seq_path, sprintf('/*.%s', seq_fmt)]);
n_frames = length(img_files(not([img_files.isdir])));
init_img_path = sprintf('%s/frame%05d.%s', seq_path, init_frame_id, seq_fmt);
init_img = imread(init_img_path);
success = mexMTF('create', '/home/abhineet/E/UofA/Thesis/Code/TrackingFramework/C++/MTF/Config/');
if ~success
    error('Tracker creation was unsuccessful');
else
    fprintf('Tracker created successfully\n');
end
% pause;

gt_path = sprintf('%s.txt', seq_path);
gt_struct = importdata(gt_path);
gt_data = gt_struct.data;
gt_corners = zeros(2, 4);
gt_corners(1, :) = gt_data(init_frame_id, [1, 3, 5, 7]);
gt_corners(2, :) = gt_data(init_frame_id, [2, 4, 6, 8]);
init_img_gray = rgb2gray(init_img);
[success, init_corners] = mexMTF('initialize', uint8(init_img_gray), double(gt_corners));
if ~success
    error('Tracker initialization was unsuccessful');
else
    fprintf('Tracker initialized successfully\n');
end
% pause
for frame_id = init_frame_id + 1:n_frames
    img_path = sprintf('%s/frame%05d.%s', seq_path, frame_id, seq_fmt);
    curr_img = imread(img_path);    
    curr_img_gray = rgb2gray(curr_img);
    gt_corners(1, :) = gt_data(frame_id, [1, 3, 5, 7]);
    gt_corners(2, :) = gt_data(frame_id, [2, 4, 6, 8]);
    % start_t = tic;
    [success, curr_corners] = mexMTF('update', uint8(curr_img_gray));
    % time_passed = toc(start_t);
    % fps = 1.0 /double(time_passed);
    % fprintf('time_passed: %f\t fps: %f\n',time_passed, fps);
    if ~success
        error('Tracker update was unsuccessful');
    end
    imshow(curr_img);
    hold on;
    plot(gt_data(frame_id, [1, 3, 5, 7, 1]), gt_data(frame_id, [2, 4, 6, 8, 2]), 'Color','g', 'LineWidth',2);
    plot([curr_corners(1, :),curr_corners(1, 1)], [curr_corners(2, :), curr_corners(2, 1)], 'Color','r', 'LineWidth',2);
    hold off;
    pause(0.001);
end
