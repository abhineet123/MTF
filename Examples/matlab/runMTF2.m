function runMTF2(varargin)

close all;

% script parameters
show_window = 0;

% MTF parameters
config_dir = '../../Config';
db_root_path = '../../../Datasets';
pipeline = 'c';
img_source = 'j';
actor_id = 1;
seq_id = 16;
seq_fmt = 'jpg';
init_frame_id = 1;

param_str = sprintf('db_root_path %s', db_root_path);
param_str = sprintf('%s pipeline %s', param_str, pipeline);
param_str = sprintf('%s img_source %s', param_str, img_source);
param_str = sprintf('%s actor_id %d', param_str, actor_id);
param_str = sprintf('%s seq_id %d', param_str, seq_id);
param_str = sprintf('%s seq_fmt %s', param_str, seq_fmt);
param_str = sprintf('%s init_frame_id %d', param_str, init_frame_id);
if mod(nargin, 2) ~= 0
    error('Optional arguments must be specified in pairs');
end
% parse optional arguments
arg_id = 1;
while arg_id <= nargin    
    arg_name = varargin{arg_id};
    arg_val = varargin{arg_id + 1};
    if strcmp(arg_name, 'config_dir')
        config_dir = arg_val;
    elseif strcmp(arg_name, 'show_window')
        show_window = arg_val;
    else
        param_str = sprintf('%s %s %s', param_str,...
            string(arg_name), string(arg_val));
    end
    arg_id = arg_id + 2;
end   
param_str = sprintf('config_dir %s %s', config_dir, param_str);

[input_id, n_frames] = mexMTF2('create_input', param_str);
if input_id == 0
    error('MTF input pipeline creation was unsuccessful');
else
    fprintf('MTF input pipeline created successfully');
    if n_frames > 0
        fprintf(' with %d frames', n_frames);
    end
    fprintf('\n');
end

tracker_id = mexMTF2('create_tracker', param_str);
if ~tracker_id
    error('Tracker creation was unsuccessful');
else
    fprintf('Tracker created successfully\n');
end

% pause
if show_window
    h = figure; 
end
while 1
    [success, curr_img] = mexMTF2('get_frame', input_id);
    if ~success
        error('Frame extraction was unsuccessful');
    end 
    [success, curr_corners] = mexMTF2('get_region', tracker_id);
    if ~success
        error('Tracker region extraction was unsuccessful');
    end
	if show_window
        if ~ ishghandle(h)
            break;
        end
		imshow(curr_img);
		hold on;
		plot([curr_corners(1, :),curr_corners(1, 1)],...
            [curr_corners(2, :), curr_corners(2, 1)],...
            'Color','r', 'LineWidth',2);
		hold off;
		pause(0.001);
	end
end
mexMTF2('clear');
