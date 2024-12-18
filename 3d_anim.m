clear;
clc;

animation_speed_multiplier = 50;

% SMC
% data_file_name = "data/SMC_Massa1koma5kg_v2.mat";
% data_file_name = "data/SMC_Massa5kg_v2.mat";
% data_file_name = "data/SMC_Massa10kg_v2.mat";

% MPC
data_file_name = "data/trajectory_data_1koma5kg.mat";
% data_file_name = "data/trajectory_data_5kg.mat";
% data_file_name = "data/trajectory_data_10kg.mat";

% PID
% data_file_name = "data/PID_Massa1koma5kg.mat";
% data_file_name = "data/PID_Massa5kg.mat";
% data_file_name = "data/PID_Massa10kg.mat";

fig = figure('Visible', 'on'); % Create the figure
set(fig, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]); % Full screen

% set axis
myaxes = axes( ...
    'xlim', [-10, 15], ...
    'ylim', [-10, 10], ...
    'zlim', [-10, 10] ...
    );
view(3);
grid on;
axis equal;
hold on;
xlabel('x');
ylabel('y');
zlabel('z');

xmin = -2;
xmax = 11;
ymin = -2;
ymax = 6;
zmin = 0;
zmax = 1.5;

% xmin = -2;
% xmax = 2;
% ymin = -2;
% ymax = 2;
% zmin = 0;
% zmax = 1.5;


axis([xmin xmax ymin ymax zmin zmax]);
axis manual;

% Create ground plane
% Create ground plane
[X, Y] = meshgrid(xmin:0.2:xmax, ymin:0.2:ymax);
Z = zeros(size(X));
surface(X, Y, Z, 'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5);




% generate 3D model
base_radius = 0.5;
base_height = 0.5;
wheel_radius = 0.2;
wheel_width = 0.1;

[xcylinder ycylinder zcylinder] = cylinder(base_radius);
zcylinder = zcylinder * base_height; % scale height

[xwheel, ywheel, zwheel] = cylinder(wheel_radius);
zwheel = zwheel * wheel_width - wheel_width/2;

[xcone ycone zcone] = cylinder([0.1 0.0]);
[xsphere ysphere zsphere] = sphere();

% Plot the different orientation of the shapes
% Add the cylinder surface
h(1) = surface(xcylinder, ycylinder, zcylinder, ...
    'FaceColor', 'black', 'EdgeColor', 'none');

% Add top and bottom lids using `fill3`
topLid = fill3(xcylinder(2, :), ycylinder(2, :), zcylinder(2, :), 'black', 'EdgeColor', 'none'); % Top lid
bottomLid = fill3(xcylinder(1, :), ycylinder(1, :), zcylinder(1, :), 'black', 'EdgeColor', 'none'); % Bottom lid

% wheels
for i = 1:3
    wheelCoords = [xwheel(:)'; ywheel(:)'; zwheel(:)'];
    rot_y = pi/2;
    rotationY = [
        cos(rot_y), 0, sin(rot_y);
        0, 1, 0;
        -sin(rot_y), 0, cos(rot_y)
        ];
    wheelCoords = rotationY * wheelCoords;

    % rotate the wheel to align with its respective angle
    rot_z = 2*pi/3 * (i-1);
    rotationZ = [
        cos(rot_z), -sin(rot_z), 0;
        sin(rot_z), cos(rot_z), 0;
        0, 0, 1
        ];
    wheelCoords = rotationZ * wheelCoords;

    % Translate wheel into position
    xWheel = reshape(wheelCoords(1, :), size(xwheel)) + base_radius * cos(rot_z);
    yWheel = reshape(wheelCoords(2, :), size(ywheel)) + base_radius * sin(rot_z);
    zWheel = reshape(wheelCoords(3, :), size(zwheel));

    h(1+i) = surface(xWheel, yWheel, zWheel, ...
        'FaceColor', 'red', 'EdgeColor', 'none');
    h(4+i) = fill3(xWheel(2,:), yWheel(2,:), zWheel(2,:), 'red', 'EdgeColor', 'none');
    %h(7+i) = fill3(xWheel(1,:), yWheel(1,:), zWheel(1,:), 'red', 'EdgeColor', 'none');
    
end


% Group objects into hgtransform for animation
combinedobject = hgtransform('parent', myaxes);
set([h, topLid, bottomLid], 'Parent', combinedobject);
drawnow

% define the motion coordinates

load(data_file_name);

longitude = y(1:animation_speed_multiplier:end);
latitude = x(1:animation_speed_multiplier: end);
bearing = theta(1:animation_speed_multiplier:end);
height_above_ground = wheel_radius;
altitute = height_above_ground * ones(1,length(longitude));
v_x = v_x(1:animation_speed_multiplier:end);
v_y = v_y(1:animation_speed_multiplier:end);


% Plot trajectory
%plot(x,y,'o','MarkerSize',3,'Color',[0,0,0]);
trail = animatedline('LineWidth', 2, 'Color', 'blue');

% Add shadow
shadowScale = 0.8; % Scale for the shadow
shadow = fill3(shadowScale * xcylinder, shadowScale * ycylinder, ...
               0 * zcylinder, [0.2, 0.2, 0.2], 'EdgeColor', 'none');
set(shadow, 'Parent', combinedobject);

% Add lighting
light('Position', [0, 0, 10], 'Style', 'infinite');
lighting gouraud;
material shiny; % Makes the surfaces shiny
title('3D Omni-Wheel Robot Animation');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');

% Add velocity arrow
arrow_height = height_above_ground + base_height + 0.1;
max_arrow_length = 10;
arrow = quiver3(latitude(1), longitude(1), arrow_height, ...
    v_x(1), v_y(1), 0, 'r', 'LineWidth', 2, 'MaxHeadSize', max_arrow_length);
 

% Target Trajectory
% Define rectangle dimensions
traj_l = 10; % traj_l of the rectangle
traj_w = 5;   % traj_w of the rectangle

% Define points along the trajectory
numPoints = 500; % Number of points for each side

% Edge 1: Bottom (from (0,0) to (10,0))
x1 = linspace(0, traj_l, numPoints);
y1 = zeros(1, numPoints);

% Edge 2: Right (from (10,0) to (10,5))
x2 = traj_l * ones(1, numPoints);
y2 = linspace(0, traj_w, numPoints);

% Edge 3: Top (from (10,5) to (0,5))
x3 = linspace(traj_l, 0, numPoints);
y3 = traj_w * ones(1, numPoints);

% Edge 4: Left (from (0,5) to (0,0))
x4 = zeros(1, numPoints);
y4 = linspace(traj_w, 0, numPoints);

% Combine all edges to form the trajectory
traj_x = [x1, x2, x3, x4];
traj_y = [y1, y2, y3, y4];


desired_traj = plot(traj_x, traj_y,  'MarkerSize', 1, 'MarkerFaceColor', 'black'); % Starting point


legend([desired_traj, trail], {'Desired Trajectory', 'Traced Path'}, ...
       'Location', 'northeast', 'FontSize', 12);

% % Make videowriter object
% Extract the base file name without path and extension
[~, name, ~] = fileparts(data_file_name); % 'name' will be 'SMC_Massa1koma5kg_v2'

% Create the video file name
videoFileName = "result/" + name + ".mp4"; % Append .mp4 extension

%v = VideoWriter(videoFileName, 'MPEG-4'); % Set file name and format
v = VideoWriter(videoFileName, 'Motion JPEG AVI'); % Alternative format

v.FrameRate = 30; % Set frame rate (30 fps is standard)
open(v); % Open the video writer object


% Animate using makehgtform
for i = 1:length(latitude)


    translation = makehgtform('translate', ...
        [latitude(i) longitude(i) altitute(i)]); % Correct variable name

    %set(combinedobject, 'matrix', translation);

    rotation = makehgtform('zrotate', bearing(i));
    %set(combinedobject, 'matrix', rotation);

    set(combinedobject, 'matrix', ...
        translation*rotation);
    addpoints(trail, latitude(i), longitude(i), 0);
    
    % Draw velocity arrow
    set(arrow, 'XData', latitude(i), 'YData', longitude(i), 'ZData', arrow_height, ...
        'UData', v_x(i), 'VData', v_y(i), 'WData', 0);

    % capture current frame
    frame = getframe(gcf); % capture the figure
    writeVideo(v, frame); % write the frame to the video file

    %pause(0.001);
    %disp([i,latitude(i),longitude(i)]);
end

% close the video writer
close(v);
hold off;

disp(["Animation saved as", videoFileName]);

