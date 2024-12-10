clear;
clc;

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
topLid = fill3(xcylinder(2, :), ycylinder(2, :), zcylinder(2, :), 'blue', 'EdgeColor', 'none'); % Top lid
bottomLid = fill3(xcylinder(1, :), ycylinder(1, :), zcylinder(1, :), 'blue', 'EdgeColor', 'none'); % Bottom lid

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


%load("MPC Mat Data/trajectory_data_5kg.mat");
load("SMC_Massa10kg.mat");

longitude = y;
latitude = x;

height_above_ground = wheel_radius;
altitute = height_above_ground * ones(1,length(x));
bearing = theta;

plot(x, y)
% Animate using makehgtform
for i = 1:length(latitude)


    translation = makehgtform('translate', ...
        [latitude(i) longitude(i) altitute(i)]); % Correct variable name

    %set(combinedobject, 'matrix', translation);

    rotation = makehgtform('zrotate', bearing(i));
    %set(combinedobject, 'matrix', rotation);

    set(combinedobject, 'matrix', ...
        translation*rotation);

    pause(0.01);
    disp([i,latitude(i),longitude(i)]);
end