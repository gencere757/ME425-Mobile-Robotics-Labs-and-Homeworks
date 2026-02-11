%% Environment Setup
clear; close all; clc;

clear node; % clear previous node handle if it exists

setenv('ROS_DOMAIN_ID','43'); % Set to your robots ROS_DOMAIN_ID (check the robot’s ID card)
setenv('ROS_LOCALHOST_ONLY', '0'); % 0 implies multi-host communication
setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'); % Middleware

% Create a unique node name for this MATLAB session
node = ros2node('Lidar');

%% Creating the Lidar Sub
%Lidar Subscriber
lidarSub = ros2subscriber(node, "/scan", "sensor_msgs/LaserScan",...
"Reliability", "besteffort", "Durability", "volatile", "Depth", 10);

%% Recieving the lidar message and extracting the distance and angle
%information
lidarMsg = receive(lidarSub, 10);
ranges = lidarMsg.ranges;
angles = lidarMsg.angle_min:lidarMsg.angle_increment:lidarMsg.angle_max;

%% Extracting the x and y coordinates from distance and angle information
xCoordinates = ranges .* transpose(cos(angles));
yCoordinates = ranges .* transpose(sin(angles));

%% Plotting
figure;
plot(xCoordinates,yCoordinates, '.','DisplayName', 'Lidar Readings');
hold on;
plot(0,0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Turtlebot');
legend('show');
xlabel("X Coordinate");
ylabel("Y Coordinate");
title("Environment Of The Robot");
grid on;
axis([-2 2 -2 2]);