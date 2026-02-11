%% Environment Setup
clear; close all; clc;

clear node; % clear previous node handle if it exists

setenv('ROS_DOMAIN_ID','43'); % Set to your robots ROS_DOMAIN_ID (check the robot’s ID card)
setenv('ROS_LOCALHOST_ONLY', '0'); % 0 implies multi-host communication
setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'); % Middleware

% Create a unique node name for this MATLAB session
node = ros2node('LiveLidar');

%% Creating the Lidar Sub
%Lidar Subscriber
lidarSub = ros2subscriber(node, "/scan", "sensor_msgs/LaserScan",...
"Reliability", "besteffort", "Durability", "volatile", "Depth", 10);

%% Getting Live Lidar and Plotting
T = 0.05;   %Sampling Period
%Creating the figure only once
figure; 
hScatter = scatter(nan,nan,'.');
hold on;
plot(0,0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Turtlebot');
legend('show');
xlabel("X Coordinate");
ylabel("Y Coordinate");
title("Environment Of The Robot");
grid on;
axis([-2 2 -2 2]);

%This loop will run each iteration that lidar data is received
while true
lidarMsg = receive(lidarSub, 10);
ranges = lidarMsg.ranges;
angles = lidarMsg.angle_min:lidarMsg.angle_increment:lidarMsg.angle_max;

%Filtering the Lidar Messages
valid = ~isnan(ranges) & ~isinf(ranges) & (ranges > 0);
ranges = ranges(valid);
angles = angles(valid);

%Extracting the x and y coordinates from distance and angle information

xCoordinates = ranges .* transpose(cos(angles));
yCoordinates = ranges .* transpose(sin(angles));

%Updating the plot
set(hScatter, 'XData', xCoordinates, 'YData', yCoordinates, 'DisplayName', 'Lidar Readings');
drawnow limitrate;   % Efficient plotting update

pause(T);   %Pause for sampling time 
end