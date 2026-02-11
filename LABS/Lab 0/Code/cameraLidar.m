%cameraLidar.m
clear; close all; clc;

clear node; % clear previous node handle if it exists

setenv('ROS_DOMAIN_ID','37'); % Set to your robots ROS_DOMAIN_ID (check the robot’s ID card)
setenv('ROS_LOCALHOST_ONLY', '0'); % 0 implies multi-host communication
setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'); % Middleware

% Create a unique node name for this MATLAB session
node = ros2node('matlabNode');

lidarSub = ros2subscriber(node, "/scan", "sensor_msgs/LaserScan",...
"Reliability", "besteffort", "Durability", "volatile", "Depth", 10);
camSub = ros2subscriber(node,"/image_raw/compressed","sensor_msgs/CompressedImage", ...
"Reliability","besteffort","Durability","volatile","Depth",10);

lidarMsg = receive(lidarSub, 10);
ranges = lidarMsg.ranges;

angles = lidarMsg.angle_min:lidarMsg.angle_increment:lidarMsg.angle_max;



imgMsg = receive(camSub, 10);
img  = rosReadImage(imgMsg);


imshow(img);
polarplot(ranges,angles, '.');

