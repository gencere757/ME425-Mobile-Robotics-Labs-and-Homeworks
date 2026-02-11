%% Environment Setup
clear; close all; clc;

clear node; % clear previous node handle if it exists

setenv('ROS_DOMAIN_ID','39'); % Set to your robots ROS_DOMAIN_ID (check the robot’s ID card)
setenv('ROS_LOCALHOST_ONLY', '0'); % 0 implies multi-host communication
setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'); % Middleware

% Create a unique node name for this MATLAB session
node = ros2node('TTC');

%% Subscribers And Publishers
%Setup /cmd vel publisher
velPub = ros2publisher(node,"/cmd_vel","geometry_msgs/Twist", ...
"Reliability","reliable","Durability","volatile","Depth",10);

velMsg = ros2message(velPub);

%Setup /image raw/compressed subscriber
imgSub = ros2subscriber(node,"/image_raw/compressed","sensor_msgs/CompressedImage", ...
"Reliability","besteffort","Durability","volatile","Depth",10);

%% The Program
deltaT = 0.01;
exeTime = 100;
t0 = tic;
TTCArray = [];

getImgFindCorners;
while toc(t0) < exeTime
    while objectDetected == false  %If no object is detected
    getImgFindCorners;
    send(velPub, velMsg);
    pause(1/25);
    end

    while objectDetected == true
    getImgFindCorners;
    %Assign the calculated length to L1
    l1 = max(Y) - min(Y);
    pause(deltaT);
    getImgFindCorners;
    %Assign the calculated length to L2
    l2 = max(Y) - min(Y);
    calculateTTCandMove;
    end
end

%Stop motors;
velMsg.linear.x = 0;
velMsg.angular.z = 0;
send(velPub, velMsg);

%% Plotting TTC
figure;
plot(TTCArray);
title("TTC Values For Each TTC Calculation");
xlabel("Iteration");
ylabel("TTC");
grid on;
