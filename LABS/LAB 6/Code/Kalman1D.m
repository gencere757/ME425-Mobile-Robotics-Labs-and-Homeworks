clc; clear; close all;
%% REAL - TIME PLOTTING SETUP
x_values = [];
z_values = [];
P_values = [];
K_values = [];
t_values = [];
figure;
subplot(2,2,1);
xGraph = plot(nan,nan);
title("X Values");
subplot(2,2,2);
zGraph = plot(nan,nan);
title("Measurements");
subplot(2,2,3);
PGraph = plot(nan,nan);
title("Covariances");
subplot(2,2,4);
KGraph = plot(nan,nan);
title("Kalman Gains");

clear node; % clear previous node handle if it exists

setenv('ROS_DOMAIN_ID','43'); % Set to your robots ROS_DOMAIN_ID (check the robot"s ID card)
setenv('ROS_LOCALHOST_ONLY', '0'); % 0 implies multi-host communication
setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'); % Middleware

% Create a unique node name for this MATLAB session
node = ros2node('Kalman1D');
%% Subscribers And Publishers

%Lidar Subscriber
scanSub = ros2subscriber(node, "/scan", "sensor_msgs/LaserScan",...
"Reliability", "besteffort", "Durability", "volatile", "Depth", 10);

%Velocity Publisher
velPub = ros2publisher(node,"/cmd_vel","geometry_msgs/Twist", ...
"Reliability","reliable","Durability","volatile","Depth",10);

velMsg = ros2message(velPub);

%% Variables
Q = 1;  %Model Variance
R = 0.001;   %Measurement Variance

%Initial Measurement
scanMsg = receive(scanSub, 10) ;
idx = round((pi /2 - scanMsg.angle_min) / scanMsg.angle_increment) + 1;
old_z = scanMsg.ranges(max(1, min( idx, length(scanMsg.ranges ))));
old_x_k = 0.3;
old_P = 1;

%% MAIN LOOP
timer = tic; duration = 8;
while toc( timer ) < duration
% Forward Backward motion
currentTime = toc( timer ) ;
if mod(currentTime, 4) < 2 , v = 0.1; else , v = -0.1; end
velMsg.linear.x = v; send(velPub, velMsg);
% Get Measurements
scanMsg = receive ( scanSub , 10) ;
idx = round (( pi /2 - scanMsg.angle_min ) / scanMsg.angle_increment ) +1;
z = scanMsg.ranges(max(1, min(idx, length(scanMsg.ranges))));
if isnan(z)
    z = old_z;
end
% Kalman Filter
x_k_before = old_x_k;
P_before = old_P + Q;
kalman_gain = P_before/(P_before+R);
x_k_after = x_k_before + kalman_gain * (z - x_k_before);
P_after = (1-kalman_gain) * P_before;
% Update Data and Plots
old_x_k = x_k_after;
old_P = P_after;
old_z = z;

x_values = [x_values x_k_after];
z_values = [z_values z];
P_values = [P_values P_after];
K_values = [K_values kalman_gain];
t_values = [t_values toc( timer )];
set(xGraph,"XData",t_values,"YData",x_values);
set(zGraph,"XData",t_values,"YData",z_values);
set(PGraph,"XData",t_values,"YData",P_values);
set(KGraph,"XData",t_values,"YData",K_values);

drawnow limitrate ;
end

%Stop Robot
velMsg.linear.x = 0; send(velPub, velMsg);