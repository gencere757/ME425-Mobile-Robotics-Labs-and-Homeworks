%% Environment Setup
clc; clear; close all;
clear node; % clear previous node handle if it exists

setenv('ROS_DOMAIN_ID','37'); % Set to your robots ROS_DOMAIN_ID (check the robot"s ID card)
setenv('ROS_LOCALHOST_ONLY', '0'); % 0 implies multi-host communication
setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'); % Middleware

% Create a unique node name for this MATLAB session
node = ros2node('ArucoRotate');
%% Subscribers And Publishers
imgSub = ros2subscriber(node,"/image_raw/compressed","sensor_msgs/CompressedImage", ...
"Reliability","besteffort","Durability","volatile","Depth",10);

velPub = ros2publisher(node,"/cmd_vel","geometry_msgs/Twist", ...
"Reliability","reliable","Durability","volatile","Depth",10);
velMsg = ros2message(velPub);

%% Visualization SETUP (Before Loop)
hIm = imshow(zeros(480, 640, "uint8")); hold on;
hPlot2D = plot(0, 0, "g-", "LineWidth", 3);
receive(imgSub, 10);
%% Main Loop That Will Run Indefinetely
while true 
     %% Read Image
    imgMsg = receive(imgSub, 10);
    I = rosReadImage(imgMsg);
    %% Detect Markers
    [ids, locs] = readArucoMarker(I, "DICT_5X5_250");
    %If marker found
    set(hIm, "CData", I);
    if ~isempty(ids)
        velMsg.angular.z = 0;
        send(velPub, velMsg);
        corners = locs(:,:,1);
        boxX = [corners(:,1); corners(1,1)];
        boxY = [corners(:,2); corners(1,2)];
        set(hPlot2D, "XData", boxX, "YData", boxY, "Visible", "on");
    %If Marker Not Found
    else
        set(hPlot2D, "Visible", "off");
        velMsg.angular.z = 0.25;
        send(velPub, velMsg);
    end
end