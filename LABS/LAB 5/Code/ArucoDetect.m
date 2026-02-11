%% Environment Setup
clc; clear; close all;
clear node; % clear previous node handle if it exists

setenv('ROS_DOMAIN_ID','37'); % Set to your robots ROS_DOMAIN_ID (check the robot"s ID card)
setenv('ROS_LOCALHOST_ONLY', '0'); % 0 implies multi-host communication
setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'); % Middleware

% Create a unique node name for this MATLAB session
node = ros2node('ArucoDetect');
%% Subscribers And Publishers
imgSub = ros2subscriber(node,"/image_raw/compressed","sensor_msgs/CompressedImage", ...
"Reliability","besteffort","Durability","volatile","Depth",10);
%% Visualization
% SETUP (Before Loop)
figure("Name","ArucoDetect");
hIm = imshow(zeros(480, 640, "uint8")); hold on;
hPlot2D = plot(0, 0, "g-", "LineWidth", 3);
receive(imgSub, 10);

%Update The Visualizatiion
while true
    %% Read Image
    imgMsg = receive(imgSub, 10);
    I = rosReadImage(imgMsg);
    
    %% Detect Markers
    [ids, locs] = readArucoMarker(I, "DICT_5X5_250");
    set(hIm, "CData", I);
    %% Update The Plot
    if ~isempty(ids)
        
        corners = locs(:,:,1);
        boxX = [corners(:,1); corners(1,1)];
        boxY = [corners(:,2); corners(1,2)];
        set(hPlot2D, "XData", boxX, "YData", boxY, "Visible", "on");
    else
        set(hPlot2D, "Visible", "off");
    end

end