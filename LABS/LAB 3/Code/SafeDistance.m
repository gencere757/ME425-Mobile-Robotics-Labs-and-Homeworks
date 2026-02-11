%% Environment Setup
clear; close all; clc;

clear node; % clear previous node handle if it exists

setenv('ROS_DOMAIN_ID','43'); % Set to your robots ROS_DOMAIN_ID (check the robot’s ID card)
setenv('ROS_LOCALHOST_ONLY', '0'); % 0 implies multi-host communication
setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'); % Middleware

% Create a unique node name for this MATLAB session
node = ros2node('DistanceChecker');

%% Creating the Lidar and velocity Sub
%Lidar Subscriber
lidarSub = ros2subscriber(node, "/scan", "sensor_msgs/LaserScan",...
"Reliability", "besteffort", "Durability", "volatile", "Depth", 10);

velPub = ros2publisher(node,"/cmd_vel","geometry_msgs/Twist", ...
"Reliability","reliable","Durability","volatile","Depth",10);


%% Creating Lidar Plot
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
%% Running the loop for object detection
T = 0.05;   %Sampling Period
timeWaited = 0; %The amount of time waited on current collision


velMsg = ros2message(velPub);
velMsg.linear.x = 0.1;
send(velPub, velMsg);
while true
    %Getting the Lidar Message
    lidarMsg = receive(lidarSub, 10);
    ranges = lidarMsg.ranges;
    angles = lidarMsg.angle_min:lidarMsg.angle_increment:lidarMsg.angle_max;
    
    %Filtering the Lidar Messages
    valid = ~isnan(ranges) & ~isinf(ranges) & (ranges > 0);
    ranges = ranges(valid);
    angles = angles(valid);
    
    %Getting the data that is in front of the robot
    size = length(angles);
    oneEigth = floor(size/8);
    forwardDistances = [ranges(1:oneEigth) ranges(end-oneEigth+1:end)];

    %Extracting the x and y coordinates from distance and angle information

    xCoordinates = ranges .* transpose(cos(angles));
    yCoordinates = ranges .* transpose(sin(angles));

    %Updating the plot
    set(hScatter, 'XData', xCoordinates, 'YData', yCoordinates, 'DisplayName', 'Lidar Readings');
    drawnow limitrate;   % Efficient plotting update
    
    %Collision Check
    if any(forwardDistances < 0.55) %If any object that is too close is detected
        velMsg.linear.x = 0;
        send(velPub, velMsg);
        timeWaited = timeWaited + T;
        if timeWaited >=1.8   %If the robot was idle for at least 5 seconds
            %Rotate The Robot Here
            velMsg.linear.x = 0;
            velMsg.angular.z = 1;
            send(velPub,velMsg);
            pause(pi/2);
            velMsg.angular.z = 0;
            velMsg.linear.x = 0.1;
            send(velPub, velMsg);
            timeWaited = 0;
        end
    elseif timeWaited ~= 0 %If the robot has been waiting for some time
        timeWaited = 0; %Reset the counter
        %Resend the message
        velMsg.linear.x = 0.1;
        send(velPub, velMsg);
    end

    pause(T);   %Wait for the sampling period
end