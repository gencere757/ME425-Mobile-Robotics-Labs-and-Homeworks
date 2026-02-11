%% Environment Setup
clear node; % clear previous node handle if it exists

setenv('ROS_DOMAIN_ID','36'); % Set to your robots ROS_DOMAIN_ID (check the robot"s ID card)
setenv('ROS_LOCALHOST_ONLY', '0'); % 0 implies multi-host communication
setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'); % Middleware

% Create a unique node name for this MATLAB session
node = ros2node('ArucoTrack');
%% Subscribers And Publishers
imgSub = ros2subscriber(node,"/image_raw/compressed","sensor_msgs/CompressedImage", ...
"Reliability","besteffort","Durability","volatile","Depth",10);

velPub = ros2publisher(node,"/cmd_vel","geometry_msgs/Twist", ...
"Reliability","reliable","Durability","volatile","Depth",10);
velMsg = ros2message(velPub);

%% Defining Constant Parameters and Specifications
markerSize = 0.1;

%The Corners Positions In The World Frame
worldPoints = [-markerSize/2 -markerSize/2 0;
    markerSize/2 -markerSize/2 0;
    markerSize markerSize 0;
    -markerSize/2 markerSize/2 0];

%Desired Stopping Distance
d_desired = 0.2;

%% The Control Parameters
k_lin = 1.5;
k_ang = 0.75;

%% Visualization SETUP (Before Loop)
hIm = imshow(zeros(480, 640, "uint8")); hold on;
hPlot2D = plot(0, 0, "g-", "LineWidth", 3);
receive(imgSub, 20);

%% The Error Data Vectors Initialization
linearErrors = [];
angularErrors = [];

%% The Program Loop
while true
     %% Read Image
    imgMsg = receive(imgSub, 20);
    I = rosReadImage(imgMsg);
    set(hIm, "CData", I);
    
    % Detect ArUco markers
    [ids, locs] = readArucoMarker(I, 'DICT_5X5_250');
    %locs = transpose(locs);
    % If No Marker Detected
    if isempty(ids)
        % STATE: SEARCHING - rotate in place
        velMsg.linear.x = 0;
        velMsg.angular.z = 0.3; % constant rotation speed
        send(velPub, velMsg);
        set(hPlot2D, "Visible", "off"); %Update Plot
    % If Marker Found
    else    
        % STATE: TRACKING - control to chase marker
        %% Get the Rotation Matrix and Translation Vector
        [rotMatrix, tVector] = extrinsics(locs, worldPoints(:,1:2), cameraParams);
        x_R = tVector(3);
        y_R = -tVector(1);

        %% Compute The Errors
        e_theta = atan2(y_R,x_R);
        e_d = x_R - d_desired;
        angularErrors = [angularErrors e_theta];
        linearErrors = [linearErrors e_d];

        %% Compute The Control Inputs
        v = k_lin * e_d;
        w = k_ang * e_theta;

        %Anti Reverse Logic
        if e_d < 0.05
            v = 0;
        end
        if e_theta < 0.05
            w = 0;
        end
        %% Assign the Velocities
        velMsg.linear.x = v;
        velMsg.angular.z = w;
        send(velPub, velMsg);

        %% Plotting The Picture
        corners = locs(:,:,1);
        boxX = [corners(:,1); corners(1,1)];
        boxY = [corners(:,2); corners(1,2)];
        set(hPlot2D, "XData", boxX, "YData", boxY, "Visible", "on");
    end
end

