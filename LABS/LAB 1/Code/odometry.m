clc; close all; clear;

clear node; % clear previous node handle if it exists

% Environment Setup
setenv('ROS_DOMAIN_ID','45'); % Set to your robot's ROS_DOMAIN_ID
setenv('ROS_LOCALHOST_ONLY', '0'); % 0 implies multi-host communication
setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'); % Middleware

node = ros2node('odometry');  % Create node

% Publishers & Subscribers
velPub = ros2publisher(node,"/cmd_vel","geometry_msgs/Twist", ...
"Reliability","reliable","Durability","volatile","Depth",10);

velMsg = ros2message(velPub);

encSub = ros2subscriber(node,"/joint_states","sensor_msgs/JointState", ...
"Reliability","besteffort","Durability","volatile","Depth",10);

% Initial Encoder Reading
encMsg = receive(encSub,10);
jointNames = string(encMsg.name);
idxL = find(jointNames=="wheel_left_joint" | jointNames=="left_wheel",1);
idxR = find(jointNames=="wheel_right_joint" | jointNames=="right_wheel",1);

% Parameters Initialization
exeTime = 3;
period = 0.01;
R = 0.033;
L = 0.287;

% Vectors Initialization
xPositionsForwardHigh = [0];
yPositionsForwardHigh = [0];
rotationAnglesForwardHigh = [0];
xPositionsForwardLow = [0];
yPositionsForwardLow = [0];
rotationAnglesForwardLow = [0];
xPositionsBackwardHigh = [0];
yPositionsBackwardHigh = [0];
rotationAnglesBackwardHigh = [0];
xPositionsBackwardLow = [0];
yPositionsBackwardLow = [0];
rotationAnglesBackwardLow = [0];
xPositionsRotationRight = [0];
yPositionsRotationRight = [0];
rotationAnglesRotationRight = [0];
xPositionsRotationLeft = [0];
yPositionsRotationLeft = [0];
rotationAnglesRotationLeft = [0];
xPositionsMixed = [0];
yPositionsMixed = [0];
rotationAnglesMixed = [0];

% Forward Motion High Speed

velMsg.linear.x = 0.2;
velMsg.angular.z = 0;
send(velPub, velMsg);

encMsg = receive(encSub,10);
oldLeftWheelPos = encMsg.position(idxL);
oldRightWheelPos = encMsg.position(idxR);
x = 0; y = 0; theta = 0;
iteration = 2;
t0 = tic;

while toc(t0) < exeTime
    
    % Read Encoders
    encMsg = receive(encSub,0.5);
    leftWheelPosition = encMsg.position(idxL);
    rightWheelPosition = encMsg.position(idxR);

    % Odometric Estimation
    Dl = R * (leftWheelPosition - oldLeftWheelPos);
    Dr = R * (rightWheelPosition - oldRightWheelPos);
    Dc = (Dr + Dl) / 2;
    dTheta = (Dr - Dl) / L;

    % Values Recording (x,y,theta)
    x = x + Dc * cos(theta + dTheta/2);
    y = y + Dc * sin(theta + dTheta/2);
    theta = theta + dTheta;

    xPositionsForwardHigh(iteration) = x;
    yPositionsForwardHigh(iteration) = y;
    rotationAnglesForwardHigh(iteration) = theta;

    oldLeftWheelPos = leftWheelPosition;
    oldRightWheelPos = rightWheelPosition;
    iteration = iteration + 1;

    pause(period);
end



% Forward Motion Low Speed


velMsg.linear.x = 0.1;
velMsg.angular.z = 0;
send(velPub, velMsg);

encMsg = receive(encSub,10);
oldLeftWheelPos = encMsg.position(idxL);
oldRightWheelPos = encMsg.position(idxR);
x = 0; y = 0; theta = 0;
iteration = 2;
t0 = tic;

while toc(t0) < exeTime
    
    % Read Encoders
    encMsg = receive(encSub,0.5);
    leftWheelPosition = encMsg.position(idxL);
    rightWheelPosition = encMsg.position(idxR);

    % Odometric Estimation
    Dl = R * (leftWheelPosition - oldLeftWheelPos);
    Dr = R * (rightWheelPosition - oldRightWheelPos);
    Dc = (Dr + Dl) / 2;
    dTheta = (Dr - Dl) / L;

    % Values Recording (x,y,theta)
    x = x + Dc * cos(theta + dTheta/2);
    y = y + Dc * sin(theta + dTheta/2);
    theta = theta + dTheta;

    xPositionsForwardLow(iteration) = x;
    yPositionsForwardLow(iteration) = y;
    rotationAnglesForwardLow(iteration) = theta;

    oldLeftWheelPos = leftWheelPosition;
    oldRightWheelPos = rightWheelPosition;
    iteration = iteration + 1;

    pause(period);
end



% Backward Motion High Speed


velMsg.linear.x = -0.2;
velMsg.angular.z = 0;
send(velPub, velMsg);

encMsg = receive(encSub,10);
oldLeftWheelPos = encMsg.position(idxL);
oldRightWheelPos = encMsg.position(idxR);
x = 0; y = 0; theta = 0;
iteration = 2;
t0 = tic;

while toc(t0) < exeTime
    
    % Read Encoders
    encMsg = receive(encSub,0.5);
    leftWheelPosition = encMsg.position(idxL);
    rightWheelPosition = encMsg.position(idxR);

    % Odometric Estimation
    Dl = R * (leftWheelPosition - oldLeftWheelPos);
    Dr = R * (rightWheelPosition - oldRightWheelPos);
    Dc = (Dr + Dl) / 2;
    dTheta = (Dr - Dl)/L;

    % Values Recording (x,y,theta)
    x = x + Dc * cos(theta + dTheta/2);
    y = y + Dc * sin(theta + dTheta/2);
    theta = theta + dTheta;

    xPositionsBackwardHigh(iteration) = x;
    yPositionsBackwardHigh(iteration) = y;
    rotationAnglesBackwardHigh(iteration) = theta;

    oldLeftWheelPos = leftWheelPosition;
    oldRightWheelPos = rightWheelPosition;
    iteration = iteration + 1;

    pause(period);
end



% Backward Motion Low Speed


velMsg.linear.x = -0.1;
velMsg.angular.z = 0;
send(velPub, velMsg);

encMsg = receive(encSub,10);
oldLeftWheelPos = encMsg.position(idxL);
oldRightWheelPos = encMsg.position(idxR);
x = 0; y = 0; theta = 0;
iteration = 2;
t0 = tic;

while toc(t0) < exeTime
    
    % Read Encoders
    encMsg = receive(encSub,0.5);
    leftWheelPosition = encMsg.position(idxL);
    rightWheelPosition = encMsg.position(idxR);

    % Odometric Estimation
    Dl = R * (leftWheelPosition - oldLeftWheelPos);
    Dr = R * (rightWheelPosition - oldRightWheelPos);
    Dc = (Dr + Dl) / 2;
    dTheta = (Dr - Dl)/L;

    % Values Recording (x,y,theta)
    x = x + Dc * cos(theta + dTheta/2);
    y = y + Dc * sin(theta + dTheta/2);
    theta = theta + dTheta;

    xPositionsBackwardLow(iteration) = x;
    yPositionsBackwardLow(iteration) = y;
    rotationAnglesBackwardLow(iteration) = theta;

    oldLeftWheelPos = leftWheelPosition;
    oldRightWheelPos = rightWheelPosition;
    iteration = iteration + 1;

    pause(period);
end



% Left Rotation


velMsg.linear.x = 0.075;
velMsg.angular.z = 0.09;
send(velPub, velMsg);

encMsg = receive(encSub,10);
oldLeftWheelPos = encMsg.position(idxL);
oldRightWheelPos = encMsg.position(idxR);
x = 0; y = 0; theta = 0;
iteration = 2;
t0 = tic;

while toc(t0) < exeTime
    
    % Read Encoders
    encMsg = receive(encSub,0.5);
    leftWheelPosition = encMsg.position(idxL);
    rightWheelPosition = encMsg.position(idxR);

    % Odometric Estimation
    Dl = R * (leftWheelPosition - oldLeftWheelPos);
    Dr = R * (rightWheelPosition - oldRightWheelPos);
    Dc = (Dr + Dl)/2;
    dTheta = (Dr - Dl)/L;

    % Values Recording (x,y,theta)
    x = x + Dc * cos(theta + dTheta/2);
    y = y + Dc * sin(theta + dTheta/2);
    theta = theta + dTheta;

    xPositionsRotationLeft(iteration) = x;
    yPositionsRotationLeft(iteration) = y;
    rotationAnglesRotationLeft(iteration) = theta;

    oldLeftWheelPos = leftWheelPosition;
    oldRightWheelPos = rightWheelPosition;
    iteration = iteration + 1;

    pause(period);
end



% Right Rotation


velMsg.linear.x = 0.075;
velMsg.angular.z = -0.09;
send(velPub, velMsg);

encMsg = receive(encSub,10);
oldLeftWheelPos = encMsg.position(idxL);
oldRightWheelPos = encMsg.position(idxR);
x = 0; y = 0; theta = 0;
iteration = 2;
t0 = tic;

while toc(t0) < exeTime
    
    % Read Encoders
    encMsg = receive(encSub,0.5);
    leftWheelPosition = encMsg.position(idxL);
    rightWheelPosition = encMsg.position(idxR);

    % Odometric Estimation
    Dl = R * (leftWheelPosition - oldLeftWheelPos);
    Dr = R * (rightWheelPosition - oldRightWheelPos);
    Dc = (Dr + Dl)/2;
    dTheta = (Dr - Dl)/L;

    % Values Recording (x,y,theta)
    x = x + Dc * cos(theta + dTheta/2);
    y = y + Dc * sin(theta + dTheta/2);
    theta = theta + dTheta;

    xPositionsRotationRight(iteration) = x;
    yPositionsRotationRight(iteration) = y;
    rotationAnglesRotationRight(iteration) = theta;

    oldLeftWheelPos = leftWheelPosition;
    oldRightWheelPos = rightWheelPosition;
    iteration = iteration + 1;

    pause(period);
end



% Mixed Motion (three phases)


encMsg = receive(encSub,10);
oldLeftWheelPos = encMsg.position(idxL);
oldRightWheelPos = encMsg.position(idxR);
x = 0; y = 0; theta = 0;
iteration = 2;

% Phase 1
velMsg.linear.x = 0.1;
velMsg.angular.z = 0;
send(velPub, velMsg);
t0 = tic;
while toc(t0) < exeTime
    
    % Read Encoders
    encMsg = receive(encSub,0.5);
    leftWheelPosition = encMsg.position(idxL);
    rightWheelPosition = encMsg.position(idxR);

    % Odometric Estimation
    Dl = R * (leftWheelPosition - oldLeftWheelPos);
    Dr = R * (rightWheelPosition - oldRightWheelPos);
    Dc = (Dr + Dl)/2;
    dTheta = (Dr - Dl)/L;

    % Values Recording (x,y,theta)
    x = x + Dc * cos(theta + dTheta/2);
    y = y + Dc * sin(theta + dTheta/2);
    theta = theta + dTheta;

    xPositionsMixed(iteration) = x;
    yPositionsMixed(iteration) = y;
    rotationAnglesMixed(iteration) = theta;

    oldLeftWheelPos = leftWheelPosition;
    oldRightWheelPos = rightWheelPosition;
    iteration = iteration + 1;

    pause(period);
end

% Phase 2
velMsg.linear.x = 0.05;
velMsg.angular.z = 0.1;
send(velPub, velMsg);
t0 = tic;
while toc(t0) < exeTime
    
    % Read Encoders
    encMsg = receive(encSub,0.5);
    leftWheelPosition = encMsg.position(idxL);
    rightWheelPosition = encMsg.position(idxR);

    % Odometric Estimation
    Dl = R * (leftWheelPosition - oldLeftWheelPos);
    Dr = R * (rightWheelPosition - oldRightWheelPos);
    Dc = (Dr + Dl)/2;
    dTheta = (Dr - Dl)/L;

    % Values Recording (x,y,theta)
    x = x + Dc * cos(theta + dTheta/2);
    y = y + Dc * sin(theta + dTheta/2);
    theta = theta + dTheta;

    xPositionsMixed(iteration) = x;
    yPositionsMixed(iteration) = y;
    rotationAnglesMixed(iteration) = theta;

    oldLeftWheelPos = leftWheelPosition;
    oldRightWheelPos = rightWheelPosition;
    iteration = iteration + 1;

    pause(period);
end

% Phase 3
velMsg.linear.x = -0.1;
velMsg.angular.z = -0.05;
send(velPub, velMsg);
t0 = tic;
while toc(t0) < exeTime
    
    % Read Encoders
    encMsg = receive(encSub,0.5);
    leftWheelPosition = encMsg.position(idxL);
    rightWheelPosition = encMsg.position(idxR);

    % Odometric Estimation
    Dl = R * (leftWheelPosition - oldLeftWheelPos);
    Dr = R * (rightWheelPosition - oldRightWheelPos);
    Dc = (Dr + Dl)/2;
    dTheta = (Dr - Dl)/L;

    % Values Recording (x,y,theta)
    x = x + Dc * cos(theta + dTheta/2);
    y = y + Dc * sin(theta + dTheta/2);
    theta = theta + dTheta;

    xPositionsMixed(iteration) = x;
    yPositionsMixed(iteration) = y;
    rotationAnglesMixed(iteration) = theta;

    oldLeftWheelPos = leftWheelPosition;
    oldRightWheelPos = rightWheelPosition;
    iteration = iteration + 1;

    pause(period);
end

% Final stop
velMsg.linear.x = 0;
velMsg.angular.z = 0;
send(velPub, velMsg);