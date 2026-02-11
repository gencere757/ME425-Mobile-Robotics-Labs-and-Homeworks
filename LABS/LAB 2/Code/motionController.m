clear; close all; clc; clear node;
wrap = @(ang) atan2(sin(ang), cos(ang));
%% 1 Environment Setup
setenv('ROS_DOMAIN_ID','43'); % Set to your robot's ROS_DOMAIN_ID
setenv('ROS_LOCALHOST_ONLY', '0'); % 0 implies multi-host communication
setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'); % Middleware

node = ros2node('motionController');  % Create node
%% 2 Publishers & Subscribers
velPub = ros2publisher(node,"/cmd_vel","geometry_msgs/Twist", ...
"Reliability","reliable","Durability","volatile","Depth",10);

encSub = ros2subscriber(node,"/joint_states","sensor_msgs/JointState", ...
"Reliability","besteffort","Durability","volatile","Depth",10);
%% 3 Initial Encoder Reading
encMsg = receive(encSub,10);
jointNames = string(encMsg.name);
idxL = find(jointNames=="wheel_left_joint" | jointNames=="left_wheel",1);
idxR = find(jointNames=="wheel_right_joint" | jointNames=="right_wheel",1);
oldLeftWheelPos = encMsg.position(idxL);
oldRightWheelPos = encMsg.position(idxR);

%% 4 Parameters
x = -0.3; % Your Starting x [m]
y = -0.3; % Your Starting y [m]
theta = 0.05; % Your Starting theta [rad]
x_goal = 0.0;
y_goal = 0.0;
R = 0.033; % Wheel Radius [m]
L = 0.287; % Axle Distance [m]

%% 5 Variables Initialization
%dataMatrix = [];
x_values = [x];
y_values = [y];
theta_values = [theta];
rho_values = [sqrt((x_goal - x)^2+(y_goal - y)^2)];

velMsg = ros2message(velPub);
velMsg.linear.y = 0;
velMsg.linear.z = 0;
velMsg.angular.x = 0;
velMsg.angular.y = 0;

deltaX = x_goal - x;
deltaY = y_goal - y;
ro = sqrt(deltaX^2+deltaY^2);
alpha = -theta + atan2(deltaY,deltaX);
beta = -theta - alpha;
kp = 0.15;
ka = 0.8;
kb = -0.25;
v = kp*ro;
omega = ka*alpha+kb*beta;
v = max(min(v, 0.22), -0.22);
omega = max(min(omega, 2.84), -2.84);

velMsg.linear.x = v;
velMsg.angular.z = omega;
send(velPub, velMsg);

exeTime = 20; period = 0.01; t0 = tic;
iteration = 2;
while toc(t0) < exeTime
    
   %% 6.1 Read Encoders
    encMsg = receive(encSub,10);
    leftWheelPosition = encMsg.position(idxL);
    rightWheelPosition = encMsg.position(idxR);

    %% 6.2 Odometric Estimation
    Dl = R * (leftWheelPosition - oldLeftWheelPos);
    Dr = R * (rightWheelPosition - oldRightWheelPos);
    Dc = (Dr + Dl) / 2;
    dTheta = (Dr - Dl) / L;

    %% 6.3 Values Recording (x,y,theta)
    x = x + Dc * cos(theta + dTheta/2);
    y = y + Dc * sin(theta + dTheta/2);
    theta = theta + dTheta;
    
    x_values(iteration) = x;
    y_values(iteration) = y;
    theta_values(iteration) = theta;
   

    %% 6.4 Motion Controller
    deltaX = x_goal - x;
    deltaY = y_goal - y;
    ro = sqrt(deltaX^2+deltaY^2);
    rho_values(iteration) = ro;
    alpha = -theta + atan2(deltaY,deltaX);
    beta = -theta - alpha;
    
    v = kp*ro;
    omega = ka*alpha+kb*beta;
    
    velMsg.linear.x = v;
    velMsg.angular.z = omega;
    send(velPub, velMsg);

    oldLeftWheelPos = leftWheelPosition;
    oldRightWheelPos = rightWheelPosition;
    iteration = iteration + 1;
    
    pause(period);
end

% Final stop
velMsg.linear.x = 0;
velMsg.angular.z = 0;
send(velPub, velMsg);


%% 7 Plotting Results
figure; plot(x_values,y_values);
xlabel("X Position (m)"); ylabel("Y Position (m)");
xlim([-0.5 0.5]); ylim([-0.5 0.5]); grid on;
figure; subplot(1,4,1); plot(x_values);
xlabel("Time"); ylabel("X [m]"); grid on;
subplot(1,4,2); plot(y_values);
xlabel("Time"); ylabel("Y [m]"); grid on;
subplot(1,4,3); plot(rho_values);
xlabel("Time"); ylabel("Rho [m]"); grid on;
subplot(1,4,4); plot(theta_values);
xlabel("Time"); ylabel("Theta [rad]"); grid on;
