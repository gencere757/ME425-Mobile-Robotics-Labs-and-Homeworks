clc; clear; close all;

%% REAL - TIME PLOTTING SETUP
y_est_values = [];
theta_est_values = [];
P11_values = [];
P22_values = [];
t_values = [];

figure('Position', [100 100 1200 600]);

subplot(2,2,1);
yGraph = plot(nan, nan, 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('y (m)');
title('Lateral Distance y');
grid on;

subplot(2,2,2);
thetaGraph = plot(nan, nan, 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('\theta (rad)');
title('Heading Angle \theta');
grid on;

subplot(2,2,3);
P11Graph = plot(nan, nan, 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Var(y)');
title('Variance P_{11}');
grid on;

subplot(2,2,4);
P22Graph = plot(nan, nan, 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Var(\theta)');
title('Variance P_{22}');
grid on;

clear node;

setenv('ROS_DOMAIN_ID','43');
setenv('ROS_LOCALHOST_ONLY', '0');
setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp');

node = ros2node('Kalman2D');

%% Subscribers And Publishers

% Lidar Subscriber
scanSub = ros2subscriber(node, "/scan", "sensor_msgs/LaserScan",...
    "Reliability", "besteffort", "Durability", "volatile", "Depth", 10);

% Velocity Publisher
velPub = ros2publisher(node, "/cmd_vel", "geometry_msgs/Twist", ...
    "Reliability", "reliable", "Durability", "volatile", "Depth", 10);

velMsg = ros2message(velPub);

%% Initialization

Q = 0.001 * eye(2);
R = 0.01 * eye(2);
delta_t = 0.1;

old_P = eye(2);
old_x_k = [0.3; 0];

%% MAIN LOOP

timer = tic;
duration = 8;

while toc(timer) < duration
    % Forward Backward motion
    currentTime = toc(timer);
    if mod(currentTime, 4) < 2
        v = 0.1;
    else
        v = -0.1;
    end
    velMsg.linear.x = v;
    send(velPub, velMsg);
    
    % Get Measurements
    scanMsg = receive(scanSub, 10);
    angles = scanMsg.angle_min + (0:length(scanMsg.ranges)-1) * scanMsg.angle_increment;
    idx = find(angles >= pi/3 & angles <= 2*pi/3);
    r_wall = scanMsg.ranges(idx);
    th_wall = angles(idx);
    valid = r_wall > scanMsg.range_min & r_wall < scanMsg.range_max;
    
    % Fit Measurements
    x_pts = r_wall(valid) .* cos(th_wall(valid));
    y_pts = r_wall(valid) .* sin(th_wall(valid));
    
    if length(x_pts) < 5  % ← SAFETY CHECK!
        continue;
    end
    
    p = polyfit(x_pts, y_pts, 1);
    m = p(1);
    b = p(2);
    
    theta_meas = atan(m);
    y_meas = b / sqrt(m^2 + 1);
    z_k = [y_meas; theta_meas];
    
    % Kalman Filter
    A = eye(2);
    H = eye(2);
    
    B = [sin(old_x_k(2)) * delta_t; 0];  % 2x1
    u_k = v;
    
    % Prediction
    x_k_before = A * old_x_k + B * u_k;
    P_before = A * old_P * A' + Q;
    
    % Update
    S = H * P_before * H' + R;
    kalman_gain = P_before * H' / S;
    x_k_after = x_k_before + kalman_gain * (z_k - H * x_k_before);
    P_after = (eye(2) - kalman_gain * H) * P_before;
    
    % Update state
    old_P = P_after;
    old_x_k = x_k_after;
    
    % Store data
    y_est_values = [y_est_values x_k_after(1)];
    theta_est_values = [theta_est_values x_k_after(2)];
    P11_values = [P11_values P_after(1,1)];
    P22_values = [P22_values P_after(2,2)];
    t_values = [t_values currentTime];  % ← DON'T FORGET THIS!
    
    % Update all 4 plots
    subplot(2,2,1);
    set(yGraph, "XData", t_values, "YData", y_est_values);
    
    subplot(2,2,2);
    set(thetaGraph, "XData", t_values, "YData", theta_est_values);
    
    subplot(2,2,3);
    set(P11Graph, "XData", t_values, "YData", P11_values);
    
    subplot(2,2,4);
    set(P22Graph, "XData", t_values, "YData", P22_values);
    
    drawnow limitrate;
end

% Stop Robot
velMsg.linear.x = 0;
send(velPub, velMsg);