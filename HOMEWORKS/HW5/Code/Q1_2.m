% Parameters
dt = 1; % time step
T = 500; % total time steps

% Noise covariances
Q = 0.01 * eye(2); % process noise covariance
R = diag([0.05^2, (deg2rad(2))^2]); % measurement noise covariance (distance and angle)

% Initial state and covariance
x_true = zeros(2, T);
x_est = zeros(2, T);
P = zeros(2, 2, T);

x_est(:,1) = [0; 0];
P(:,:,1) = eye(2);

% Control inputs (example: move diagonally with some variation)
u = repmat([0.5; 0.3], 1, T) + 0.1*randn(2, T);

% Storage for predicted states and covariances
x_pred = zeros(2, T);
P_pred = zeros(2, 2, T);

% Simulate true robot motion
for k = 2:T
    w = mvnrnd([0 0], Q)';
    x_true(:,k) = x_true(:,k-1) + u(:,k-1) + w;
end

% EKF loop
for k = 2:T
    % Prediction
    x_pred(:,k) = x_est(:,k-1) + u(:,k-1);
    P_pred(:,:,k) = P(:,:,k-1) + Q;
    
    % Measurement simulation
    d = norm(x_true(:,k));
    alpha = atan2(x_true(2,k), x_true(1,k));
    v = mvnrnd([0 0], R)';
    z = [d; alpha] + v;
    
    % Compute Jacobian H
    px = x_pred(1,k);
    py = x_pred(2,k);
    dist = sqrt(px^2 + py^2);
    if dist < 1e-4
        dist = 1e-4; % avoid division by zero
    end
    H = [px/dist, py/dist;
        -py/(dist^2), px/(dist^2)];
    
    % Innovation
    h_x = [dist; atan2(py, px)];
    y = z - h_x;
    % Normalize angle difference to [-pi, pi]
    y(2) = mod(y(2) + pi, 2*pi) - pi;
    
    % Innovation covariance
    S = H * P_pred(:,:,k) * H' + R;
    
    % Kalman gain
    K = P_pred(:,:,k) * H' / S;
    
    % Update
    x_est(:,k) = x_pred(:,k) + K * y;
    P(:,:,k) = (eye(2) - K * H) * P_pred(:,:,k);
end

% Plot results
time = 1:T;

figure;
subplot(2,1,1);
plot(time, x_true(1,:), 'g-', time, x_pred(1,:), 'b--', time, x_est(1,:), 'r-');
legend('True x', 'Predicted x', 'Estimated x');
xlabel('Time step');
ylabel('x position');
title('State x over time');

subplot(2,1,2);
plot(time, x_true(2,:), 'g-', time, x_pred(2,:), 'b--', time, x_est(2,:), 'r-');
legend('True y', 'Predicted y', 'Estimated y');
xlabel('Time step');
ylabel('y position');
title('State y over time');

figure;
subplot(2,2,1);
plot(time, squeeze(P_pred(1,1,:)), 'b-');
title('P_{11} (a priori covariance)');
xlabel('Time step');
ylabel('Variance');

subplot(2,2,2);
plot(time, squeeze(P_pred(2,2,:)), 'b-');
title('P_{22} (a priori covariance)');
xlabel('Time step');
ylabel('Variance');

subplot(2,2,3);
plot(time, squeeze(P(1,1,:)), 'r-');
title('P_{11} (a posteriori covariance)');
xlabel('Time step');
ylabel('Variance');

subplot(2,2,4);
plot(time, squeeze(P(2,2,:)), 'r-');
title('P_{22} (a posteriori covariance)');
xlabel('Time step');
ylabel('Variance');