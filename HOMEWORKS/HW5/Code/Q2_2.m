% ekf_range_bearing.m
% Extended Kalman Filter for a unicycle robot with range/bearing measurements from origin.
clear; close all; clc;

%% Simulation parameters
dt = 0.1;               % time step [s]
T = 60;                 % total time [s]
N = floor(T/dt)+1;
t = (0:N-1)'*dt;

%% Ground truth initial state
x_true = zeros(3, N);
x_true(:,1) = [0.5; -0.2; deg2rad(30)];   % [x; y; theta]

%% Control profile (v, omega)
v_cmd = 0.8 + 0.4*sin(0.2*t);             % [m/s]
omega_cmd = 0.1*cos(0.07*t);              % [rad/s]
u = [v_cmd, omega_cmd]';

%% Noise parameters
sigma_v = 0.05;          % std dev of linear speed noise [m/s]
sigma_w = 0.02;          % std dev of angular speed noise [rad/s]
Q_u = diag([sigma_v^2, sigma_w^2]);  % control noise covariance

sigma_r = 0.3;           % range noise std [m]
sigma_alpha = deg2rad(1.0); % bearing noise std [rad]
R = diag([sigma_r^2, sigma_alpha^2]);

%% EKF initialization
x_est = zeros(3, N);
P = zeros(3,3,N);
% initial estimate and covariance
x_est(:,1) = [0.0; 0.0; deg2rad(0)];   % maybe wrong initial pose
P(:,:,1) = diag([1.0, 1.0, deg2rad(30)^2]);

% storage for priors (a-priori) and posts (a-posteriori)
x_minus = zeros(3,N);
P_minus = zeros(3,3,N);

%% Simulate true trajectory with process noise added to controls
rng(1); % for reproducibility
for k = 1:N-1
    v = u(1,k);
    w = u(2,k);
    % process noise on controls
    vn = sigma_v * randn;
    wn = sigma_w * randn;
    v_noisy = v + vn;
    w_noisy = w + wn;
    theta = x_true(3,k);

    % true motion (Euler)
    x_true(1,k+1) = x_true(1,k) + v_noisy * dt * cos(theta);
    x_true(2,k+1) = x_true(2,k) + v_noisy * dt * sin(theta);
    x_true(3,k+1) = wrapToPi(x_true(3,k) + w_noisy * dt);
end

%% Generate noisy measurements z_k = [range; bearing] at each time
z = zeros(2,N);
for k = 1:N
    xx = x_true(1,k);
    yy = x_true(2,k);
    d = sqrt(xx^2 + yy^2);
    alpha = atan2(yy, xx);

    z(1,k) = d + sigma_r * randn;
    z(2,k) = wrapToPi(alpha + sigma_alpha * randn);
end

%% EKF loop
for k = 2:N
    % previous posterior estimate
    x_prev = x_est(:,k-1);
    P_prev = P(:,:,k-1);

    % control used (we assume measured command without extra bias)
    v = u(1,k-1);
    w = u(2,k-1);
    theta = x_prev(3);

    % 1) Prediction
    x_pred = zeros(3,1);
    x_pred(1) = x_prev(1) + v * dt * cos(theta);
    x_pred(2) = x_prev(2) + v * dt * sin(theta);
    x_pred(3) = wrapToPi(x_prev(3) + w * dt);

    % Jacobian F = df/dx at previous state
    F = [1, 0, -v*dt*sin(theta);
         0, 1,  v*dt*cos(theta);
         0, 0,  1];

    % Jacobian L = df/du
    L = [dt*cos(theta), 0;
         dt*sin(theta), 0;
         0,             dt];

    Qk = L * Q_u * L';

    P_pred = F * P_prev * F' + Qk;

    % store prior
    x_minus(:,k) = x_pred;
    P_minus(:,:,k) = P_pred;

    % 2) Measurement update
    % measurement predicted
    px = x_pred(1); py = x_pred(2);
    r2 = px^2 + py^2;
    d_pred = sqrt(r2);
    alpha_pred = atan2(py, px);
    z_pred = [d_pred; alpha_pred];

    % Measurement Jacobian H
    if d_pred < 1e-8
        % avoid division by zero; set H to zeros (poor observability near origin)
        H = zeros(2,3);
    else
        H = [ px/d_pred,        py/d_pred,      0;
             -py/r2,            px/r2,          0 ];
    end

    S = H * P_pred * H' + R;
    K = P_pred * H' / S;

    % innovation
    innov = z(:,k) - z_pred;
    innov(2) = wrapToPi(innov(2));

    % update
    x_upd = x_pred + K * innov;
    x_upd(3) = wrapToPi(x_upd(3));
    P_upd = (eye(3) - K*H) * P_pred;

    % store posterior
    x_est(:,k) = x_upd;
    P(:,:,k) = P_upd;
end

%% Plot results
figure('Name','States: True vs Estimated');
subplot(3,1,1);
plot(t, x_true(1,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t, x_est(1,:), 'r--', 'LineWidth', 1.3);
xlabel('Time [s]'); ylabel('x [m]'); legend('True','Estimated'); grid on;

subplot(3,1,2);
plot(t, x_true(2,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t, x_est(2,:), 'r--', 'LineWidth', 1.3);
xlabel('Time [s]'); ylabel('y [m]'); legend('True','Estimated'); grid on;

subplot(3,1,3);
plot(t, wrapToPi(x_true(3,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(t, wrapToPi(x_est(3,:)), 'r--', 'LineWidth', 1.3);
xlabel('Time [s]'); ylabel('\theta [rad]'); legend('True','Estimated'); grid on;

% Covariances (a-priori and a-posteriori diagonal elements)
Pminus11 = squeeze(P_minus(1,1,:));
Pminus22 = squeeze(P_minus(2,2,:));
Pminus33 = squeeze(P_minus(3,3,:));

Ppost11 = squeeze(P(1,1,:));
Ppost22 = squeeze(P(2,2,:));
Ppost33 = squeeze(P(3,3,:));

figure('Name','Covariance Diagonals (a-priori vs a-posteriori)');
subplot(3,1,1);
plot(t, Pminus11, 'm--', 'LineWidth', 1.2); hold on;
plot(t, Ppost11, 'k-', 'LineWidth', 1.2);
ylabel('P_{11}'); legend('P^-','P'); grid on;

subplot(3,1,2);
plot(t, Pminus22, 'm--', 'LineWidth', 1.2); hold on;
plot(t, Ppost22, 'k-', 'LineWidth', 1.2);
ylabel('P_{22}'); legend('P^-','P'); grid on;

subplot(3,1,3);
plot(t, Pminus33, 'm--', 'LineWidth', 1.2); hold on;
plot(t, Ppost33, 'k-', 'LineWidth', 1.2);
ylabel('P_{33}'); xlabel('Time [s]'); legend('P^-','P'); grid on;

% Plot estimation errors
err = x_true - x_est;
err(3,:) = arrayfun(@(a) wrapToPi(a), err(3,:));
figure('Name','Estimation errors');
subplot(3,1,1); plot(t, err(1,:), 'LineWidth', 1.2); ylabel('x error [m]'); grid on;
subplot(3,1,2); plot(t, err(2,:), 'LineWidth', 1.2); ylabel('y error [m]'); grid on;
subplot(3,1,3); plot(t, err(3,:), 'LineWidth', 1.2); ylabel('\theta error [rad]'); xlabel('Time [s]'); grid on;

% 2D trajectory
figure('Name','2D Trajectory');
plot(x_true(1,:), x_true(2,:), 'b-', 'LineWidth', 1.5); hold on;
plot(x_est(1,:), x_est(2,:), 'r--', 'LineWidth', 1.2);
plot(0,0,'ko','MarkerSize',6,'MarkerFaceColor','k'); % origin
legend('True','Estimated','Origin'); axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]'); title('Trajectory');

disp('Simulation complete.');