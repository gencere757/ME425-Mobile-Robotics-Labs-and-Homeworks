clc; clear; close all;

T = 0.1;  %Sampling Time

t = 0;  %Current Time
simTime = 100;

q_real = [1e-3;1e-3];
u = [0.1;0.1];  %Velocity Commands

A = eye(2);
q_old = q_real;
P_old = [0.005 0; 0 0.005];
real_covariance = [0.0001 0; 0 0.0001];
Q = [0.01 0; 0 0.01];
R = [0.0025 0; 0 0.0012];
%Data Recording Initialization
posRecord = [1e-3;1e-3];
estimateRecordBefore = [1e-3;1e-3];
estimateRecordAfter = [1e-3;1e-3];
times = [0];
P11_vals_before = [0.005];
P22_vals_before = [0.005];
P11_vals_after = [0.005];
P22_vals_after = [0.005];
    
while t < simTime
    %Take Measurement
    d = sqrt(q_real(1)^2 + q_real(2)^2);
    alpha = atan2(q_real(2), q_real(1));
    measurement_noise = transpose(mvnrnd(zeros(2,1), R));
    measurement = [d; alpha] + measurement_noise;

    %Extended Kalman Filter
    q_before = q_old + T * u;
    x = q_before(1);
    y = q_before(2);
    dist = x^2 + y^2;
    if dist < 1e-4
        dist = 1e-4;
    end
    H = [sqrt(dist)^-1*x sqrt(dist)^-1*y; -y/(dist) x/(dist)];  %Jacobian
    P_before = A*P_old*transpose(A) + Q;
    kalman_Gain = P_before*transpose(H) * inv(H* P_before*transpose(H)+R);
    innovation = measurement- [sqrt(x^2+y^2); atan2(y,x)];
    %Angle Wrapping
    innovation(2) = mod(innovation(2) + pi, 2*pi) - pi;

    q_after = q_before + kalman_Gain * (innovation);
    P_after = (eye(size(kalman_Gain*H)) - kalman_Gain * H)*P_before;


    %Update Real Location
    noise_real = transpose(mvnrnd(zeros(2,1), real_covariance));
    q_real = q_real + [u(1) * T; u(2) * T] + noise_real;
    %Update Time And Other Variables
    t = t + T;
    q_old = q_after;
    P_old = P_after;


    %Record Values
    posRecord = [posRecord q_real];
    estimateRecordBefore = [estimateRecordBefore q_before];
    estimateRecordAfter = [estimateRecordAfter q_after];
    times = [times t];
    P11_vals_before = [P11_vals_before P_before(1,1)];
    P22_vals_before = [P22_vals_before P_before(2,2)];
    P11_vals_after = [P11_vals_after P_after(1,1)];
    P22_vals_after = [P22_vals_after P_after(2,2)];
end
    
%% Plotting
figure;
subplot(2,1,1);
plot(times, posRecord(1,:),"DisplayName","Real X Values");
title("Real, Predicted and Optimal X Values");
hold on;
plot(times, estimateRecordAfter(1,:),"DisplayName","Optimal X Values (After Measurement)");
plot(times, estimateRecordBefore(1,:), "DisplayName","Predicted X Values (Before Measurement)");
grid on;
legend show;
xlabel("Time [s]");
ylabel("X Coordinate");

subplot(2,1,2);
plot(times, posRecord(2,:),"DisplayName","Real Y Values");
title("Real, Predicted and Optimal Y Values");
hold on;
xlabel("Time [s]");
ylabel("Y Coordinate");
plot(times, estimateRecordAfter(2,:),"DisplayName","Optimal X Values (After Measurement)");
plot(times, estimateRecordBefore(2,:), "DisplayName","Predicted Y Values (Before Measurement)");
grid on;
legend show;

figure;
subplot(2,1,1);
plot(times,P11_vals_before,"DisplayName", "Covariance For X Coordinate Estimation (Before Measurement)");
hold on;
plot(times, P11_vals_after,"DisplayName","Covariance For X Coordinate Estimation (Updated)");
grid on;
title("P11 Values")
legend show;
xlabel("Time [s]");
ylabel("P11");

subplot(2,1,2);
plot(times,P11_vals_before,"DisplayName", "Covariance For X Coordinate Estimation (Before Measurement)");
hold on;
plot(times,P22_vals_after,"DisplayName","Covariance For Y Coordinate Estimation (Updated)");
grid on;
title("P22 Values");
legend show;
xlabel("Time [s]");
ylabel("P22");