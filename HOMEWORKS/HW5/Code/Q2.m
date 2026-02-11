    clc; clear; close all;
    
    T = 0.05;  %Sampling Time
    
    t = 0;  %Current Time
    simTime = 100;
    
    q_real = [1e-3;1e-3; 1e-3];
    u = [0.1;0.1];  %Velocity Commands
    
    q_old = q_real;
    P_old = [0.005 0 0; 0 0.005 0;0 0 0.005];
    real_covariance = [0.0001 0 0; 0 0.0001 0;0 0 0.0001];
    Q = [0.01 0 0; 0 0.01 0; 0 0 0.001];
    R = [0.0005 0; 0 0.0005];
    %Data Recording Initialization
    posRecord = [1e-3;1e-3; 1e-3];
    estimateRecordBefore = [1e-3;1e-3; 1e-3];
    estimateRecordAfter = [1e-3;1e-3; 1e-3];
    times = [0];
    P11_vals_before = [0.005];
    P22_vals_before = [0.005];
    P11_vals_after = [0.005];
    P22_vals_after = [0.005];
    P33_vals_before = [0.005];
    P33_vals_after = [0.005];
        
    while t < simTime
        %Take Measurement
        d = sqrt(q_real(1)^2 + q_real(2)^2);
        alpha = atan2(q_real(2), q_real(1));
        measurement_noise = transpose(mvnrnd(zeros(2,1), R));
        measurement = [d; alpha] + measurement_noise;
    
        %Extended Kalman Filter
        B = [cos(q_old(3)) 0;sin(q_old(3)) 0; 0 1];
        q_before = q_old + T * B * u;

        x = q_before(1);
        y = q_before(2);
        theta = q_before(3);
        theta = mod(theta + pi, 2*pi) - pi;  % wrap to [-pi, pi]
        innovation = measurement- [sqrt(x^2+y^2); atan2(y,x)];
        %Angle Wrapping
        if innovation(2) < -pi
            innovation(2) = innovation(2) + 2*pi;
        elseif innovation(2) > pi
            innovation(2) = innovation(2) - 2*pi;
        end
        A = [1 0 -u(1)*sin(theta)*T; 0 1 u(1)*cos(theta)*T;0 0 1];
        H = [sqrt(x^2+y^2)^-1*x sqrt(x^2+y^2)^-1*y 0; -y/(x^2+y^2) x/(x^2+y^2) 0];  %Jacobian
        P_before = A*P_old*transpose(A) + Q;
        kalman_Gain = P_before*transpose(H) * inv(H* P_before*transpose(H)+R);
        q_after = q_before + kalman_Gain * (innovation);
        q_after(3) = mod(q_after(3) + pi, 2*pi) - pi;  % wrap to [-pi, pi]
        P_after = (eye(size(kalman_Gain*H)) - kalman_Gain * H)*P_before;
    
    
        %Update Real Location
        noise_real = transpose(mvnrnd(zeros(3,1), real_covariance));
        q_real = q_real + T * B * u + noise_real;
        q_real(3) = mod(q_real(3) + pi, 2*pi) - pi;  % wrap to [-pi, pi]
        %Update Time And Other Variables
        t = t + T;
        q_old = q_after;
        q_old(3) = mod(q_old(3) + pi, 2*pi) - pi;  % wrap to [-pi, pi]
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
        P33_vals_before = [P33_vals_before P_before(3,3)];
        P33_vals_after = [P33_vals_after P_after(3,3)];
    end
        
    %% Plotting
    figure;
    subplot(3,1,1);
    plot(times, posRecord(1,:),"DisplayName","Real X Values");
    title("Real, Predicted and Optimal X Values");
    hold on;
    plot(times, estimateRecordAfter(1,:),"DisplayName","Optimal X Values (After Measurement)");
    plot(times, estimateRecordBefore(1,:), "DisplayName","Predicted X Values (Before Measurement)");
    grid on;
    legend show;
    xlabel("Time [s]");
    ylabel("X Coordinate");
    
    subplot(3,1,2);
    plot(times, posRecord(2,:),"DisplayName","Real Y Values");
    title("Real, Predicted and Optimal Y Values");
    hold on;
    xlabel("Time [s]");
    ylabel("Y Coordinate");
    plot(times, estimateRecordAfter(2,:),"DisplayName","Optimal Y Values (After Measurement)");
    plot(times, estimateRecordBefore(2,:), "DisplayName","Predicted Y Values (Before Measurement)");
    grid on;
    legend show;

    subplot(3,1,3);
    plot(times, posRecord(3,:),"DisplayName","Real Theta Values");
    title("Real, Predicted and Optimal Theta Values");
    hold on;
    xlabel("Time [s]");
    ylabel("Theta Value");
    plot(times, estimateRecordAfter(3,:),"DisplayName","Optimal Theta Values (After Measurement)");
    plot(times, estimateRecordBefore(3,:), "DisplayName","Predicted Theta Values (Before Measurement)");
    grid on;
    legend show;
    


    figure;
    subplot(3,1,1);
    plot(times,P11_vals_before,"DisplayName", "Covariance For X Coordinate Estimation (Before Measurement)");
    hold on;
    plot(times, P11_vals_after,"DisplayName","Covariance For X Coordinate Estimation (Updated)");
    grid on;
    title("P11 Values")
    legend show;
    xlabel("Time [s]");
    ylabel("P11");
    
    subplot(3,1,2);
    plot(times,P11_vals_before,"DisplayName", "Covariance For Y Coordinate Estimation (Before Measurement)");
    hold on;
    plot(times,P22_vals_after,"DisplayName","Covariance For Y Coordinate Estimation (Updated)");
    grid on;
    title("P22 Values");
    legend show;
    xlabel("Time [s]");
    ylabel("P22");

    subplot(3,1,3);
    plot(times,P33_vals_before,"DisplayName", "Covariance For Theta Coordinate Estimation (Before Measurement)");
    hold on;
    plot(times,P33_vals_after,"DisplayName","Covariance For Theta Coordinate Estimation (Updated)");
    grid on;
    title("P33 Values");
    legend show;
    xlabel("Time [s]");
    ylabel("P33");