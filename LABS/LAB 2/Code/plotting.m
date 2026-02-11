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