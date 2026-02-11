figure;

subplot(2,1,1);
plot(xPositionsForwardHigh, yPositionsForwardHigh);
hold on;
plot(xFH, yFH);
ylim([-0.05 0.05]);
title("Trajectory Of Forward Motion With High Speed");
xlabel("X Coordinate");
ylabel("Y Coordinate");
grid on;
legend("Estimated Trajectory", "Desired Trajectory");
hold off;

subplot(2,1,2);
plot(rotationAnglesForwardHigh);
hold on;
plot(tFH);
plot(tFH);
ylim([-0.05 0.05]);
title("Rotation Angle For Forward Motion With High Speed");
xlabel("Iterations");
ylabel("Rotation Angle (theta)");
grid on;
legend("Estimated Trajectory", "Desired Trajectory");
hold off;



figure;
subplot(2,1,1);
plot(xPositionsForwardLow, yPositionsForwardLow);
hold on;
plot(xFL, yFL);
ylim([-0.05 0.05]);
title("Trajectory Of Forward Motion With Low Speed");
xlabel("X Coordinate");
ylabel("Y Coordinate");
grid on;
legend("Estimated Trajectory", "Desired Trajectory");
hold off;

subplot(2,1,2);
plot(rotationAnglesForwardLow);
hold on;
plot(tFL);
ylim([-0.05 0.05]);
title("Rotation Angle For Forward Motion With Low Speed");
xlabel("Iterations");
ylabel("Rotation Angle (theta)");
grid on;
legend("Estimated Trajectory", "Desired Trajectory");
hold off;



figure;
subplot(2,1,1);
plot(xPositionsBackwardHigh, yPositionsBackwardHigh);
hold on;
plot(xBH, yBH);
set(gca, 'XDir', 'reverse');
ylim([-0.05 0.05]);
title("Trajectory Of Backward Motion With High Speed");
xlabel("X Coordinate");
ylabel("Y Coordinate");
grid on;
legend("Estimated Trajectory", "Desired Trajectory");
hold off;

subplot(2,1,2);
plot(rotationAnglesBackwardHigh);
hold on;
plot(tBH);
ylim([-0.05 0.05]);
title("Rotation Angle For Backward Motion With High Speed");
xlabel("Iterations");
ylabel("Rotation Angle (theta)");
grid on;
legend("Estimated Trajectory", "Desired Trajectory");
hold off;


figure;
subplot(2,1,1);
plot(xPositionsBackwardLow, yPositionsBackwardLow);
hold on;
plot(xBL, yBL);
set(gca, 'XDir', 'reverse');
ylim([-0.05 0.05]);
title("Trajectory Of Backward Motion With Low Speed");
xlabel("X Coordinate");
ylabel("Y Coordinate");
grid on;
legend("Estimated Trajectory", "Desired Trajectory");
hold off;

subplot(2,1,2);
plot(rotationAnglesBackwardLow);
hold on;
plot(tBL);
ylim([-0.05 0.05]);
title("Rotation Angle For Backward Motion With Low Speed");
xlabel("Iterations");
ylabel("Rotation Angle (theta)");
grid on;
legend("Estimated Trajectory", "Desired Trajectory");
hold off;


figure;
subplot(2,1,1);
plot(xPositionsRotationLeft, yPositionsRotationLeft);
hold on;
plot(xRL, yRL);
ylim([-0.05 0.05]);
title("Trajectory Of Rotation Motion Towards Left");
xlabel("X Coordinate");
ylabel("Y Coordinate");
grid on;
legend("Estimated Trajectory", "Desired Trajectory");
hold off;

subplot(2,1,2);
plot(rotationAnglesRotationLeft);
hold on;
plot(tRL);
title("Rotation Angle For Rotation Motion Towards Left");
xlabel("Iterations");
ylabel("Rotation Angle (theta)");
grid on;
legend("Estimated Trajectory", "Desired Trajectory");
hold off;


figure;
subplot(2,1,1);
plot(xPositionsRotationRight, yPositionsRotationRight);
hold on;
plot(xRR, yRR);
ylim([-0.05 0.05]);
title("Trajectory Of Rotation Motion Towards Right");
xlabel("X Coordinate");
ylabel("Y Coordinate");
grid on;
legend("Estimated Trajectory", "Desired Trajectory");
hold off;

subplot(2,1,2);
plot(rotationAnglesRotationRight);
hold on;
plot(tRR);
title("Rotation Angle For Rotation Motion Towards Right");
xlabel("Iterations");
ylabel("Rotation Angle (theta)");
grid on;
legend("Estimated Trajectory", "Desired Trajectory");
hold off;


figure;
subplot(2,1,1);
plot(xPositionsMixed, yPositionsMixed);
hold on;
plot(xM, yM);
ylim([-0.05 0.05]);
title("Trajectory Of Mixed Motion");
xlabel("X Coordinate");
ylabel("Y Coordinate");
grid on;
legend("Estimated Trajectory", "Desired Trajectory");
hold off;

subplot(2,1,2);
plot(rotationAnglesMixed);
hold on;
plot(tM);
title("Rotation Angle For Mixed Motion");
xlabel("Iterations");
ylabel("Rotation Angle (theta)");
grid on;
legend("Estimated Trajectory", "Desired Trajectory");
hold off;