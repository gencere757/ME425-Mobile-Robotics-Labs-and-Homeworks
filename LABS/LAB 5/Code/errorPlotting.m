figure;
subplot(2,1,1);
plot(linearErrors);
grid on;
title("Linear Errors");
xlabel("Iteration");
ylabel("Error [m]");


subplot(2,1,2);
plot(angularErrors);
grid on;
title("Angular Errors");
xlabel("Iteration");
ylabel("Error [rad]");