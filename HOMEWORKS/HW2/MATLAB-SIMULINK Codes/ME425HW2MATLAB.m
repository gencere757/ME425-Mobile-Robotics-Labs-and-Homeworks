clc;clear;close all;
%Variable Initialization
rho_initial=0.5;
alpha_initial=pi/6;
beta_initial=-pi/12;

k_rho=0.45;
k_alpha=3;
k_beta=-1;
simTime = 20;

%Running the simulation
out=sim("ME425HW2SIMULINK.slx");

%Getting the variables from the simulation
t=out.tout;
rho=out.rho;
alpha=out.alpha;
beta=out.beta;

%Plotting
figure;
subplot(3,1,1);
plot(t,rho);
xlabel("Time (s)");
ylabel("ρ (m)");
grid on;
subplot(3,1,2);
plot(t,alpha);
xlabel("Time (s)");
ylabel("α (rad)");
grid on;
subplot(3,1,3);
plot(t,beta);
xlabel("Time (s)");
ylabel("β (rad)");
grid on;
figure;
polarplot(rho,-beta);
title("Trajectory Taken By The Robot")