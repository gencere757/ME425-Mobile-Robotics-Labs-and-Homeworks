clc;clear;close all;
simTime = 60;
%System Parameters
Ix = 0.02; 
Iy = 0.02; 
Iz = 0.04;  
m = 1;
b = 1e-3;
l = 0.35;
d = 1e-4;
g = 9.81;
%Controller Parameters
% Altitude
P_z = 0.75;
I_z = 0.1;
D_z = 2;

% Roll
P_phi = 0.5;
I_phi   = 0.1;
D_phi   = 0.95;

% Pitch
P_theta = 0.5;
I_theta = 0.1;
D_theta = 0.95;

% Yaw
P_psi = 0.5;
I_psi = 0.05;
D_psi = 0.95;

%Desired And Starting Values
xDesired  = 1;
yDesired = 1;
zDesired = 2;
initialAngles = [0 0 0];

%Running the simulation
out = sim("ME425QuadcoptorMotionControlV4");
%Getting the results
t = out.tout;
X = out.X;
Y = out.Y;
Z = out.Z;
phi = out.roll;
theta = out.pitch;
psi = out.yaw;
U1 = out.U1;
U2 = out.U2;
U3 = out.U3;
U4 = out.U4;

%Plotting
figure;
plot(t,X, 'DisplayName', "X Coordinate");
hold on;
plot(t,Y, 'DisplayName', 'Y Coordinate');
plot(t,Z, 'DisplayName', 'Z Coordinate');
title("Altitude");
xlabel("Time (s)");
ylabel("Coordinate (m)");
grid on;
legend;

figure;
plot(t, phi,'DisplayName', "Roll Angle");
hold on;
plot(t, theta,'DisplayName', "Pitch Angle");
plot(t, psi,'DisplayName', "Yaw Angle");
xlabel("Time (s)");
ylabel("Angle (rad)");
grid on;
legend;

figure;
hold on;
grid on;
xlabel("Time (s)");
ylabel("Control Effort (N)");
legend;
plot(t,U1,'DisplayName',"U1");
plot(t,U2,'DisplayName',"U2");
plot(t,U3,'DisplayName',"U3");
plot(t,U4,'DisplayName',"U4");
