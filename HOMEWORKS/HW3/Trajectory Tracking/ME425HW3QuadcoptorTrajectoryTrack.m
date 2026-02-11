clc;clear;close all;
simTime = 100;
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
%X Coord
P_x = 0.75;
D_x = 1.5;
I_x = 0.075;

%Y Coord
P_y = 0.75;
D_y = 1.1;
I_y = 0.075;

% Altitude
P_z = 0.75;
D_z = 1.1;
I_z = 0.085;

% Roll
P_phi = 0.75;
I_phi   = 0.075;
D_phi   = 0.6;

% Pitch
P_theta = 0.75;
I_theta = 0.075;
D_theta = 0.6;

% Yaw
P_psi = 0.8;
I_psi = 0.05;
D_psi = 1.5;

%Desired And Starting Values
xDesired  = 1;  %Will Be Applied After the robot hovers
yDesired = 0;
zDesired = 2;
psiAngle = 0;
initialAngles = [0.01 0.01 0.01];

%Running the simulation
out = sim("ME425QuadcoptorTrajectoryTrackingV1.slx");
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
title("Position Dynamics");
grid on;
legend;

figure;
plot(t, phi,'DisplayName', "Roll Angle");
hold on;
plot(t, theta,'DisplayName', "Pitch Angle");
plot(t, psi,'DisplayName', "Yaw Angle");
grid on;
title("Attitude Dynamics");
legend;

figure;
hold on;
grid on;
xlabel("Time (s)");
ylabel("Control Effort (N)");
title("Control Efforts");
legend;
plot(t,U1,'DisplayName',"U1");
plot(t,U2,'DisplayName',"U2");
plot(t,U3,'DisplayName',"U3");
plot(t,U4,'DisplayName',"U4");