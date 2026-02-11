%Parameter Initialization
exeTime = 3;
period = 0.01;
iteration = 2;

R = 0.033;
L = 0.287;

t0 = 0;
x = 0;
y = 0;
theta = 0;

% Vectors Initialization
xFH = [0];
yFH = [0];
tFH = [0];
xFL = [0];
yFL = [0];
tFL = [0];
xBH = [0];
yBH = [0];
tBH = [0];
xBL = [0];
yBL = [0];
tBL = [0];
xRR = [0];
yRR = [0];
tRR = [0];
xRL = [0];
yRL = [0];
tRL = [0];
xM = [0];
yM = [0];
tM = [0];

%Forward Motion High Speed
v = 0.2;
w = 0;
while t0 < exeTime
    %x, y, theta Calculations
    x = x + v * period * cos(theta);
    y = y + v * period * sin(theta);
    theta = theta + w * period;
    % Values Recording (x,y,theta)
    xFH(iteration) = x;
    yFH(iteration) = y;
    if rem(iteration,5) == 0
        tFH(floor(iteration / 5)) = theta;
    end

    %Update Parameters
    iteration = iteration + 1;     
    
    t0 = t0 + period;
end

t0 = 0;
x = 0;
y = 0;
theta = 0;
iteration = 2;

v = 0.1;
w = 0;
%Forward Motion Low Speed
while t0 < exeTime
    %x, y, theta Calculations
    x = x + v * period * cos(theta);
    y = y + v * period * sin(theta);
    theta = theta + w * period;
    % Values Recording (x,y,theta)
    xFL(iteration) = x;
    yFL(iteration) = y;
    if rem(iteration,5) == 0
        tFL(floor(iteration / 5)) = theta;
    end

    %Update Parameters
    iteration = iteration + 1;     
    
    t0 = t0 + period;
end

t0 = 0;
x = 0;
y = 0;
theta = 0;
iteration = 2;

v = -0.2;
w = 0;
%Backward Motion High Speed
while t0 < exeTime
    %x, y, theta Calculations
    x = x + v * period * cos(theta);
    y = y + v * period * sin(theta);
    theta = theta + w * period;
    % Values Recording (x,y,theta)
    xBH(iteration) = x;
    yBH(iteration) = y;
    if rem(iteration,5) == 0
        tBH(floor(iteration / 5)) = theta;
    end

    %Update Parameters
    iteration = iteration + 1;     
    
    t0 = t0 + period;
end

t0 = 0;
x = 0;
y = 0;
theta = 0;
iteration = 2;

v = -0.1;
w = 0;
%Backward Motion Low Speed
while t0 < exeTime
    %x, y, theta Calculations
    x = x + v * period * cos(theta);
    y = y + v * period * sin(theta);
    theta = theta + w * period;
    % Values Recording (x,y,theta)
    xBL(iteration) = x;
    yBL(iteration) = y;
    if rem(iteration,5) == 0
        tBL(floor(iteration / 5)) = theta;
    end

    %Update Parameters
    iteration = iteration + 1;     
    
    t0 = t0 + period;
end

t0 = 0;
x = 0;
y = 0;
theta = 0;
iteration = 2;

v = 0.075;
w = 0.09;
%Left Rotation
while t0 < exeTime
    %x, y, theta Calculations
    x = x + v * period * cos(theta);
    y = y + v * period * sin(theta);
    theta = theta + w * period;
    % Values Recording (x,y,theta)
    xRL(iteration) = x;
    yRL(iteration) = y;
    if rem(iteration,5) == 0
        tRL(floor(iteration / 5)) = theta;
    end

    %Update Parameters
    iteration = iteration + 1;     
    
    t0 = t0 + period;
end
t0 = 0;
x = 0;
y = 0;
theta = 0;
iteration = 2;

v = 0.075;
w = -0.09;
%Right Rotation
while t0 < exeTime
    %x, y, theta Calculations
    x = x + v * period * cos(theta);
    y = y + v * period * sin(theta);
    theta = theta + w * period;
    % Values Recording (x,y,theta)
    xRR(iteration) = x;
    yRR(iteration) = y;
    if rem(iteration,5) == 0
        tRR(floor(iteration / 5)) = theta;
    end

    %Update Parameters
    iteration = iteration + 1;     
    
    t0 = t0 + period;
end

t0 = 0;
x = 0;
y = 0;
theta = 0;
iteration = 2;

v = 0.1;
w = 0;
%Mixed Motion
while t0 < exeTime
    %x, y, theta Calculations
    x = x + v * period * cos(theta);
    y = y + v * period * sin(theta);
    theta = theta + w * period;
    % Values Recording (x,y,theta)
    xM(iteration) = x;
    yM(iteration) = y;
    if rem(iteration,5) == 0
        tM(floor(iteration / 5)) = theta;
    end

    %Update Parameters
    iteration = iteration + 1;     
    
    t0 = t0 + period;
end

t0 = 0;
v = 0.05;
w = 0.1;
%Mixed Motion
while t0 < exeTime
    %x, y, theta Calculations
    x = x + v * period * cos(theta);
    y = y + v * period * sin(theta);
    theta = theta + w * period;
    % Values Recording (x,y,theta)
    xM(iteration) = x;
    yM(iteration) = y;
    if rem(iteration,5) == 0
        tM(floor(iteration / 5)) = theta;
    end

    %Update Parameters
    iteration = iteration + 1;     
    
    t0 = t0 + period;
end

t0 = 0;
v = -0.1;
w = -0.05;
%Mixed Motion
while t0 < exeTime
    %x, y, theta Calculations
    x = x + v * period * cos(theta);
    y = y + v * period * sin(theta);
    theta = theta + w * period;
    % Values Recording (x,y,theta)
    xM(iteration) = x;
    yM(iteration) = y;
    if rem(iteration,5) == 0
        tM(floor(iteration / 5)) = theta;
    end

    %Update Parameters
    iteration = iteration + 1;     
    
    t0 = t0 + period;
end