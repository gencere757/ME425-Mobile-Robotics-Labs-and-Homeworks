% Calculate TTC
TTC = min(100,l1 * deltaT / abs(l1 - l2));
%Save TTC to the array
TTCArray = [TTCArray, TTC];
% Move the robot according to Test cases
if TTC < 10
    goBackDuration = 4;
    timer = tic;
    while toc(timer) < goBackDuration
        velMsg.linear.x = -0.05;
    end
else
    velMsg.linear.x = 0.05;  % Normal speed
end
send(velPub, velMsg);