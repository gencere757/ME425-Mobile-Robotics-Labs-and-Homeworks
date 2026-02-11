imgMsg = receive(imgSub, 10);
img = rosReadImage(imgMsg);
I = rgb2gray(img);
[rows, cols] = size(I);

%Crop to Center (Region of Interest)
roiPercentage = 0.4;
r_min = floor(rows*(1-roiPercentage)/2); r_max = floor(rows*(1+roiPercentage)/2);
c_min = floor(cols*(1-roiPercentage)/2); c_max = floor(cols*(1+roiPercentage)/2);
I_roi = I(r_min:r_max, c_min:c_max);
corners = detectHarrisFeatures(I_roi);
corners = corners.selectStrongest(20);
if corners.Count == 4
    objectDetected = true;
    %Offset corners back to original image coordinates,
    X = corners.Location(:,1) + c_min; Y = corners.Location(:,2) + r_min;

    %Now you have x and y coordinates of the detected corners (as vectors)
    imshow(img); hold on;
    %Plot the rectangle in red
    rectangle('Position', [min(X), min(Y), max(X)-min(X), max(Y)-min(Y)], ...
        'EdgeColor', 'r', 'LineWidth', 2);
    %Plot the right edge in green
    line([max(X), max(X)], [min(Y), max(Y)], 'Color', 'g', 'LineWidth', 2);
    %Calculate the length of the right edge
    rightEdgeLength = r_max - r_min;
    
    hold off;
else
    objectDetected = false;
    imshow(img);
end

