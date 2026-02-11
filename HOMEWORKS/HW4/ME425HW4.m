clc; clear; close all;
h = fspecial('gaussian', [15 15], 12.5);

%% Image 1
imgKitchen = imread("IMG1Kitchen.jpg");
imgKitchen = rgb2gray(imgKitchen);
figure;
subplot(3,2,1),
imshow(imgKitchen);
title("Original Image");

imgKitchenFiltered = imfilter(imgKitchen,h);
subplot(3,2,2);
imshow(imgKitchenFiltered);
title("Filtered Image");

subplot(3,2,3);
C = corner(imgKitchen,"Harris", 50);
imshow(imgKitchen);
hold on;
plot(C(:,1), C(:,2), "o");
title("Corner Detection Using Harris Corners On Original Image");

subplot(3,2,4);
CFiltered = corner(imgKitchenFiltered, "Harris", 50);
imshow(imgKitchenFiltered);
hold on;
plot(CFiltered(:,1), CFiltered(:,2), "o");
title("Corner Detection Using Harris Corners On Filtered  Image");

subplot(3,2,5);
C = corner(imgKitchen, "MinimumEigenvalue",50);
imshow(imgKitchen);
hold on;
plot(C(:,1), C(:,2), "o");
title("Corner Detection Using Minimum Eigenvalue On Original Image");

subplot(3,2,6);
CFiltered = corner(imgKitchenFiltered, "MinimumEigenvalue", 50);
imshow(imgKitchenFiltered);
hold on;
plot(CFiltered(:,1), CFiltered(:,2), "o");
title("Corner Detection Using Minimum Eigenvalue On Filtered Image");

%% Image 2
imgHouse = imread("IMG2House.jpg");
imgHouse = rgb2gray(imgHouse);
figure;
subplot(3,2,1),
imshow(imgHouse);
title("Original Image");

imgHouseFiltered = imfilter(imgHouse,h);
subplot(3,2,2);
imshow(imgHouseFiltered);
title("Filtered Image");

subplot(3,2,3);
C = corner(imgHouse, "Harris",50);
imshow(imgHouse);
hold on;
plot(C(:,1), C(:,2), "o");
title("Corner Detection Using Harris Corners On Original Image");

subplot(3,2,4);
CFiltered = corner(imgHouseFiltered, "Harris", 50);
imshow(imgHouseFiltered);
hold on;
plot(CFiltered(:,1), CFiltered(:,2), "o");
title("Corner Detection Using Harris Corners On Filtered  Image");

subplot(3,2,5);
C = corner(imgHouse, "MinimumEigenvalue",50);
imshow(imgHouse);
hold on;
plot(C(:,1), C(:,2), "o");
title("Corner Detection Using Minimum Eigenvalue On Original Image");

subplot(3,2,6);
CFiltered = corner(imgHouseFiltered, "MinimumEigenvalue", 50);
imshow(imgHouseFiltered);
hold on;
plot(CFiltered(:,1), CFiltered(:,2), "o");
title("Corner Detection Using Minimum Eigenvalue On Filtered Image");

%% Image 3
imgTable = imread("Table.jpg");
imgTable = rgb2gray(imgTable);
figure;
subplot(3,2,1),
imshow(imgTable);
title("Original Image");

imgTableFiltered = imfilter(imgTable,h);
subplot(3,2,2);
imshow(imgTableFiltered);
title("Filtered Image");

subplot(3,2,3);
C = corner(imgTable, "Harris",50);
imshow(imgTable);
hold on;
plot(C(:,1), C(:,2), "o");
title("Corner Detection Using Harris Corners On Original Image");

subplot(3,2,4);
CFiltered = corner(imgTableFiltered, "Harris", 50);
imshow(imgTableFiltered);
hold on;
plot(CFiltered(:,1), CFiltered(:,2), "o");
title("Corner Detection Using Harris Corners On Filtered  Image");

subplot(3,2,5);
C = corner(imgTable, "MinimumEigenvalue",50);
imshow(imgTable);
hold on;
plot(C(:,1), C(:,2), "o");
title("Corner Detection Using Minimum Eigenvalue On Original Image");

subplot(3,2,6);
CFiltered = corner(imgTableFiltered, "MinimumEigenvalue", 50);
imshow(imgTableFiltered);
hold on;
plot(CFiltered(:,1), CFiltered(:,2), "o");
title("Corner Detection Using Minimum Eigenvalue On Filtered Image");

%% Image 4
imgFarm = imread("Farm.jpeg");
imgFarm = rgb2gray(imgFarm);
figure;
subplot(3,2,1),
imshow(imgFarm);
title("Original Image");

imgFarmFiltered = imfilter(imgFarm,h);
subplot(3,2,2);
imshow(imgFarmFiltered);
title("Filtered Image");

subplot(3,2,3);
C = corner(imgFarm, "Harris",50);
imshow(imgFarm);
hold on;
plot(C(:,1), C(:,2), "o");
title("Corner Detection Using Harris Corners On Original Image");

subplot(3,2,4);
CFiltered = corner(imgFarmFiltered, "Harris", 50);
imshow(imgFarmFiltered);
hold on;
plot(CFiltered(:,1), CFiltered(:,2), "o");
title("Corner Detection Using Harris Corners On Filtered  Image");

subplot(3,2,5);
C = corner(imgFarm, "MinimumEigenvalue",50);
imshow(imgFarm);
hold on;
plot(C(:,1), C(:,2), "o");
title("Corner Detection Using Minimum Eigenvalue On Original Image");

subplot(3,2,6);
CFiltered = corner(imgFarmFiltered, "MinimumEigenvalue", 50);
imshow(imgFarmFiltered);
hold on;
plot(CFiltered(:,1), CFiltered(:,2), "o");
title("Corner Detection Using Minimum Eigenvalue On Filtered Image");

%% Image 5
imgClassroom = imread("Classroom.jpeg");
imgClassroom = rgb2gray(imgClassroom);
figure;
subplot(3,2,1),
imshow(imgClassroom);
title("Original Image");

imgClassroomFiltered = imfilter(imgClassroom,h);
subplot(3,2,2);
imshow(imgClassroomFiltered);
title("Filtered Image");

subplot(3,2,3);
C = corner(imgClassroom, "Harris",50);
imshow(imgClassroom);
hold on;
plot(C(:,1), C(:,2), "o");
title("Corner Detection Using Harris Corners On Original Image");

subplot(3,2,4);
CFiltered = corner(imgClassroomFiltered, "Harris", 50);
imshow(imgClassroomFiltered);
hold on;
plot(CFiltered(:,1), CFiltered(:,2), "o");
title("Corner Detection Using Harris Corners On Filtered  Image");

subplot(3,2,5);
C = corner(imgClassroom, "MinimumEigenvalue",50);
imshow(imgClassroom);
hold on;
plot(C(:,1), C(:,2), "o");
title("Corner Detection Using Minimum Eigenvalue On Original Image");

subplot(3,2,6);
CFiltered = corner(imgClassroomFiltered, "MinimumEigenvalue", 50);
imshow(imgClassroomFiltered);
hold on;
plot(CFiltered(:,1), CFiltered(:,2), "o");
title("Corner Detection Using Minimum Eigenvalue On Filtered Image");