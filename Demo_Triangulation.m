%% Demo Code for Morphing function based on Traiangulation
%% Code Written by Nitin J. Sanket (nitinsan@seas.upenn.edu)
%% MSE in Robotics Student, University of Pennsylvania
%% Project 2: CIS581 Computer Vision
% This function also creates a video and saves the frames
clc
clear all
close all

%% Task 1: DEFINING CORRESPONDENCES AND TRIANGULATION
I1 = im2double(imread('Nitin.jpg'));
I2 = im2double(imread('PB2.jpg'));
%% Resize image to same size
if(numel(I1)>numel(I2)) % im1 is bigger than im2
    I1 = imresize(I1, [size(I2,1),size(I2,2)], 'bicubic');
else % im2 is bigger than im1
    I2 = imresize(I2, [size(I1,1),size(I1,2)], 'bicubic');
end

[im1_pts, im2_pts] = click_correspondences(I1, I2);


figure,
imshow(I1);
hold on;
for i = 1:size(im1_pts,1)
    plot(im1_pts(i,1), im1_pts(i,2), 'r*');
end
hold off;

figure,
imshow(I2);
hold on;
for i = 1:size(im2_pts,1)
    plot(im2_pts(i,1), im2_pts(i,2), 'r*');
end
hold off;

%% Perform Triangulation using Delaunay
% Compute the average shape, i.e., mean of the two point sets
% Shape B - the average shape
AvgShape  = 0.5*im1_pts + 0.5*im2_pts;
% tri = delaunayTriangulation(AvgShape);
tri = delaunay(AvgShape(:,1), AvgShape(:,2));

figure,
imshow(I1);
hold on;
triplot(tri, AvgShape(:,1), AvgShape(:,2),'r');
hold off;

figure,
imshow(I2);
hold on;
triplot(tri, AvgShape(:,1), AvgShape(:,2),'r');
hold off;
% pause;

% figure,
t = linspace(0,1,60);
for i = 1:numel(t)
    warp_frac = t(i);
    dissolve_frac = t(i);
    tic
    disp(['Iteration: ',num2str(i)]);
    morphed_im = morph(I1, I2, im1_pts, im2_pts, tri, warp_frac, dissolve_frac);
    toc
    figure,
    imshow(mat2gray(morphed_im));
    pause(0.0001);
end
