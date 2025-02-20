%% Demo Code for Morphing function based on Thin Plate Spline
%% Code Written by Nitin J. Sanket (nitinsan@seas.upenn.edu)
%% MSE in Robotics Student, University of Pennsylvania
%% Project 2: CIS581 Computer Vision
% This code also creates a video
clc
clear all
close all

%% Task 1: DEFINING CORRESPONDENCES AND TRIANGULATION
load PB2
% load im2_pts
% I1 = im2double(imread('Nitin.jpg'));
% I2 = im2double(imread('Einstein4.jpg'));
%% Resize image to same size
% if(numel(I1)>numel(I2)) % im1 is bigger than im2
%     I1 = imresize(I1, [size(I2,1),size(I2,2)], 'bicubic');
% else % im2 is bigger than im1
%     I2 = imresize(I2, [size(I1,1),size(I1,2)], 'bicubic');
% end
% I1 = im1;
% I2 = im2;

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
% [im1_pts, im2_pts] = click_correspondences(I1, I2);
h_avi = avifile('TriangulationTPS.avi','fps',10);
%% Perform TPS
% Compute the average shape, i.e., mean of the two point sets
% AvgShape - the average shape
AvgShape  = 0.5*im1_pts + 0.5*im2_pts;

t = linspace(0,1,60);
for i = 1:numel(t)
    warp_frac = t(i);
    dissolve_frac = t(i);
    tic
    disp(['Iteration: ',num2str(i)]);
    morphed_im = morph_tps_wrapper(I1, I2, im1_pts, im2_pts, warp_frac, dissolve_frac);
    toc
    figure,
    imshow(mat2gray(morphed_im));
    imwrite(morphed_im,[pwd,'\Frames\',num2str(i),'.jpg']);
    h_avi = addframe(h_avi, morphed_im);
    pause(0.0001);
end
h_avi = close(h_avi); clear h_avi;
