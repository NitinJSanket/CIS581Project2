%% Clock Correspondences for Traiangulation based Morphing
%% Code Written by Nitin J. Sanket (nitinsan@seas.upenn.edu)
%% MSE in Robotics Student, University of Pennsylvania
%% Project 2: CIS581 Computer Vision
function [im1_pts, im2_pts] = click_correspondences(im1,im2)
[im1_pts, im2_pts] = cpselect(im1, im2, 'Wait',true); % (Moving, Fixed)
save im1_pts
save im2_pts