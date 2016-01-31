%% Wrapper Code for Morphing function based on TPS
%% Code Written by Nitin J. Sanket (nitinsan@seas.upenn.edu)
%% MSE in Robotics Student, University of Pennsylvania
%% Project 2: CIS581 Computer Vision
function morphed_im = morph_tps_wrapper(im1, im2, im1_pts, im2_pts, warp_frac, dissolve_frac)
%% Make Images Same Size
I1 = im1;
I2 = im2;
if(size(I1,1)~=size(I2,1))||(size(I1,2)~=size(I2,2))
    if(numel(I1)>numel(I2)) % im1 is bigger than im2
        I1 = imresize(I1, [size(I2,1),size(I2,2)], 'bicubic');
    else % im2 is bigger than im1
        I2 = imresize(I2, [size(I1,1),size(I1,2)], 'bicubic');
    end
end
I1 = im2double(I1);
I2 = im2double(I2);
%% Find Points on Intermediate Image
BPoints = (1 - warp_frac)*im1_pts + warp_frac*im2_pts;

%% TPS
[a1_x,ax_x,ay_x,w_x] = est_tps(BPoints, im1_pts(:,1));
[a1_y,ax_y,ay_y,w_y] = est_tps(BPoints, im1_pts(:,2));
Target1 = morph_tps(I1, a1_x, ax_x, ay_x, w_x, a1_y, ax_y, ay_y, w_y, BPoints, [size(I1,1), size(I1,2)]);
[a1_x,ax_x,ay_x,w_x] = est_tps(BPoints, im2_pts(:,1));
[a1_y,ax_y,ay_y,w_y] = est_tps(BPoints, im2_pts(:,2));
Target2 = morph_tps(I2, a1_x, ax_x, ay_x, w_x, a1_y, ax_y, ay_y, w_y, BPoints, [size(I2,1), size(I2,2)]);

%% Dissolve two warpped images
morphed_im = (1-dissolve_frac)*Target1 + dissolve_frac*Target2;
morphed_im = (morphed_im);

% figure,
% imshow(morphed_im);
end