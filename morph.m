%% Morphing function for Traiangulation based Morphing
%% Code Written by Nitin J. Sanket (nitinsan@seas.upenn.edu)
%% MSE in Robotics Student, University of Pennsylvania
%% Project 2: CIS581 Computer Vision
function morphed_im = morph(im1, im2, im1_pts, im2_pts, tri, warp_frac, dissolve_frac)
I1 = im2double(im1);
I2 = im2double(im2);
if(size(I1,1)~=size(I2,1))||(size(I1,2)~=size(I2,2))
    if(numel(I1)>numel(I2)) % im1 is bigger than im2
        I1 = imresize(I1, [size(I2,1),size(I2,2)], 'bicubic');
    else % im2 is bigger than im1
        I2 = imresize(I2, [size(I1,1),size(I1,2)], 'bicubic');
    end
end

%% Task 2: IMAGE MORPH VIA TRIANGULATION
BPoints = (1-warp_frac)*im1_pts + warp_frac*im2_pts;
[m, n, ~] = size(I1); % x and y with r and c swapped
for c = 1:n
    for r = 1:m
        t(c, r) = tsearchn(BPoints, tri, [c, r]);
    end
end

% figure,
% imagesc(t);
%% Barycentric Co-ordinates of Intermediate Image
NTri = size(tri,1);
for i = 1:NTri
    Ainv(:,:,i) = inv([BPoints(tri(i,1),:)', BPoints(tri(i,2),:)', BPoints(tri(i,3),:)'; [1 1 1]]);
end

% c is x and r is y in MATLAB
BarycentricCoord = zeros(m,n,3);
for c = 1:n
    for r = 1:m
        if(~isnan(t(r,c)))
            Temp = Ainv(:,:,t(c,r))*[c r 1]';
            BarycentricCoord(r,c,1) = Temp(1);
            BarycentricCoord(r,c,2) = Temp(2);
            BarycentricCoord(r,c,3) = Temp(3);
        end
    end
end

%% Barycentric Co-ordinates of Source Image
NTri = size(tri,1);
for i = 1:NTri
    ASource1(:,:,i) = ([im1_pts(tri(i,1),:)', im1_pts(tri(i,2),:)', im1_pts(tri(i,3),:)'; [1 1 1]]);
    ASource2(:,:,i) = ([im2_pts(tri(i,1),:)', im2_pts(tri(i,2),:)', im2_pts(tri(i,3),:)'; [1 1 1]]);
end

[m, n, o] = size(t);
Coord1 = zeros(m, n, 3);
Coord2 = zeros(m, n, 3);
% c is x and r is y in MATLAB
for c = 1:n
    for r = 1:m
        if(~isnan(t(r,c)))
            Temp1 = ASource1(:,:,t(c,r))*squeeze(BarycentricCoord(r,c,:));
            Temp2 = ASource2(:,:,t(c,r))*squeeze(BarycentricCoord(r,c,:));
            Coord1(r,c,2) = Temp1(1)./Temp1(3);
            Coord1(r,c,1) = Temp1(2)./Temp1(3);
            Coord2(r,c,2) = Temp2(1)./Temp2(3);
            Coord2(r,c,1) = Temp2(2)./Temp2(3);
        end
    end
end

Target1 = zeros(m,n,3);
Target2 = zeros(m,n,3);
for c = 1:n
    for r = 1:m
        if(round(Coord1(r,c,1))~=0 && round(Coord1(r,c,2))~=0 && round(Coord2(r,c,1))~=0 && round(Coord2(r,c,2))~=0)
            Target1(r,c,:) = I1(round(Coord1(r,c,1)), round(Coord1(r,c,2)),:);
            Target2(r,c,:) = I2(round(Coord2(r,c,1)), round(Coord2(r,c,2)),:);
        end
    end
end

% figure,
% imshow(Target1);
% figure,
% imshow(Target2);
morphed_im = (1-dissolve_frac)*Target1 + dissolve_frac*Target2;
% morphed_im = im2double(morphed_im);
% figure,
% imagesc(morphed_im);
% pause;
end
