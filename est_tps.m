%% Estimate TPS Parameters for TPS based Morphing
%% Code Written by Nitin J. Sanket (nitinsan@seas.upenn.edu)
%% MSE in Robotics Student, University of Pennsylvania
%% Project 2: CIS581 Computer Vision
function [a1,ax,ay,w] = est_tps(ctr_pts, target_value)
lambda = 0.001; % Lambda
p = size(ctr_pts, 1); % p = Number of points
TempX = repmat(ctr_pts(:,1), 1, p);
TempY = repmat(ctr_pts(:,2), 1, p);
TempXDiff = TempX - transpose(TempX);
TempYDiff = TempY - transpose(TempY);
%% Calculate K matrix
K = TempXDiff.*TempXDiff + TempYDiff.*TempYDiff;
Index = (K~=0);
K = -1.*K.*log(K);
K(Index==0) = 0;

%% Calculate P matrix
P = [ones(p,1), ctr_pts];
% Get A matrix
A = [[K P];[transpose(P), zeros(3)]];
A = A + lambda*eye(p+3);
v = [target_value; zeros(3, 1)];

%% Calculate TPS Coefficients
TPSCoeff = A\v;

%% Assign to return values
w = TPSCoeff(1:p);
a1 = TPSCoeff(p+1); 
ax = TPSCoeff(p+2); 
ay = TPSCoeff(p+3); 
end