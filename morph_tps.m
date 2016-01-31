%% Morphing function for Morphing using TPS
%% Code Written by Nitin J. Sanket (nitinsan@seas.upenn.edu)
%% MSE in Robotics Student, University of Pennsylvania
%% Project 2: CIS581 Computer Vision
function morphed_im = morph_tps(im_source, a1_x, ax_x, ay_x, w_x, a1_y, ax_y, ay_y, w_y, ctr_pts, sz)
if(sz(1)~=size(im_source,1))||(sz(2)~=size(im_source,2))
    im_source = imresize(im_source, [sz(1) sz(2)], 'bicubic');
end
%% Initialize necessary variables
Points = [zeros(2, sz(1)*sz(2)); ones(1, sz(1)*sz(2))];
p = size(ctr_pts, 1); % Number of control points
morphed_im = zeros(sz(1), sz(2), 3); % Assuming RGB Image
TempK = [repmat(1:sz(2), 1, sz(1)); reshape(repmat(1:sz(1), sz(2), 1), [1 sz(1)*sz(2)]);];

%% Calculate TPS Parameters
Diff = @(vector, ctrl_pts_col) bsxfun(@minus, ctrl_pts_col, repmat(vector, p, 1));
RSquare = Diff(TempK(1,:), ctr_pts(:,1)).^2 + Diff(TempK(2,:), ctr_pts(:,2)).^2;
Index = (RSquare~=0);
U = -RSquare.*log(RSquare);
U(Index==0) = 0;

Points(1:2, :)  = bsxfun(@plus, [a1_x ;a1_y], [ax_x, ay_x; ax_y, ay_y]*TempK + [w_x'; w_y']*U);

%% Bring Points back to legal limits
Points = round(bsxfun(@rdivide, Points, Points(end, :)));
Points = bsxfun(@min, bsxfun(@max, Points, [1 1 1]'), [sz(2), sz(1), 1]');

%% Find Morphed Image
for i = 1 : sz(1)*sz(2)
    morphed_im(TempK(2,i), TempK(1,i), :) = im_source(Points(2, i), Points(1, i), :);
end

end