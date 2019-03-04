function [ warped_pts ] = warp_pts( video_pts, logo_pts, sample_pts)
% warp_pts computes the homography that warps the points inside
% video_pts to those inside logo_pts. It then uses this
% homography to warp the points in sample_pts to points in the logo
% image
% Inputs:
%     video_pts: a 4x2 matrix of (x,y) coordinates of corners in the
%         video frame
%     logo_pts: a 4x2 matrix of (x,y) coordinates of corners in
%         the logo image
%     sample_pts: a nx2 matrix of (x,y) coordinates of points in the video
%         video that need to be warped to corresponding points in the
%         logo image
% Outputs:
%     warped_pts: a nx2 matrix of (x,y) coordinates of points obtained
%         after warping the sample_pts
% Written for the University of Pennsylvania's Robotics:Perception course
[ H ] = est_homography(video_pts, logo_pts);

% YOUR CODE HERE
a=[];
a=[sample_pts ones(size(sample_pts,1),1)];
%warped_pts = [sample_pts ones(size(sample_pts, 1), 1)]*H;
w=a*H;
w(:, 1) = w(:, 1)./w(:, 3);
w(:, 2) = w(:, 2)./w(:, 3);
warped_pts = w(:, [1 2]);
end
