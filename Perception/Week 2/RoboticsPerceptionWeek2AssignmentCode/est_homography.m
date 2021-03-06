function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course
%logo_pts=[0,0;990,0;990,400;0,400];
%video_pts=[141.5360  175.3260
  %293.2480  142.6483
  %295.4956  219.9856
  %144.9074  261.3775];

% YOUR CODE HERE
x=[];
x=video_pts;
lx=[];
lx=logo_pts;
ax=[];
ay=[];
A=[];
r=1;
for i=1:4
    ax(i,:)=[-x(i,1),-x(i,2),-1,0,0,0,x(i,1)*lx(i,1),x(i,2)*lx(i,1),lx(i,1)];
    ay(i,:)=[0,0,0,-x(i,1),-x(i,2),-1,x(i,1)*lx(i,2),x(i,2)*lx(i,2),lx(i,2)];
    A(r,:)=[ax(i,:)];
    A(r+1,:)=[ay(i,:)];
    r=r+2;
end
 
[U,S,V] = svd(A);
 H1 = V(:,9);
 H2=reshape(H1,3,3);
 H=H2;
end
