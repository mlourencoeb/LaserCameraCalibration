% This files show how to use the methods included in this repository
% 
% Data was previously processed to use the methods.
% Planes matrix load from LeftLidarCalibration and RightLidarCalibration
% are obtained from Bouguet plane poses.
% LidarPlanes was manually segmented to compute
% the planes for each frame using Points struct;

load('Calib_Results_stereo.mat');
GT = [R T/1000; 0 0 0 1];
GT = inv(GT);

% run right view
load('Calib_Results_Right.mat')
load RightLidarCalibration.mat

%% 1st method
[Ti, ~] = LinearLidarCameraCalib(Planes, LidarPlanes, Points);
Tor = OptimalLidarCameraCalib(Ti,Planes, LidarPlanes, Points, 0);

%2nd method
thetac = [];
alphac=[];
for i = 1:size(Planes,3)
    theta=Planes(1:3,3,i);
    alpha=Planes(1:3,4,i)'*theta;
    if(alpha<0)
        theta=-theta; alpha=-alpha;
    end
    thetac=[thetac theta]; alphac=[alphac alpha];
end

planes = [thetac;alphac] ;
[Ti, inliers] = P3PCalibration(LidarPlanes,planes, Points, 0.0001)
[Tp3pr, ~] = OptimalLidarCameraCalib(Ti,Planes(:,:, inliers), LidarPlanes(:, inliers), Points(inliers),0);

%% run left view
load('Calib_Results_Left.mat')
load LeftLidarCalibration.mat

[Ti, ~] = LinearLidarCameraCalib(Planes, LidarPlanes, Points);
Tol = OptimalLidarCameraCalib(Ti,Planes, LidarPlanes, Points, 0);

%2nd method
thetac = [];
alphac=[];
for i = 1:size(Planes,3)
    theta=Planes(1:3,3,i);
    alpha=Planes(1:3,4,i)'*theta;
    if(alpha<0)
        theta=-theta; alpha=-alpha;
    end
    thetac=[thetac theta]; alphac=[alphac alpha];
end
[Ti, inliers] = P3PCalibration(LidarPlanes,planes, Points, 0.0001)
[Tp3pl, ~] = OptimalLidarCameraCalib(Ti,Planes(:,:, inliers), LidarPlanes(:, inliers), Points(inliers),0);

%% compare the results against GT stereo transformation
Tstereo = Tol*inv(Tor)
Tstereop3p = Tp3pl*inv(Tp3pr)
[te1, re1] = CompareTs(GT, Tstereo);
[te2, re2] = CompareTs(GT, Tstereop3p);

fprintf('First method error in rotation = %f, and second one = %f.\n', re1, re2);
fprintf('First method error in translation = %f, and second one = %f.\n', te1, te2);


