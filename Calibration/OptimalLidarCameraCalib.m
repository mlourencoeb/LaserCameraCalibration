function [T, n_planePoints] = OptimalLidarCameraCalib(Ti, CameraPlanePoses, LidarPlanes, Points, newMethod)
warning off;
thetac = [];
alphac=[];
for i = 1:size(CameraPlanePoses,3)
    theta=CameraPlanePoses(1:3,3,i);
    alpha=CameraPlanePoses(1:3,4,i)'*theta;
    if(alpha<0)
        theta=-theta; alpha=-alpha;
    end
    thetac=[thetac theta/alpha]; alphac=[alphac alpha/alpha];
end

thetal=[]; alphal=[];
planePoints=[];
n_planePoints=[];

for i = 1:size(LidarPlanes,2)
    alphal=[alphal LidarPlanes(4,i)/LidarPlanes(4,i)];
    thetal=[thetal LidarPlanes(1:3,i)/LidarPlanes(4,i)];
    % inliers in selected plane points
    planePoints=[planePoints; Points{i}'];
    % number of inliers in selected plane points
    n=size(Points{i},2);
    n_planePoints=[n_planePoints ; n];
    
end

q=rot2quat(Ti(1:3,1:3));
par=[q ; Ti(1:3,4)];


cost=computeRMSWeightedDistVerticesToPlanes(par,planePoints,n_planePoints,...
    thetac,alphac);

% Uses optimization toolbox
if(newMethod == 0)
    [par_est, fval, exitFlag,output]=fminunc(@computeRMSWeightedDistVerticesToPlanes,...
        par,[],planePoints,n_planePoints,thetac,alphac);
else
    options = optimset('Algorithm','levenberg-marquardt','MaxIter',1000,'TolFun', 10^-6);
    [par_est, fval, exitFlag,output]=fminsearch(...
        @computeRMSWeightedDistVerticesToPlanes,...
        par,options,planePoints,n_planePoints,thetac,alphac);
end
cost=computeRMSWeightedDistVerticesToPlanes(par_est,planePoints,...
    n_planePoints,thetac,alphac);
% fprintf(1,'RMS distance of points to planes after search: %f\n',cost);

% Convert back to rotation matrix
R2=quat2rot(par_est(1:4));
t2=par_est(5:7);

T = [R2 t2; 0 0 0 1];

warning on;
end

