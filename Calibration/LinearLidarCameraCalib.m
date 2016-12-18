function [T, n_planePoints] = LinearLidarCameraCalib(CameraPlanePoses, LidarPlanes, Points)
thetac = [];
alphac=[];
for i = 1:size(CameraPlanePoses,3)
    theta=CameraPlanePoses(1:3,3,i);
        alpha=CameraPlanePoses(1:3,4,i)'*theta;
    if(alpha<0)
            theta=-theta; alpha=-alpha;
    end
    thetac=[thetac theta]; alphac=[alphac alpha];
end

% Conversion from mm (MCCT fomat) to metres (ladar data format)
% alphac=alphac.*1.0e-3;

thetal=[]; alphal=[];
planePoints=[];
n_planePoints=[];

for i = 1:size(LidarPlanes,2)
    alphal=[alphal LidarPlanes(4,i)];
    thetal=[thetal LidarPlanes(1:3,i)];
    % inliers in selected plane points
    planePoints=[planePoints; Points{i}'];
    % number of inliers in selected plane points
    n=size(Points{i},2);
    n_planePoints=[n_planePoints ; n];
    
end

% Closed form solution for translation vector
t1=(thetac*thetac')\thetac*(alphac-alphal)';

% fprintf(1,'Computed error in distance to planes: %f\n',...
%     computeRMSDiffDistanceToPlanes(t1,thetac,alphac,thetal,alphal));

% Computing best rotation
% camera_normals(Q)= R * laser_normals(P)
[U,S,V]=svd(thetal*thetac');
R=V*U';
if(det(R)<0)
    R = V * diag([ ones(size(V,2)-1, 1) ; -1]) * U';
end

T = [R t1; 0 0 0 1];

end