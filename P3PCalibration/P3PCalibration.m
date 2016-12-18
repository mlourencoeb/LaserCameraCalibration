function [best_T, best_inliers] = P3PCalibration(PIcam,PIdep,Points,threshold)
% TODO : this can be largely improved in terms of computational efficiency
N = size(PIcam,2);
min_cost = inf;
best_inliers = [];
inliersRatio = 0.75;
plane_set = nchoosek(N,3);
planePoints=[];
n_planePoints=[];

for i = 1:size(PIcam,2)
    planePoints=[planePoints; Points{i}'];
    n=size(Points{i},2);
    n_planePoints=[n_planePoints ; n];
end

t1=(PIcam(1:3,:)*PIdep(1:3,:)')\PIcam(1:3,:)*(PIcam(4,:)-PIdep(4,:))';
PIcamnorm = PIcam ./ repmat(PIcam(4,:),4,1);
PIdepnorm = PIdep ./ repmat(PIdep(4,:),4,1);

for i = 1:1000
    index = randsample(N, 3);
    R = rotation3Pt(PIcam(1:3,index), PIdep(1:3,index), t1,0); %CHANGED
    error = nan(1,N);
    for m = 1:size(R,3)
        error(m,:) = ComputeRMSWeightDistance(R(:,:,m), -t1, planePoints,n_planePoints,PIdep(1:3,:),PIdep(4,:));
        %ComputeW([R(:,:,m) t1; 0 0 0 1], PIcamnorm, PIdepnorm);
        inliers    = find(error(m,:) < threshold);
        cost       = sum(error(m,inliers).^2) + threshold^2*(size(error,2)-length(inliers));
        cost = cost / length(inliers);
        if cost < min_cost
            min_cost     = cost;
            best_inliers = inliers;
            best_R       = R(:,:,m);
        end
    end
    if length(best_inliers)/N > inliersRatio
        break;
    end
end

best_T = [best_R -t1; 0 0 0 1];


% best_T = DepthCam3PlaneCalibNewRegist2(PIcam(:,best_inliers), PIdep(:,best_inliers)); %CHANGED
% save Depth_RegAll.mat A