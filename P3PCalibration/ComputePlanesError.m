function e = ComputePlanesError(T,PIcam,PIdep)
N_PLANES = size(PIcam,2);
TP = [[T(1:3,1:3); -T(1:3,4).'*T(1:3,1:3)] [0;0;0;1]];
e = zeros(1,N_PLANES);
for n=1:N_PLANES
    PIdepP = TP*PIcam(:,n);
    PIdepP = PIdepP(1:3)/PIdepP(4);
    e(n)  = sqrt(sum((PIdepP-PIdep(1:3,n)).^2));
end