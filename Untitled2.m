minfun = @(T)fminPoints(points4,T);
T = fminsearch(minfun,[0,0],optimset('MaxFunEvals',40000,'MaxIter',40000,...
                         'Display','iter','Algorithm','levenberg-marquardt','ToLX',1e-10,'TolFun',1e-10));

% points_ = points2;
%  points_ = points6-T;     
points_ = points6;
T(1)=2.85*0.0254;
T(2)=5.5*0.0254;
% T(1)=2.85*0.0254;
% T(2)=3.5*0.0254;

                     
for i = 1:1:size(points_,1)
%    tmp1 = atan (abs(points_(i,3))/norm(points_(i,1:2)));
%    tmp2 = atan (abs(points_(i+1,3))/norm(points_(i+1,1:2)));
%    dis_ang(i)=(tmp1-tmp2)/pi*180; 
   ang(i) = atan((points_(i,3)-T(2))/(norm(points_(i,1:2),2)-T(1)))*180/pi;
    
    
end
ang = ang+16.9;

scan = cell(1,64);
count = ptCloud.Count;
for i = 1:1:ptCloud.Count
    point = ptCloud.Location(i,:);
    ang = atan(point(3)/(point(1)^2+point(2)^2)^0.5);
    
    

