function loss = fminPoints(points,T)

loss = 0;

    for i=1:1:size(points,1)
        
%         tmp(i) = atan((points4(i,3)-T(3))/norm(points4(i,1:2)-T(1:2),2))-(2-(i-1)*0.3387)/180*pi;
          tn = (points(i,3)-T(2))/(norm(points(i,1:2),2)-T(1));
           ang = atan(tn)*180/pi;
%          ang = atan((points(i,3)-T(2))/(norm(points(i,1:2),2)-T(1)))*180/pi;
        tmp = abs(ang-(2-(i-1)*0.3387));

        loss = loss+tmp;
    end



end