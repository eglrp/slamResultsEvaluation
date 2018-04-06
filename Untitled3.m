count = ptCloud.Count;
points = ptCloud.Location;
% ang_max = atan(points(1,3)/norm(points(1,1:2),2))*180/pi;
% ang_min = atan(points(count,3)/norm(points(count,1:2),2))*180/pi;
T(1)=2.85*0.0254;
T(2)=5.5*0.0254;
T(3)=2.85*0.0254;
T(4)=3.5*0.0254;

ang_df1 = 0.3387;
ang_df2 = 0.5161;


ang_1_max =2+0.5*ang_df1;
ang_1_min =-8.5-0.5*ang_df1;

ang_2_max = 8+0.5*ang_df2;
ang_2_min = -8-0.5*ang_df2;

scan = cell (1,64);
for i=1:1:64
    scan{i}=[];
end

for i=1:1:ptCloud.Count

    tmp_ang1 = atan((points(i,3)-T(2))/(norm(points(i,1:2),2)-T(1)))*180/pi;
    tmp_ang2 = atan((points(i,3)-T(4))/(norm(points(i,1:2),2)-T(3)))*180/pi+16.9;
    
    if tmp_ang1<ang_1_max && tmp_ang1>ang_1_min
        scanID = ceil((ang_1_max-tmp_ang1)/ang_df1);
       if scanID - (ang_1_max-tmp_ang1)/ang_df1 >0.9 || scanID - (ang_1_max-tmp_ang1)/ang_df1 <0.1
           count= count-1;
           continue
       end
        
    elseif tmp_ang2<ang_2_max && tmp_ang2>ang_2_min
        scanID = ceil((ang_2_max-tmp_ang2)/ang_df2)+32;
        if scanID - 32 - (ang_2_max-tmp_ang2)/ang_df2 >0.9 || scanID - 32 - (ang_2_max-tmp_ang2)/ang_df2 <0.1
           count= count-1;
        continue
       end
    else
            count= count-1;
            continue
    end
    if scanID >64||scanID<1
        count= count-1;
        continue
    end
  
    scan{scanID}=[scan{scanID};points(i,:)];
    
end





% ang_df = (ang_max-ang_min)/63;
% scan = cell (1,64);
% for i=1:1:64
%     scan{i}=[];
% end
% for i=1:1:ptCloud.Count-1
% %     scanID =1;
% %     for j=1:1:64
% %         if atan(points(i,3)/norm(points(i,1:2),2))*180/pi <
%     scanID = (atan(points(i,3)/norm(points(i,1:2),2))*180/pi-ang_min)/ang_df+1;
%     scanID = round(scanID);
%     if scanID >64||scanID<1
%         count= count-1;
%         continue
%     end
%     scan{scanID}=[scan{scanID};points(i,:)];
% end

for j = 1:1:64
    i=65-j;
    scatter3(scan{i}(:,1),scan{i}(:,2),scan{i}(:,3),'Marker','.','LineWidth',0.3);
    hold on
    
end






