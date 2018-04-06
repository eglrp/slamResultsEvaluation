% temp = pose9(:,6);
% pose9(:,5:6) = pose9(:,4:5);
% pose9(:,4)   = temp; 
[FileName,PathName] = uigetfile('*.txt','Select the groundtruth Txt files');
fid=fopen([PathName FileName]);
temp=textscan(fid,'%f %f %f %f %f %f %f %f %f %f %f %f ');
groundtruth = zeros(length(temp{1}),12);
fclose(fid);

for j =1 :1:size(temp,2)
    groundtruth(:,j) = temp{j};
end

% 
% [FileName,PathName] = uigetfile('*.txt','Select the evaluation Txt files');
% fid=fopen([PathName FileName]);
% temp=textscan(fid,'%f %f %f %f %f %f %f %f %f %f %f %f ');
% pose = zeros(length(temp{1}),12);
% fclose(fid);
% 
% for j =1 :1:size(temp,2)
%     pose(:,j) = temp{j};
% end

[FileName,PathName] = uigetfile('*.txt','Select the evaluation Txt files');
fid=fopen([PathName FileName]);
temp=textscan(fid,'%f %f %f %f %f %f  ');
pose = zeros(length(temp{1}),6);
fclose(fid);

for j =1 :1:size(temp,2)
    pose(:,j) = temp{j};
end
        

%01
Rl2c=[7.967514e-03 -9.999679e-01 -8.462264e-04;-2.771053e-03 8.241710e-04 -9.999958e-01;9.999644e-01 7.969825e-03 -2.764397e-03];
tl2c=[-1.377769e-02; -5.542117e-02 ;-2.918589e-01];

% %06
% Rl2c=[7.027555e-03 -9.999753e-01 2.599616e-05;-2.254837e-03 -4.184312e-05 -9.999975e-01;9.999728e-01 7.027479e-03 -2.255075e-03];
% tl2c=[-7.137748e-03; -7.482656e-02; -3.336324e-01];

%  %09
% Rl2c=[7.027555e-03 -9.999753e-01 2.599616e-05;-2.254837e-03 -4.184312e-05 -9.999975e-01;9.999728e-01 7.027479e-03 -2.255075e-03];
% tl2c=[-7.137748e-03; -7.482656e-02; -3.336324e-01];

num = size(pose,1);
if size(groundtruth,1)>num
    groundtruth(num+1:end,:)=[];
end

evaluationMatrix = zeros (num,12);
Rtli2li = zeros (num,12);
for i=1:1:num
    %变换过坐标的激光雷达旋转矩阵
    R1 = [1 ,0 ,0;
        0 , cos(pose(i,1)), -sin(pose(i,1));
        0 , sin(pose(i,1)), cos(pose(i,1))];
    R2 = [cos(pose(i,2)), 0,  sin(pose(i,2));
                0     ,    1          , 0;
         -sin(pose(i,2)), 0,  cos(pose(i,2)) ];
    R3 = [cos(pose(i,3)), -sin(pose(i,3)), 0  ;
          sin(pose(i,3)), cos(pose(i,3)) , 0;
            0  ,               0,            1 ];
    t  = [pose(i,4);pose(i,5);pose(i,6)]; 
   
    
    RtLidarAftTrans  = [R2*R1*R3, t;
                        0 ,0 ,0 , 1];

%     RtLidarAftTrans = [pose(i,1:4);pose(i,5:8);pose(i,9:12);0,0,0,1];

    Rtli2li(i,:) = [RtLidarAftTrans(1,:),RtLidarAftTrans(2,:),RtLidarAftTrans(3,:)];
           
    


    %转为变换前的激光雷达旋转平移矩阵
    
    Rlb2la =  [0 1 0 ;
             0 0 1 ;
             1 0 0 ;];
    tlb2la = [0;0;0];
    Rtlb2la=[Rlb2la,tlb2la;
             0 ,0 ,0  1];
    
    
    Rla2lb=Rlb2la';
    tla2lb=-Rla2lb*tlb2la;
     Rtla2lb=[Rla2lb,tla2lb;
             0 ,0 ,0  1];
    
    RtLidarBefTrans = Rtla2lb  * RtLidarAftTrans * Rtlb2la;
                   
                   
    %转为相机坐标下的矩阵

    Rc2l = Rl2c';
    tc2l = -Rc2l*tl2c;
    
    Rtc2l = [ Rc2l ,tc2l;
                0,0,0,1 ];
    %得到相机到初始激光雷达坐标系下的坐标      
    Rtc2initl =  RtLidarBefTrans* Rtc2l ;
    
    %转到相机到初始相机下的矩阵
    Rtl2c = [ Rl2c ,tl2c;
                0,0,0,1 ];
            
    Rtc2initc = Rtl2c*Rtc2initl;
    
    evaluationMatrix(i,:)=[Rtc2initc(1,:),Rtc2initc(2,:),Rtc2initc(3,:)];
    
    
end
    
 %% 旋转矩阵到欧拉角
 tht_groundtruth = zeros (num,3);
 tht_evaluation = zeros (num,3);
 tht_test = zeros (num,3);
 
 
 for i = 1:1:num
    tht_groundtruth(i,:) = rotationMatrixToYPR( [groundtruth(i,1:3);groundtruth(i,5:7);groundtruth(i,9:11)] );
     
    tht_evaluation(i,:) = rotationMatrixToYPR( [evaluationMatrix(i,1:3);evaluationMatrix(i,5:7);evaluationMatrix(i,9:11)] ); 
    
     tht_test(i,:) = rotationMatrixToYPR( [Rtli2li(i,1:3);Rtli2li(i,5:7);Rtli2li(i,9:11)] ); 
 end
 
 
 
 %% range
 range = zeros(1,num);
 for i = 2:1:num
     range(i)=range(i-1)+norm([groundtruth(i,4),groundtruth(i,12),groundtruth(i,8)]-[groundtruth(i-1,4),groundtruth(i-1,12),groundtruth(i-1,8)],2);
 end
 range_test = zeros(1,num);
 for i = 2:1:num
     range_test(i)=range_test(i-1)+norm([evaluationMatrix(i,4),evaluationMatrix(i,12),evaluationMatrix(i,8)]-[evaluationMatrix(i-1,4),evaluationMatrix(i-1,12),evaluationMatrix(i-1,8)],2);
 end
 
%% error
error =  evaluationMatrix-groundtruth;
error_t  =error(:,[4,8,12]);
error_t_evaluate = [];
error_t_evaluate_percent_xyz = [];
error_t_evaluate_percent=[];
evaluate_range = 100;
i=1;
while (evaluate_range<=range(end))
    ind(i) = find(range>evaluate_range, 1 );
    ratio = (evaluate_range-range(ind(i)-1))/(range(ind(i))-range(ind(i)-1));
    
    error_t_evaluate(i,:) = error_t(ind(i)-1,:)+ratio*( error_t(ind(i),:)- error_t(ind(i)-1,:));
    error_t_evaluate_percent(i,:)=error_t_evaluate(i,:)/evaluate_range;
    error_t_evaluate_percent_xyz(i)=norm(error_t_evaluate(i,:),2)/evaluate_range;
    
    i=i+1;
    evaluate_range=evaluate_range+100;
end

num_evaluate_range = length(error_t_evaluate_percent_xyz);
mean_error_t_percent = sum(error_t_evaluate_percent_xyz)/length(error_t_evaluate_percent_xyz);



%%   path
% color =[1,0,0;0,0,1];
% path
color= lines(num_evaluate_range+1);

figure()
plot3(evaluationMatrix(1:ind(1),4),evaluationMatrix(1:ind(1),8),evaluationMatrix(1:ind(1),12),'-','Color',color(1,:));
hold on
plot3(groundtruth(1:ind(1),4),groundtruth(1:ind(1),8),groundtruth(1:ind(1),12),'--','Color',color(1,:));
for  i = 2:1:num_evaluate_range
    plot3(evaluationMatrix(ind(i-1):ind(i),4),evaluationMatrix(ind(i-1):ind(i),8),evaluationMatrix(ind(i-1):ind(i),12),'-','Color',color(i,:));
    
    plot3(groundtruth(ind(i-1):ind(i),4),groundtruth(ind(i-1):ind(i),8),groundtruth(ind(i-1):ind(i),12),'--','Color',color(i,:));
    
end

    plot3(evaluationMatrix(ind(i):end,4),evaluationMatrix(ind(i):end,8),evaluationMatrix(ind(i):end,12),'-','Color',color(end,:));
    
    plot3(groundtruth(ind(i):end,4),groundtruth(ind(i):end,8),groundtruth(ind(i):end,12),'--','Color',color(end,:));
%orientation
lmd = 50;
ind_2 = [1,ind];
ori_gt = [groundtruth(ind_2,4),groundtruth(ind_2,8),groundtruth(ind_2,12)];
ori_test = [evaluationMatrix(ind_2,4),evaluationMatrix(ind_2,8),evaluationMatrix(ind_2,12)];
x_gt =  lmd *[groundtruth(ind_2,1),groundtruth(ind_2,5),groundtruth(ind_2,9)];
y_gt =  lmd *[groundtruth(ind_2,2),groundtruth(ind_2,6),groundtruth(ind_2,10)];
z_gt =  lmd *[groundtruth(ind_2,3),groundtruth(ind_2,7),groundtruth(ind_2,11)];
x_test = lmd * [evaluationMatrix(ind_2,1),evaluationMatrix(ind_2,5),evaluationMatrix(ind_2,9)];
y_test = lmd * [evaluationMatrix(ind_2,2),evaluationMatrix(ind_2,6),evaluationMatrix(ind_2,10)];
z_test = lmd * [evaluationMatrix(ind_2,3),evaluationMatrix(ind_2,7),evaluationMatrix(ind_2,11)];

% 
% for i = 1:1:num_evaluate_range+1
%    %x
%     plot3([ori_gt(i,1),ori_gt(i,1)+x_gt(i,1)],[ori_gt(i,2),ori_gt(i,2)+x_gt(i,2)],[ori_gt(i,3),ori_gt(i,3)+x_gt(i,3)],'r');
%     plot3([ori_test(i,1),ori_test(i,1)+x_test(i,1)],[ori_test(i,2),ori_test(i,2)+x_test(i,2)],[ori_test(i,3),ori_test(i,3)+x_test(i,3)],'r');
%     %y
%     plot3([ori_gt(i,1),ori_gt(i,1)+y_gt(i,1)],[ori_gt(i,2),ori_gt(i,2)+y_gt(i,2)],[ori_gt(i,3),ori_gt(i,3)+y_gt(i,3)],'g');
%     plot3([ori_test(i,1),ori_test(i,1)+y_test(i,1)],[ori_test(i,2),ori_test(i,2)+y_test(i,2)],[ori_test(i,3),ori_test(i,3)+y_test(i,3)],'g');
%     %z
%     plot3([ori_gt(i,1),ori_gt(i,1)+z_gt(i,1)],[ori_gt(i,2),ori_gt(i,2)+z_gt(i,2)],[ori_gt(i,3),ori_gt(i,3)+z_gt(i,3)],'b');
%     plot3([ori_test(i,1),ori_test(i,1)+z_test(i,1)],[ori_test(i,2),ori_test(i,2)+z_test(i,2)],[ori_test(i,3),ori_test(i,3)+z_test(i,3)],'b');
%     
% end
    
axis equal
grid on
xlabel ('x/m');
ylabel ('y/m');
zlabel ('z/m');

% figure()
% plot3(evaluationMatrix(927:987,4),evaluationMatrix(927:987,8),evaluationMatrix(927:987,12),'-r*');
% hold on
% plot3(groundtruth(927:987,4),groundtruth(927:987,8),groundtruth(927:987,12),'-bo');
% axis equal
% grid on
% xlabel ('x/m');
% ylabel ('y/m');
% zlabel ('z/m');

%% error t

figure()
 subplot(1,3,1)
  plot (range(1:ind(1)),error_t(1:ind(1),1),'Color',color(1,:));
  hold on;
 for i = 2:1:num_evaluate_range
    plot (range(ind(i-1):ind(i)),error_t(ind(i-1):ind(i),1),'Color',color(i,:));
 end
 plot (range(ind(i):end),error_t(ind(i):end,1),'Color',color(end,:));
 xlabel ('range/m');
 ylabel ('{error_x}/m');grid on
 
 subplot(1,3,2)
 plot (range(1:ind(1)),error_t(1:ind(1),2),'Color',color(1,:));
  hold on;
 for i = 2:1:num_evaluate_range
    plot (range(ind(i-1):ind(i)),error_t(ind(i-1):ind(i),2),'Color',color(i,:));
 end
  plot (range(ind(i):end),error_t(ind(i):end,2),'Color',color(end,:));
 xlabel ('range/m');
 ylabel ('{error_y}/m');grid on

 
 subplot(1,3,3)
  plot (range(1:ind(1)),error_t(1:ind(1),3),'Color',color(1,:));
  hold on;
 for i = 2:1:num_evaluate_range
    plot (range(ind(i-1):ind(i)),error_t(ind(i-1):ind(i),3),'Color',color(i,:));
 end
  plot (range(ind(i):end),error_t(ind(i):end,3),'Color',color(end,:));
 xlabel ('range/m');
 ylabel ('{error_z}/m');grid on
 
 
 figure()
 subplot(1,3,1)
 plot (100:100:num_evaluate_range*100,error_t_evaluate_percent(:,1)*100);
 xlabel ('range/m');
 ylabel ('{error_x} percentage');grid on
 subplot(1,3,2)
 plot (100:100:num_evaluate_range*100,error_t_evaluate_percent(:,2)*100);
 xlabel ('range/m');
 ylabel ('{error_y} percentage');grid on
 subplot(1,3,3)
 plot (100:100:num_evaluate_range*100,error_t_evaluate_percent(:,3)*100);
 xlabel ('range/m');
 ylabel ('{error_z} percentage');grid on
 
figure()
 plot (100:100:num_evaluate_range*100,error_t_evaluate_percent_xyz*100);
  xlabel ('range/m');
 ylabel ('{error} percentage');
 grid on
 
 %% related error between consecutive frames
 RtRelet_groundtruth=zeros(num-1,12);RtRelet_evaluate=zeros(num-1,12);
 thtRelet_groundtruth = zeros(num,3); thtRelet_evaluate = zeros(num,3);
 
 for i = 1:1:num-1
     RtRelet_groundtruth(i,:)=kittiPoseRelet(groundtruth(i:i+1,:));
    RtRelet_evaluate(i,:)=kittiPoseRelet(evaluationMatrix(i:i+1,:));
 end
 RtRelet_groundtruth = [zeros(1,12);RtRelet_groundtruth];
 RtRelet_evaluate = [zeros(1,12);RtRelet_evaluate];
 
 
 for i = 1:1:num
    thtRelet_groundtruth(i,:) = rotationMatrixToYPR( [RtRelet_groundtruth(i,1:3);RtRelet_groundtruth(i,5:7);RtRelet_groundtruth(i,9:11)] );
     
    thtRelet_evaluate(i,:) = rotationMatrixToYPR( [RtRelet_evaluate(i,1:3);RtRelet_evaluate(i,5:7);RtRelet_evaluate(i,9:11)] ); 
   
 end
 
 
 
 error_releted = RtRelet_evaluate-RtRelet_groundtruth;
 error_releted_t = error_releted(:,[4,8,12]);
 error_releted_tht = thtRelet_evaluate-thtRelet_groundtruth;

 %t
 figure()
 subplot(1,3,1)
  plot (range(1:ind(1)),error_releted_t(1:ind(1),1),'Color',color(1,:));
  hold on;
 for i = 2:1:num_evaluate_range
    plot (range(ind(i-1):ind(i)),error_releted_t(ind(i-1):ind(i),1),'Color',color(i,:));
 end
 plot (range(ind(i):end),error_releted_t(ind(i):end,1),'Color',color(end,:));
 xlabel ('range/m');
 ylabel ('{error_{xReleted}}/m');grid on
 
 subplot(1,3,2)
 plot (range(1:ind(1)),error_releted_t(1:ind(1),2),'Color',color(1,:));
  hold on;
 for i = 2:1:num_evaluate_range
    plot (range(ind(i-1):ind(i)),error_releted_t(ind(i-1):ind(i),2),'Color',color(i,:));
 end
  plot (range(ind(i):end),error_releted_t(ind(i):end,2),'Color',color(end,:));
 xlabel ('range/m');
 ylabel ('{error_{yReleted}}/m');grid on

 
 subplot(1,3,3)
  plot (range(1:ind(1)),error_releted_t(1:ind(1),3),'Color',color(1,:));
  hold on;
 for i = 2:1:num_evaluate_range
    plot (range(ind(i-1):ind(i)),error_releted_t(ind(i-1):ind(i),3),'Color',color(i,:));
 end
  plot (range(ind(i):end),error_releted_t(ind(i):end,3),'Color',color(end,:));
 xlabel ('range/m');
 ylabel ('{error_{zReleted}}/m');grid on

 
 
 %tht
 figure()
 subplot(1,3,1)
  plot (range(1:ind(1)),error_releted_tht(1:ind(1),1),'Color',color(1,:));
  hold on;
 for i = 2:1:num_evaluate_range
    plot (range(ind(i-1):ind(i)),error_releted_tht(ind(i-1):ind(i),1),'Color',color(i,:));
 end
 plot (range(ind(i):end),error_releted_tht(ind(i):end,1),'Color',color(end,:));
 xlabel ('range/m');
 ylabel ('{error_{\ThetaxReleted}}/m');grid on
 
 subplot(1,3,2)
 plot (range(1:ind(1)),error_releted_tht(1:ind(1),2),'Color',color(1,:));
  hold on;
 for i = 2:1:num_evaluate_range
    plot (range(ind(i-1):ind(i)),error_releted_tht(ind(i-1):ind(i),2),'Color',color(i,:));
 end
  plot (range(ind(i):end),error_releted_tht(ind(i):end,2),'Color',color(end,:));
 xlabel ('range/m');
 ylabel ('{error_{\ThetayReleted}}/m');grid on

 
 subplot(1,3,3)
  plot (range(1:ind(1)),error_releted_tht(1:ind(1),3),'Color',color(1,:));
  hold on;
 for i = 2:1:num_evaluate_range
    plot (range(ind(i-1):ind(i)),error_releted_tht(ind(i-1):ind(i),3),'Color',color(i,:));
 end
  plot (range(ind(i):end),error_releted_tht(ind(i):end,3),'Color',color(end,:));
 xlabel ('range/m');
 ylabel ('{error_{\ThetazReleted}}/m');grid on

 
 %% diff error
range_ = range(2:end);
diff_error_t = zeros(num-1,3);
diff_error_t_abs = zeros(num-1,3);
diff_error_t_abs_range = zeros(num-1,1);
for i = 1:1:num-1
    diff_error_t(i,:) = abs(error_t(i+1,:))-abs(error_t(i,:));
    diff_error_t_abs(i,:) = abs(diff_error_t(i,:));
    diff_error_t_abs_range(i) = norm(diff_error_t_abs(i,:),2);
end
figure()
subplot(2,3,1)
plot(range_,diff_error_t(:,1))
xlabel ('range/m');
ylabel ('\Delta{error_x}/m');
subplot(2,3,2)
plot(range_,diff_error_t(:,2))
xlabel ('range/m');
ylabel ('\Delta{error_y}/m');
subplot(2,3,3)
plot(range_,diff_error_t(:,3))
xlabel ('range/m');
ylabel ('\Delta{error_z}/m');
subplot(2,3,4)
plot(range_,diff_error_t_abs(:,1))
xlabel ('range/m');
ylabel ('|\Delta{error_x}|/m');
subplot(2,3,5)
plot(range_,diff_error_t_abs(:,2))
xlabel ('range/m');
ylabel ('|\Delta{error_y}|/m');
subplot(2,3,6)
plot(range_,diff_error_t_abs(:,3))
xlabel ('range/m');
ylabel ('|\Delta{error_z}|/m');

figure()
plot(range_,diff_error_t_abs_range)
xlabel ('range/m');
ylabel ('|\Delta{error}|/m');
 
 
 