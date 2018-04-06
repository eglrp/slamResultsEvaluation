function RtRelet=kittiPoseRelet(Rt)

    [R1,t1 ]= kittiPoseToRT(Rt(1,:));
    [R2,t2 ]= kittiPoseToRT(Rt(2,:));
    
    Rinit2R1 = R1';
    tinit2t1 = - Rinit2R1*t1;
    Rtinit2R1 = [Rinit2R1,tinit2t1;
                 0 , 0, 0, 1];
    
    RtRelet_ = Rtinit2R1*[R2,t2;0,0,0,1];
    RtRelet = [RtRelet_(1,:),RtRelet_(2,:),RtRelet_(3,:)];
%      RtRelet = RtRelet_;
    

end