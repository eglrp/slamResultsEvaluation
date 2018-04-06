function [Rinit, tinit] = kittiPoseToRT(Rt)
    Rinit = [Rt(1:3);Rt(5:7);Rt(9:11)];
    tinit = [Rt(4);Rt(8);Rt(12)];

end