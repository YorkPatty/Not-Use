% Note that the SIDynamometer project is already modified to 

% Using To Workspace Block (will convert to IDDATA later)

% simulink model needs a mux block collecting speed, throttle, and Map
% the mux sends to workspace called SpdThrMap
function [iddataCollection, vddataCollection] = GetIDData(RPM,ThrB, ThrW, runTime)
numIter = length(ThrB);
%runTime = 200;

    
    sim('SiDynoReferenceApplication',runTime)
    iddataCollection = SpdThrMap;
    sim('SiDynoReferenceApplication',runTime);
    vddataCollection = SpdThrMap;
    close all;
