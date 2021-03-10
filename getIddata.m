% simulink model needs a mux block collecting speed, throttle, and Map
% the mux sends to workspace called SpdThrMap

numIter = 9;
rpmDiff = 1000;
runTime = 20;
proj = openProject('C:\Users\austi\MATLAB\Projects\examples\DataDyno\main\SIdynamometer.prj');

iddataCollection = cell(1,numIter);
vddataCollection = cell(1,numIter);
RPM = [0 1000];
for i =  1:numIter
    RPM = [0 i*rpmDiff];
    sim('SiDynoReferenceApplication',runTime)
    iddataCollection(i) = {SpdThrMap};
    sim('SiDynoReferenceApplication',runTime);
    vddataCollection(i) = {SpdThrMap};
    close all;
end