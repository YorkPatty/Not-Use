% simulink model needs a mux block collecting speed, throttle, and Map
% the mux sends to workspace called SpdThrMap

numIter = 2;
rpmDiff = 1000;
runTime = 1000;
proj = openProject('C:\Users\patmike\MATLAB\Projects\examples\SIDynamometer\main\SIdynamometer.prj');

iddataCollection = cell(1,numIter);
vddataCollection = cell(1,numIter);

for i =  1:numIter
    RPM = [0 i*rpmDiff];
    sim('SiDynoReferenceApplication',runTime)
    iddataCollection(i) = {ThrSpdMAP};
    sim('SiDynoReferenceApplication',runTime);
    vddataCollection(i) = {ThrSpdMAP};
    close all;
end