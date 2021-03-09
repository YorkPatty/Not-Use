numIter = 9;
rpmDiff = 1000;
runTime = 20;
proj = openProject('C:\Users\austi\MATLAB\Projects\examples\DataDyno\main\SIdynamometer.prj');

RPM = [0 1000];
sim('SiDynoReferenceApplication',runTime);
iddataCollection = {ThrToMAP};
sim('SiDynoReferenceApplication',runTime);
vddataCollection = {ThrToMAP};
for i =  2:numIter
    RPM = [0 i*rpmDiff];
    sim('SiDynoReferenceApplication',runTime)
    iddataCollection(i) = {ThrToMAP};
    sim('SiDynoReferenceApplication',runTime);
    vddataCollection(i) = {ThrToMAP};
    close all;
end