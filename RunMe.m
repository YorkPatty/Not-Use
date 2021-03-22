% enter the RPM that you would like to test and the run time
% make sure that the three variables below are all the same length 
ThrB = [5 8 10 15];
ThrW = [3 3 4  5 ];
rpm = linspace(3000,3000,length(ThrB));
numIter = length(ThrB);

runTime = 1000;
BBT = 1.8;
PofZ = 0.5;

iddataCollection = cell(1,numIter);
vddataCollection = cell(1,numIter);

proj = openProject('/Users/jonathanwozny/MATLAB/projects/examples/SIDynamometer/main/SIDynamometer.prj');

for i = 1:length(ThrB)
RPM = [0 rpm(i)];
ThrBase = ThrB(i);
ThrWiggle = ThrW(i);

[iddataCollection{i} vddataCollection{i}] = GetIDData(RPM, ThrB, ThrW, runTime);
fprintf('Run: %i/%i\n',i,length(ThrB(i)));
end
%[MPCobjs,Fits,Validations,BestFit] = GenerateMPCDesignsTry1(iddataCollection,vddataCollection);
[Fits,Validations,BestFit] = GenerateMPCDesignsTry1(iddataCollection,vddataCollection);

for i = 1:length(ThrB)

    figure;
    plot(Fits{i}(:,1), Fits{i}(:,2),'k-',Fits{i}(:,1), Fits{i}(:,3),'b-');
    title("Model Identification: Throttle = " + ThrB(i) + " Wiggle = " + ThrW(i) +" ")
    xlabel('Time (s)')
    ylabel('Boost (Pa)')

    figure;
    plot(Validations{i}(:,1), Validations{i}(:,2),'r-',Validations{i}(:,1), Validations{i}(:,3),'b-');
    title("Model Validation: Throttle = " + ThrB(i) + " Wiggle = " + ThrW(i) +" ")
    xlabel('Time (s)')
    ylabel('Boost (Pa)')
    annotation('textbox', [0.72, 0.2, 0.17, 0.045], 'String', "Best Fit: " +BestFit(i)+"%")


    disp(' ');
    fprintf('Best Fit for EngSpd = %i (Throttle = %i, with Wiggle = %i) is %.2f\n', rpm(i), ThrB(i),ThrW(i), BestFit(i));
    disp(' ');
end
