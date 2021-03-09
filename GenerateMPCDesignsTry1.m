% note that EGR and VGT are the throttle and wastegate in our case (input)s
% MAP is our output values from the data

% EGR (in) --> IDdata{i}.simin.signals.values(:,1)
% VGT --> IDdata{i}.simin.signals.values(:,2)
% Speed --> IDdata{i}.simin.signals.values(:,4)
% MAP --> IDdata{i}.simout.signals.values(:,7)
% EGR (out) --> IDdata{i}.simout.signals.values(:,14)

%Create system identification models
function [MPCobjs,Fits,Validations]=GenerateMPCDesignsTry1(iddataCollection,vddataCollection)

   for i=1:length(iddataCollection(:,1)),
       
      %Plant fit 
    %  FuelMass=1000.*IDdata{i}.simout.signals.values(:,11)./(IDdata{i}.simin.signals.values(:,3).*(2.*IDdata{i}.simin.signals.values(:,4)/60.));  %Calculate fuel mass from AFR and airflow data
     
      %Set up system identification input data for ThrottlePos and Speed
     % JUST UNPACKAGING IDDATA (inputs)
      u=[iddataCollection{i}.InputData(:,1)]; 
         
        %  FuelMass 
        %  IDdata{i}.simin.signals.values(:,4)];  
      
     % Set up system identification output data for MAP
     % JUST UNPACKAGING IDDATA (outputs)
      y=iddataCollection{i}.OutputData(:,1); 
      
      % set to >2 based on just looking at DataAnalyzer and seeing where it
      % stops growing
      % JUST CLEANING UP THE DATA TO GET PURE STEADY STATE RESPONSE
      ind=find(iddataCollection{i}.sa(:,1)>3.);  %Find initial steady-state start point to remove initial engine startup transient
      % z=iddata(y(ind,:),u(ind,:));
      z = iddataCollection{i};
      %z.Ts=0.01; % same sampling time as current system
      
      % creating fourth order state space models using system identifaction
      % toolbox
      IDmodel=n4sid(z,4,'Ts',0.01,'DisturbanceModel','none');
      
      % simulate the response (yout) of the identified model(state space); returns output
      % response of the state space using input values 
      yout=sim(IDmodel,z.InputData); % yout is the outputs from the model
      time=(0:size(yout,1)-1)'*z.Ts;
      
      % just saving the fits for later comparison (outputs of the function)
      Fits{i}=[time z.OutputData(:,1) yout(:,1)]; %z.OutputData(:,2) yout(:,2)];
      
      %Design MPC Control
      MPCobjs{i}=DesignMPC(IDmodel,z);     
      %MPCobjs{i}=mpc(IDmodel,z);  
      
      %Plant validation
   %   FuelMass=1000.*VData{i}.simout.signals.values(:,11)./(VData{i}.simin.signals.values(:,3).*(2.*VData{i}.simin.signals.values(:,4)/60.));  %Calculate fuel mass from AFR and airflow data
     
   u=[VData{i}.InputData(:,1)];
        % FuelMass 
        % VData{i}.simin.signals.values(:,4)];  %Set up system identification input data for EGR, VGT, FUEL, SPEED
      y=VData{i}.OutputData(:,1); %Set up system identifaction output data for MAP, EGR
      ind=find(VData{i}.sa>3.);  %Find initial steady-state start point to remove initial engine startup transient
      z=iddata(y(ind,:),u(ind,:));
      z.Ts=vddataCollection.Ts;
      yout=sim(IDmodel,z.InputData);
      time=(0:size(yout,1)-1)'*z.Ts;
      Validations{i}=[time z.OutputData(:,1) yout(:,1) z.OutputData(:,2) yout(:,2)];
      
   end

end


function MPCobj=DesignMPC(IDmodel,IDdata)

   Plant = ss(IDmodel);
   Plant.InputName='';
   Plant.InputUnit='';
   
   % Augment plant with unmeasured disturbance channel.
   %set(Plant, 'B', [Plant.B Plant.B(:,1)], 'D', [Plant.D Plant.D(:,1)]);
   
   % Assign names to I/O variables
   % INPUTS: 
        % EGRPOS AND VGTPOS  --> replace with THRPOS
        % FUELMASS and SPEED --> measured disturbances (DELETE FUEL MASS)
        % UD1 and UD2        --> unmeasured disturbances (KEEP)
        
   % OUTPUTS:
        % BOOST and EGRMASSFLOW --> keep BOOST (MAP) and delete EGRMASSFLOW
   set(Plant, 'InputName', {'THRPOS'},...
    'OutputName', {'MAP'},...
    'InputUnit', {'mm'},...
    'OutputUnit', {'KPa'});
    
    % MAKE INTO setmpcsignals(Plant, 'mv', [1], 'md', [2], 'ud', [3,4],   'mo', [1]);
    %                                 ThrPos    EngSpd      UD1 and UD2    MAP
    %                              
    % Assign MPC channel type
    % Plant = setmpcsignals(Plant,'mv',[1 2],'md',[3 4],'ud',[5 6],'mo',[1 2]);
    setmpcsignals(Plant, 'mv', [1], 'mo', [1]);
    Model.Plant = Plant;
    
    %Set nomimal inputs and outputs obtained at the equilibrium operating point
    Model.Nominal.U=[mean(IDdata.u)];
    Model.Nominal.Y=mean(IDdata.y);
    Model.Nominal.X=[0 0 0 0];
    Model.Nominal.DX=[0 0 0 0];

    %% Define sample time and horizons
    Ts=0.2; % sample time
    p=20;   % prediction time is about 4 seconds
    m=2;    % 10% of prediction horizon

    %% Define hard and soft constraints
    % EGRPOS, VGTPOS move full-range approx 1 second, FUELMASS can move
    % full-range approx 100 ms, SPEED is the slowest input, moving on order of
    % several seconds.
    InputSpecs(1)=struct('Min',0.1,'Max',5,'RateMin',-0.5,'RateMax',0.5,'MinECR',1,'MaxECR',1,'RateMinECR',0,'RateMaxECR',0);
    %InputSpecs(2)=struct('Min',0.1,'Max',0.9,'RateMin',-0.05,'RateMax',0.05,'MinECR',1,'MaxECR',1,'RateMinECR',0,'RateMaxECR',0);
    OutputSpecs(1)=struct('Min',0.8*min(IDdata.y(:,1)),'Max',1.2*max(IDdata.y(:,1)),'MinECR',1,'MaxECR',1);
    %OutputSpecs(2)=struct('Min',0.8*min(IDdata.y(:,2)),'Max',1.2*max(IDdata.y(:,2)),'MinECR',1,'MaxECR',1);

    %% Define weights on manipulated and controlled variables.
    Weights=struct('ManipulatedVariables',[0],...
       'ManipulatedVariablesRate',[1],...
       'OutputVariables',[0.1]);

    %% Create MPC controller in MATLAB
    MPCobj=mpc(Model,Ts,p,m,Weights,InputSpecs,OutputSpecs);

    %% Review MPC design
    review(MPCobj);

    %% Validate MPC design in the GUI environment
    mpcDesigner(MPCobj);

end