%function [MPCobjs, Fits, Validations, BestFit]=GenerateMPCDesignsTry1(iddataCollection,vddataCollection)
function [Fits, Validations, BestFit]=GenerateMPCDesignsTry1(iddataCollection,vddataCollection)
% This function takes simulated input/output data and creates state space
% models. 

% Inputs: The i/o data will be throttle and wastegate position and engine
%         speed as the inputs. MAP (intake manifold pressure) will be the 
%         output. 
    % IDData: i/o data used for system identification (model creation)
    % VDData: i/o data that will be used for model validation

% Outputs: 
    % MPCobjs: MPC objects created from the identified model
    % Fits: time, measured outputs, simulated outputs (w/ ID'd model)
    % Validations: same as above but with validation data
         
% Output Data: 
    % MAP (intake manifold pressure)

    for i=1:length(iddataCollection(1,:)),
       
      %Plant fit    
      % unpackage identification data (take out Thr and Wg and assign to
      % new variable)
      u = [iddataCollection{i}.signals.values(:,1)]; % add Wg (:,1:2)
      
     % unpackaging output (MAP)
      y=[iddataCollection{i}.signals.values(:,3)]; 
      
      % clean up data and create iddata object
      % set value of  chosen based on examination of data inspector
      ind=find(iddataCollection{i}.time(:,1)>10);  %Find initial steady-state start point to remove initial engine startup transient
      z=iddata(y(ind,:),u(ind,:)); % create iddata object with i/o data
      z.Ts=0.01; % same sampling time as SI Engine Dyno system
      
      % create specified order state space models
        % order of 5 works well for throttle to MAP
      IDmodel=n4sid(z,2,'Ts',0.01,'DisturbanceModel','none');
      
      % simulate the response (yout) of the identified model(state space); 
      yout=sim(IDmodel,z.InputData); % yout is the outputs from the model
      time=(0:size(yout,1)-1)'*z.Ts;
      
      % Saving the time, measured and simulated outputs for later analysis
      % FUNCTION OUTPUT
      Fits{i}=[time z.OutputData(:,1) yout(:,1)]; % add more depending on number of outputs
      [y fits ic] = compare(z, IDmodel);
          GoodFits(i) = fits; % store to see fit of each MPC
      %Design MPC Control
%       MPCobjs{i} = DesignMPC(IDmodel,z); % FUNCTION OUTPUT
      
      %Plant validation   
      % unpackage data
      u=[vddataCollection{i}.signals.values(:,1)];% add 2 for (:,1:2)
      y=[vddataCollection{i}.signals.values(:,3)]; %Set up system identifaction output data for MAP, EGR
      ind=find(vddataCollection{i}.time>10);  %Find initial steady-state start point to remove initial engine startup transient
      z=iddata(y(ind,:),u(ind,:)); % create iddata object
      z.Ts=0.01; % set sample time
      yout=sim(IDmodel,z.InputData); % simulate output response of model
      time=(0:size(yout,1)-1)'*z.Ts;
      % save output data for later analysis (output of function)
      Validations{i}=[time z.OutputData(:,1) yout(:,1)]; % FUNCTION OUTPUT
      
      % validate with 
          % y is the simulated output response
          % NRMSE best fit value (same best fit as SID Toolbox 'model output')
          % ic: initial conditions
          [y fits ic] = compare(z, IDmodel);
          BestFit(i) = fits; % store to see fit of each MPC
   end
 
end


function MPCobj=DesignMPC(IDmodel,IDdata)
% This function takes the state spaces models and creates the
% corresponding number of controllers. Each MPC is reviewed and then 
% open for further tuning in the MPCDesigner app. 

% Inputs: 
    % IDmodel: created state space model
    % IDdata: workspace data from GetIDdata for system ID
    
% Outputs:
    % MPCobj: mpc object ready for design
    
   % create the plant
   Plant = ss(IDmodel);
   Plant.InputName='';
   Plant.InputUnit='';
   
   % Augment plant with unmeasured disturbance channel.
   % set(Plant, 'B', [Plant.B Plant.B(:,1)], 'D', [Plant.D Plant.D(:,1)]);
   
   % Assign names to I/O variables
   set(Plant, 'InputName', {'THRPOS'},... % {'THRPOS'; 'WGPOS'; 'ENGSPD'; 'UD1'; 'UD2'}
    'OutputName', {'MAP'},...
    'InputUnit', {'%'},... % {'%', '%', 'RPM', '', ''}
    'OutputUnit', {'KPa'});
   
                               
    % Assign MPC channel type
    % Plant = setmpcsignals(Plant,'mv',[1 2],'md',[3],'ud',[4 5],'mo',[1]);
    setmpcsignals(Plant, 'mv', [1], 'mo', [1]);
    Model.Plant = Plant;
    
    %Set nomimal inputs and outputs obtained at the equilibrium operating point
    Model.Nominal.U=[mean(IDdata.u)]; % [mean(IDdata.u 0 0]
    Model.Nominal.Y=mean(IDdata.y);
    Model.Nominal.X=[0 0 0 0];
    Model.Nominal.DX=[0 0 0 0];

    %% Define sample time and horizons
    Ts=0.01; % sample time
    p=20;   % prediction time is about 4 seconds --> 
    m=2;    % 10% of prediction horizon

    %% Define hard and soft constraints
    % EGRPOS, VGTPOS move full-range approx 1 second, FUELMASS can move
    % full-range approx 100 ms, SPEED is the slowest input, moving on order of
    % several seconds.
    InputSpecs(1)=struct('Min',0,'Max',100,'RateMin',-0.5,'RateMax',0.5,'MinECR',1,'MaxECR',1,'RateMinECR',0,'RateMaxECR',0);
    %InputSpecs(2)=struct('Min',0.01,'Max',0.9,'RateMin',-0.05,'RateMax',0.05,'MinECR',1,'MaxECR',1,'RateMinECR',0,'RateMaxECR',0);
    OutputSpecs(1)=struct('Min',0.8*min(IDdata.y(:,1)),'Max',1.2*max(IDdata.y(:,1)),'MinECR',1,'MaxECR',1);
    %OutputSpecs(2)=struct('Min',0.8*min(IDdata.y(:,2)),'Max',1.2*max(IDdata.y(:,2)),'MinECR',1,'MaxECR',1);

    %% Define weights on manipulated and controlled variables.
    Weights=struct('ManipulatedVariables',[0],... % [0 0]
       'ManipulatedVariablesRate',[1],...         % [1 1]
       'OutputVariables',[0.1]);                  

    %% Create MPC controller in MATLAB
    MPCobj=mpc(Model,Ts,p,m,Weights,InputSpecs,OutputSpecs);

    %% Review MPC design
    review(MPCobj);

    %% Validate MPC design in the GUI environment
    mpcDesigner(MPCobj);

end
