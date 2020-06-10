%% Clean up
clearvars;  % Clear workspace
close all % Close all open figures
clc  % Clear command window
Simulink.sdi.clear;  % Clear the simulation data inspector

%% Load bus definitions
load busDefs  % load the buses definitions

%% Open model
open_system('project4Model');  % load the simulink model

%% Solver Parameters
simPrm.h      = 1E-3; % Integration time step (s)
simPrm.solTyp = 'Fixed-step';  % Solver type
simPrm.sol    = 'ode4';  % Integration method
simPrm.tEnd   = 50; % Simulation end time (s)

%% Set Simulink Configuration
set_param('project4Model','SolverType',simPrm.solTyp);  % set this solver type in simulink parameters
set_param('project4Model','Solver'    ,simPrm.sol);     % set this integration method in simulink solver

%% Physical Constants
planet.g = 10; % grav accel (m/s^2)

%% Jib Parameters
jib.l   = 3.0;  % length of jib (m)
jib.w   = 0.1;  % width of jib  (m)
jib.rho = 2700; % density of the jib material (kg/m^3)

%% Trolley Parameters
trol.l   = 0.2;     % length of trolley (m)
trol.w   = 0.2;     % width of trolley (m)
trol.h   = 0.2;     % height of trolley (m)
trol.ics = [2.5 0]; % Initial position and speed of trolley (m, m/s)
trol.rho = 2700;    % density of trolley material (kg/m^3)
trol.clr = [147 174 200]/255; % light blue

%% Load Parameters
lod.l = 0.3;    % length of load block (m)
lod.w = 0.1;    % width of load block (m)
lod.h = 0.2;    % height of load block (m)
lod.rho = 8000; % density load block material (kg/m3);
lod.ic = -3.0;  % Initial position of the load block (m)
lod.v = lod.l*lod.w*lod.h; % volume of the load block
lod.m = lod.rho*lod.v;  % mass of the load

lod.Ixx = (1/12)*lod.m*((lod.h^2)+(lod.w^2));  % Mass moment of Inertia of load block along X-X
lod.Iyy = (1/12)*lod.m*((lod.l^2)+(lod.w^2));  % Mass moment of Inertia of load block along Y-Y
lod.Izz = (1/12)*lod.m*((lod.l^2)+(lod.h^2));  % Mass moment of Inertia of load block along Z-Z

%% Base Parameters
bas.h = 3.0;  % height (m)
bas.w = 0.25; % width (m)

%% Motor Parmaters
motor.radius = 0.2;  % slew motor disk radius (m)
motor.height = 0.1;  % slew motor disk height (m)
motor.rho    = 2700; % density of disk material (kg/m^3)
motor.m      = pi*motor.radius^2 * motor.height * motor.rho; % mass of the motor disk(kg)

% mass moments of inertia (kg-m^2)
motor.Ixx    = motor.m * ( (motor.radius^2)/4 + (motor.height^2)/12 );  % Mass moment of Inertia of motor block along X-X
motor.Iyy    = motor.Ixx;  % Mass moment of Inertia of motor block along Y-Y
motor.Izz    = motor.m * motor.radius^2 / 2;  % Mass moment of Inertia of motor block along Z-Z
motor.Iarm   = 1E-1*100;  % armature inertia in spin axis (kg-m^2)

motor.L      = 0.1;       % armature inductance (H)
motor.R      = 1.0;       % armature resistance (ohm)
motor.b      = 1.0;       % damping (N/m/(rad/s))
motor.kt     = 7E-1;      % torque constant (N-m/A)
motor.kb     = 7E-1;      % back emf constant (V/(rad/s))

motor.V = 50;

%% Variant Subsystem Setup
% SCENARIO = 1, Prescribed motion of jib assembly
% SCENARIO = 2, Jib assembly actuated by motor
Prescribed_results = Simulink.Variant('SCENARIO==1'); % Define the first scenario
Motor_results = Simulink.Variant('SCENARIO==2'); % Define the second scenario

%% Simulate Both Scenarios

SCENARIO = 1; % Prescribed motion for jib rotation
Prescribed_results = sim('project4Model','SignalLoggingName','sdat'); % Simulate first scenario and save results in prescribed_results

SCENARIO = 2; % Jib rotation by motor
Motor_results = sim('project4Model','SignalLoggingName','sdat'); % Simulate second scenario and save results in Motor_results

%% Extract Data for plotting
Component = 1;  % Variable for extracting the results from component based model
Force = 2;  % Variable for extracting the force results
Torque = 3; % Variable for extracting the Torque results
Jib_motion_1 = 4; % Variable for extracting the ang displacement of jib in scenario 1
Signal = 5;  % Variable for extracting the results from signal based model
Errors = 6;  % Variable for extracting the error between results from signal based and component based models
Angles = 1; % Variable for extracting euler angles in scenario 2
Jib_motion_2 = 2; % Variable for extracting the ang displacement of jib in scenario 2

% Scenario 1 Results (Prescribed Motion)
s1.t = Prescribed_results.tout;  % Save simulation time values in s1.t array
s1.Phi_S = Prescribed_results.sdat{Signal}.Values.Phi_S.Data(:,1); % Save phi values from signal based model in s1.Phi_S
s1.Tht_S = Prescribed_results.sdat{Signal}.Values.Tht_S.Data(:,1); % Save theta values from signal based model in s1.Tht_S
s1.Psi_S = Prescribed_results.sdat{Signal}.Values.Psi_S.Data(:,1); % Save psi values from signal based model in s1.Psi_S
s1.Phi_C = Prescribed_results.sdat{Component}.Values.Phi_C.Data(:,1); % Save phi values from component based model in s1.Phi_C
s1.Tht_C = Prescribed_results.sdat{Component}.Values.Tht_C.Data(:,1); % Save theta values from component based model in s1.Tht_C
s1.Psi_C = Prescribed_results.sdat{Component}.Values.Psi_C.Data(:,1);  % Save psi values from component based model in s1.Psi_C
s1.Phi_E = Prescribed_results.sdat{Errors}.Values.Data(:,1); % Save error in Phi in s1.Phi_E
s1.Tht_E = Prescribed_results.sdat{Errors}.Values.Data(:,2); % Save error in Phi in s1.Tht_E
s1.Psi_E = Prescribed_results.sdat{Errors}.Values.Data(:,3); % Save error in Phi in s1.Psi_E
s1.Force = Prescribed_results.sdat{Force}.Values.Data(:,1); % Save trolley force in s1.Force
s1.Torque = Prescribed_results.sdat{Torque}.Values.Data(:,1); % Save torque on jib in s1.Torque
s1.motion = Prescribed_results.sdat{Jib_motion_1}.Values.Data(:,1); % Save ang displacement of jib of scneario 1 in s1.motion

% % Scenario 2 Results (Motor)
s2.t = Motor_results.tout;  % Save simulation time values in s2.t array
s2.Phi = Motor_results.sdat{Angles}.Values.Data(:,1); % Save phi values in s2.Phi
s2.Tht = Motor_results.sdat{Angles}.Values.Data(:,2); % Save theta values in s2.Tht
s2.Psi = Motor_results.sdat{Angles}.Values.Data(:,3); % Save psi values in s2.Psi
s2.motion = Motor_results.sdat{Jib_motion_2}.Values.Data(:,1); % Save ang displacement of jib of scneario 2 in s2.motion
 
% %% Plot the results

% % Plot the all parameters vs time for first scenarios
figure(1);

subplot(3,1,1);  % Plot first figure in a subplot of 3 rows and 1 column
plot(s1.t,s1.Phi_S,'k'); % Plot time vs Phi-signal for first scenario
hold on
plot(s1.t,s1.Phi_C,'r'); % Plot time vs Phi-component for first scenario
axis([0 50 -60 60]);  % Give the limits of both axis
legend('\phi signal based','\phi Component based'); % Give the legends
title("Angle \phi vs Time"); % Give the title
xlabel('Time (sec)');  % Give X label as Time in seconds
ylabel('Angle \phi (degrees)');  % Give Y label as angle in degrees

subplot(3,1,2); % Plot second figure in a subplot of 3 rows and 1 column
plot(s1.t,s1.Tht_S,'k'); % Plot time vs Theta-signal for first scenario 
hold on
plot(s1.t,s1.Tht_C,'r');  % Plot time vs Theta-component for first scenario 
axis([0 50 -80 80]);  % Give the limits of both axis
legend('\theta signal based','\theta Component based'); % Give the legends
title("Angle \theta vs Time"); % Give the title
xlabel('Time (sec)');  % Give X label as Time in seconds
ylabel('Angle \theta (degrees)');  % Give Y label as angle in degrees

subplot(3,1,3); % Plot third figure in a subplot of 3 rows and 1 column
plot(s1.t,s1.Psi_S,'k'); % Plot time vs Psi-signal for first scenario
hold on
plot(s1.t,s1.Psi_C,'r'); % Plot time vs Psi-component for first scenario
axis([0 50 -60 360]);  % Give the limits of both axis
legend('\psi signal based','\psi Component based'); % Give the legends
title("Angle \psi vs Time"); % Give the title
xlabel('Time (sec)');  % Give X label as Time in seconds
ylabel('Angle \psi (degrees)');  % Give Y label as angle in degrees

sgtitle('Euler angles vs Time for prescribed motion scneario from singal and component models'); % Create the main title
 
% Plot the errrors in all parameters vs time for first scenario
figure(2);

subplot(3,1,1); % Plot first figure in a subplot of 3 rows and 1 column
plot(s1.t,s1.Phi_E,'b');  % Plot time vs Error in Phi
title("Error in Angle \phi vs Time"); % Give the title
xlabel('Time (sec)');  % Give X label as Time in seconds
ylabel('Error Angle \phi (degrees)');  % Give Y label as error in degrees

subplot(3,1,2); % Plot second figure in a subplot of 3 rows and 1 column
plot(s1.t,s1.Tht_E,'b'); % Plot time vs Error in Theta
title("Error in Angle \theta vs Time"); % Give the title
xlabel('Time (sec)');  % Give X label as Time in seconds
ylabel('Error Angle \theta (degrees)');  % Give Y label as error in degrees

subplot(3,1,3);  % Plot Third figure in a subplot of 3 rows and 1 column
plot(s1.t,s1.Psi_E,'b');   % Plot time vs Error in Psi
title("Error in Angle \psi vs Time"); % Give the title
xlabel('Time (sec)');  % Give X label as Time in seconds
ylabel('Error Angle \psi (degrees)');  % Give Y label as error in degrees

sgtitle('Verification - Error in Euler angles computed from signal and component models'); % Create the main title

% % Plot the computed actuations for first scenario
figure(3);

subplot(2,1,1);  % Plot first figure in a subplot of 2 rows and 1 column
plot(s1.t,s1.Force,'k');  % Plot time vs Force on Trolley
title("Force on Trolley vs Time"); % Give the title
xlabel('Time (sec)');  % Give X label as Time in seconds
ylabel('Force (N)');  % Give Y label as Force in N

subplot(2,1,2);  % Plot second figure in a subplot of 2 rows and 1 column
plot(s1.t,s1.Torque,'k');  % Plot time vs torque on jib
title("Torque on Jib vs Time"); % Give the title
xlabel('Time (sec)');  % Give X label as Time in seconds
ylabel('Torque (N-m)');  % Give Y label as Torque in N-m

sgtitle('Trolley force and Jib torque computed for prescribed motion scenario'); % Create the main title 

% % Plot the euler angles vs time for second scenario
figure(4);

subplot(3,1,1); % Plot first figure in a subplot of 3 rows and 1 column
plot(s2.t,s2.Phi,'k');  % Plot time vs Phi for second scenario
title("Angle Phi vs Time"); % Give the title
xlabel('Time (sec)');  % Give X label as Time in seconds
ylabel('Angle Phi (degrees)');  % Give Y label as angles in degrees

subplot(3,1,2);  % Plot second figure in a subplot of 3 rows and 1 column
plot(s2.t,s2.Tht,'k');  % Plot time vs Theta for second scenario
axis([0 50 -30 30]);  % Give the limits of both axis
title("Angle Theta vs Time"); % Give the title
xlabel('Time (sec)');  % Give X label as Time in seconds
ylabel('Angle Theta (degrees)');  % Give Y label as angles in degrees

subplot(3,1,3);  % Plot third figure in a subplot of 3 rows and 1 column
plot(s2.t,s2.Psi,'k');  % Plot time vs Psi for second scenario
axis([0 50 -50 250]);  % Give the limits of both axis
title("Angle Psi vs Time"); % Give the title
xlabel('Time (sec)');  % Give X label as Time in seconds
ylabel('Psi (degrees)');  % Give Y label as angles in degrees

sgtitle('Euler angles vs Time for jib actuated by motor - Scenario 2'); % Create the main title

% % Plot the motion of jib actuated by motor
figure(5);
plot(s2.t,s2.motion,'k'); % Plot time vs motion of jib for second scenario
axis([0 50 -10 300]);  % Give the limits of both axis
title("Angular displacement of Jib actuated by motor (degrees)"); % Give the title
xlabel('Time (sec)');  % Give X label as Time in seconds
ylabel('Angular displacement (degrees)');  % Give Y label as angle in degrees


%% Print the results

fprintf('<strong>                                                                                                                 </strong>\n');
fprintf('<strong>Results of Prescribed - Refer Figure 1 subplots for parameters </strong>\n');

fprintf('<strong>-----------------------------------------------------------------------------------------------------------------</strong>\n');
fprintf('<strong>Verification of model accuracy for Prescribed motion - Refer Figure 2 subplots for errors </strong>\n');
fprintf('Max error in Phi for component based vs signal based model = %f \n',max(s1.Phi_E)); % Print maximum error in Phi for first scenario
fprintf('Max error in Theta for component based vs signal based model = %f \n',max(s1.Tht_E)); % Print maximum error in Theta for first scenario
fprintf('Max error in Psi for component based vs signal based model  = %f \n',max(s1.Psi_E)); % Print maximum error in Psi for first scenario
fprintf('<strong>                                                                                                                 </strong>\n');

if max(s1.Phi_E) < 1e-8  && max(s1.Tht_E) < 1e-8  && max(s1.Psi_E) < 1e-8    % Print the conclusion
    fprintf('<strong>Both model results are identical so both models are implemented correctly for prescribed motion; Refer figure 2</strong>\n');
else
    fprintf('<strong>Both model results are not identical</strong>\n');
end
fprintf('<strong>                                                                                                                 </strong>\n');
fprintf('<strong>-----------------------------------------------------------------------------------------------------------------</strong>\n');
fprintf('<strong>Trolley force and Jib torque for Prescribed motion - Refer Figure 3 subplots </strong>\n');

fprintf('<strong>-----------------------------------------------------------------------------------------------------------------</strong>\n');
fprintf('<strong>Euler angles when jib is actuated by motor - Refer Figure 4 subplots for parameters </strong>\n');
fprintf('<strong>-----------------------------------------------------------------------------------------------------------------</strong>\n');

fprintf('<strong>Motion of Jib actuated by motor - Refer Figure 5 </strong>\n');
fprintf('<strong>-----------------------------------------------------------------------------------------------------------------</strong>\n');

