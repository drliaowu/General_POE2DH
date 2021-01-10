% Example of conversion from POE parameters to DH parameters for revolute,
% prismatic or helical joints 
%
% Please Refer to:
%
% Liao Wu, Ross Crawford, Jonathan Roberts. Geometric interpretation of the
% general POE model for a serial-link robot via conversion into D-H
% parameterization.

close all
clear
clc
%% prepare data
% POE parameters of the PUMA 560 robot are retrieved from K. Okamura and F.
% C. Park, “Kinematic calibration using the product of exponentials
% formula,” Robotica, vol. 14, no. 4, pp. 415–421, 1996. 

% nominal POE parameters
POE_nominal = [0     0     0     0     0     0     0
               0    -1    -1     0    -1     0     0
               1     0     0    -1     0    -1     0
               0     0     0    -50   -20   -50    250
               0     0     0     250   0     250   50
               0     0    -100   0    -250   0    -20]

% actual POE parameters
POE_actual = [0.04     0           0.178   0.062   0.001       0.095   0.02
              -0.02    -1.00002    -0.984  0.013   -1.00004    0.031   -0.01
              0.999    0           -0.001  -0.998  0           -0.995  0.01
              0.02     -0.02       -0.07   -51     -20.6        -51     249
              0.04     0           0.009   249     -0.0206     249     51
              0        0.05        -101    0.0752  -249        0       -20.6]


%% conversion from POE to DH

[DH_nominal, h_nominal, q_bar_nominal]= POE2DH_H(POE_nominal)

[DH_actual, h_actual, q_bar_actual]= POE2DH_H(POE_actual)

%% test one example configuration

% example joint angles
q = [pi/4;pi/4;pi/4;pi/4;pi/4;pi/4];

% forward kinematics of nominal POE parameters
fkPOE_nominal = fkPOE(POE_nominal,q)

% forward kinematics of nominal DH parameters
fkDH_nominal = fkDH_H(DH_nominal(2:7,:), h_nominal, DH(DH_nominal(1,:),'std'), DH([DH_nominal(8,1:2),0,0], 'std'), q, 'HHHHHH', 'std')

% normalized POE parameters
POE_normalized = [POE_actual(1:6,1)/q_bar_actual(1),POE_actual(1:6,2)/q_bar_actual(2),POE_actual(1:6,3)/q_bar_actual(3),POE_actual(1:6,4)/q_bar_actual(4),POE_actual(1:6,5)/q_bar_actual(5),POE_actual(1:6,6)/q_bar_actual(6),POE_actual(1:6,7)]

% forward kinematics of normalized POE parameters
fkPOE_actual = fkPOE(POE_normalized,q.*q_bar_actual)

% forward kinematics of actual DH parameters
fkDH_actual = fkDH_H(DH_actual(2:7,:), h_actual, DH(DH_actual(1,:),'std'), DH([DH_actual(8,1:2),0,0], 'std'), q.*q_bar_actual, 'HHHHHH', 'std')

%% accuracy evaluation

% rotational error for nominal parameters
er_n=zeros(100,1);
% translational error for nominal parameters
et_n=zeros(100,1);
% rotational error for actual parameters
er_a=zeros(100,1);
% translational error for actual parameters
et_a=zeros(100,1);

for i=1:100
    
    % random joint variables
    q=rand(1,6)*2*pi-pi;
    
    % forward kinematics of nominal POE parameters
    fkPOE_nominal = fkPOE(POE_nominal,q);

    % forward kinematics of nominal DH parameters
    fkDH_nominal = fkDH_H(DH_nominal(2:7,:), h_nominal, DH(DH_nominal(1,:),'std'), DH([DH_nominal(8,1:2),0,0], 'std'), q, 'HHHHHH', 'std');

    % compare two models with nominal parameters
    er_n(i)=norm(ROTM2EUL(fkDH_nominal(1:3,1:3)\fkPOE_nominal(1:3,1:3),'ZYX'));
    
    et_n(i)=norm(fkDH_nominal(1:3,4)-fkPOE_nominal(1:3,4))*0.001; %unit in m
    
    % forward kinematics of normalized POE parameters
    fkPOE_actual=fkPOE(POE_normalized,q.*q_bar_actual);    
    
    % forward kinematics of actual DH parameters
    fkDH_actual=fkDH_H(DH_actual(2:7,:), h_actual, DH(DH_actual(1,:),'std'), DH([DH_actual(8,1:2),0,0], 'std'), q.*q_bar_actual, 'HHHHHH', 'std');

    % compare two models with actual parameters
    er_a(i)=norm(ROTM2EUL(fkDH_actual(1:3,1:3)\fkPOE_actual(1:3,1:3),'ZYX'));
    
    et_a(i)=norm(fkDH_actual(1:3,4)-fkPOE_actual(1:3,4))*0.001; %unit in m
end

%% plotting
h1=figure(1);
hold on
plot(1:100,er_n,'linewidth',1.5)

h2=figure(2);
hold on
plot(1:100,et_n,'linewidth',1.5)

h3=figure(3);
hold on
plot(1:100,er_a,'linewidth',1.5)

h4=figure(4);
hold on
plot(1:100,et_a,'linewidth',1.5)