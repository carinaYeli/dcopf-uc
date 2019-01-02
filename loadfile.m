function [load,total_load]=loadfile
global mpc cn
%% load of case 14
if cn == 2
    load1=mpc.bus(:,3)*0.1;
    load2=mpc.bus(:,3)*0.2;
    load3=mpc.bus(:,3)*0.3;
    load4=mpc.bus(:,3)*0.4;
    load5=mpc.bus(:,3)*0.5;
    load6=mpc.bus(:,3)*0.7;
    load7=mpc.bus(:,3)*0.8;
    load8=mpc.bus(:,3)*0.9;
    load9=mpc.bus(:,3);
    load10=mpc.bus(:,3)*1.1;
    load11=mpc.bus(:,3)*1.2;
    load12=mpc.bus(:,3)*1.2;
    load13=mpc.bus(:,3)*1.35;
    load14=mpc.bus(:,3)*1.5;
    load15=mpc.bus(:,3)*1.6;
    load16=mpc.bus(:,3)*1.7;
    load17=mpc.bus(:,3)*1.8;
    load18=mpc.bus(:,3)*1.9;
    load19=mpc.bus(:,3)*1.8;
    load20=mpc.bus(:,3)*1.5;
    load21=mpc.bus(:,3)*0.9;
    load22=mpc.bus(:,3)*0.8;
    load23=mpc.bus(:,3)*0.5;
    load24=mpc.bus(:,3)*0.3;
    load=0.7*[load1,load2,load3,load4,load5,load6,load7,load8,load9,load10,load11,load12,load13,load14,load15,load16,load17,load18,load19,load20,load21,load22,load23,load24];
    total_load=sum(load,1);
end

%% load of case 9
if cn == 1
    load1=mpc.bus(:,3)*0.3;
    load2=mpc.bus(:,3)*0.3;
    load3=mpc.bus(:,3)*0.3;
    load4=mpc.bus(:,3)*0.4;
    load5=mpc.bus(:,3)*0.5;
    load6=mpc.bus(:,3)*0.7;
    load7=mpc.bus(:,3)*0.8;
    load8=mpc.bus(:,3)*0.9;
    load9=mpc.bus(:,3);
    load10=mpc.bus(:,3)*1.1;
    load11=mpc.bus(:,3)*1.2;
    load12=mpc.bus(:,3)*1.2;
    load13=mpc.bus(:,3)*1.35;
    load14=mpc.bus(:,3)*1.5;
    load15=mpc.bus(:,3)*1.6;
    load16=mpc.bus(:,3)*1.65;
    load17=mpc.bus(:,3)*1.7;
    load18=mpc.bus(:,3)*1.8;
    load19=mpc.bus(:,3)*1.2;
    load20=mpc.bus(:,3);
    load21=mpc.bus(:,3)*0.9;
    load22=mpc.bus(:,3)*0.8;
    load23=mpc.bus(:,3)*0.5;
    load24=mpc.bus(:,3)*0.3;
    load=0.8*[load1,load2,load3,load4,load5,load6,load7,load8,load9,load10,load11,load12,load13,load14,load15,load16,load17,load18,load19,load20,load21,load22,load23,load24];
    total_load=sum(load,1);
end

%% load of case 30
if cn == 3
    load1=mpc.bus(:,3)*0.1;
    load2=mpc.bus(:,3)*0.2;
    load3=mpc.bus(:,3)*0.3;
    load4=mpc.bus(:,3)*0.4;
    load5=mpc.bus(:,3)*0.5;
    load6=mpc.bus(:,3)*0.7;
    load7=mpc.bus(:,3)*0.8;
    load8=mpc.bus(:,3)*0.9;
    load9=mpc.bus(:,3);
    load10=mpc.bus(:,3)*1.1;
    load11=mpc.bus(:,3)*1.2;
    load12=mpc.bus(:,3)*1.2;
    load13=mpc.bus(:,3)*1.3;
    load14=mpc.bus(:,3)*1.3;
    load15=mpc.bus(:,3)*1.4;
    load16=mpc.bus(:,3)*1.5;
    load17=mpc.bus(:,3)*1.6;
    load18=mpc.bus(:,3)*1.6;
    load19=mpc.bus(:,3)*1.5;
    load20=mpc.bus(:,3)*1.3;
    load21=mpc.bus(:,3)*0.9;
    load22=mpc.bus(:,3)*0.8;
    load23=mpc.bus(:,3)*0.5;
    load24=mpc.bus(:,3)*0.3;
    load=0.5*[load1,load2,load3,load4,load5,load6,load7,load8,load9,load10,load11,load12,load13,load14,load15,load16,load17,load18,load19,load20,load21,load22,load23,load24];
    total_load=sum(load,1);
end


