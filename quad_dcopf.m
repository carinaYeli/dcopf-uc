function [cost,f_state,ge_out,power_flow,theta]=quad_dcopf(feasible_k_next_state,Time,load)
%this function is used for solving dcopf problem in unit commitment. 
% there are (2*gen_num+bus_num-1) variables in total, and the first three
% varibale are generator output in that period, and the next three
% generator output in next period. And the remainning varibales are angles
% of different buses. 
%% initilize variables
global mpc limit_line line_limit

bus_number=size(mpc.bus,1);
R=0.1*sum(load(:,Time));
gen_num=size(mpc.gen,1);
S=20*ones(1,gen_num)';% ramp up
T=15*ones(1,gen_num)';% ramp down
c_gencost=mpc.gencost(:,5)';
branch_from=mpc.branch(:,1)';
branch_to=mpc.branch(:,2)';
Vm=mpc.bus(:,8);
tap_branch=find(mpc.branch(:,9));
tap=ones(1,length(branch_from));
tap(tap_branch)=mpc.branch(tap_branch,9);
branch_x_tap=mpc.branch(:,4)'.*tap;
branch_x=mpc.branch(:,4)';
branchdata=mpc.branch(:,1:2)';
P_upper=mpc.gen(:,9);
P_lower=mpc.gen(:,10);
y=zeros(bus_number);
slack_bus=find(mpc.bus(:,2)==3);
y_linear_index=sub2ind(size(y),branchdata(1,:),branchdata(2,:));
y(y_linear_index)=1./branch_x';
y=y+y';
t1=diag(sum(y,2));
B_x=t1-y;
B_x=B_x*100;
%[Bbus,Bf,Pbusij,Pfinj]=makeBdc(mpc); used for checking the result of
%admittance matix
% B_x=Bbus;
B_x_reduced=B_x;
P_upper=mpc.gen(:,9).*feasible_k_next_state(1:gen_num)';
P_lower=mpc.gen(:,10).*feasible_k_next_state(1:gen_num)';
gen_array=zeros(bus_number,gen_num);
gen_array_index=sub2ind(size(gen_array),mpc.gen(:,1)',1:gen_num);
gen_array(gen_array_index)=1;

%% DCOPF problem formulation, set up equality and inequality constrains for DCOPF problem and 
f_obj=[mpc.gencost(:,6);zeros(gen_num+bus_number,1)];
f_h=sparse(1:gen_num,1:gen_num,2*c_gencost,bus_number+2*gen_num,bus_number+2*gen_num);
Aeq1=[gen_array, B_x,zeros(bus_number,gen_num)]; % power system network nodal balance constrains
Aeq2=zeros(1,2*gen_num+bus_number);
Aeq2(gen_num+slack_bus)=1;
Aeq=[Aeq1;Aeq2];

beq=[load(:,Time);0];
A1=[diag(ones(1,gen_num)),zeros(gen_num,gen_num+bus_number)]; % power output upper limit of that Time period
A2=[diag(-ones(1,gen_num)),zeros(gen_num,gen_num+bus_number)];% power output lower limit of that Time period
A3=[zeros(gen_num,gen_num+bus_number),diag(ones(1,gen_num))]; % power output upper limit of that Next period
A4=[zeros(gen_num,gen_num+bus_number),diag(-ones(1,gen_num))];% power output lower limit of that Next period
A5=[diag(ones(1,gen_num)),zeros(gen_num,bus_number-1),diag(-ones(1,gen_num))]; %ramp up limit
A6=[-diag(ones(1,gen_num)),zeros(gen_num,bus_number-1),diag(ones(1,gen_num))]; % ramp down limit
line_limitation=zeros(1,length(mpc.branch(:,1)));
line_limitation(limit_line)=line_limit;

% line_limitation(1)=150;
% line_limitation(2)=60;
% line_limitation(3)=70;
limit_branch_index=find(line_limitation);
limit_branch=branchdata(:,limit_branch_index);
A7=zeros(size(limit_branch,2),bus_number+2*gen_num);
B_x2=B_x;
B_x2(slack_bus,:)=0;
for i=1:length(limit_branch(1,:)) % congestion cosntrain, considering both direction of power flow
    a=limit_branch(1,i);
    A7(i,gen_num+a)=-B_x2(limit_branch(1,i),limit_branch(2,i));
    b=limit_branch(2,i);
    A7(i,gen_num+b)=B_x2(limit_branch(2,i),limit_branch(1,i));
end
 A8=-A7;
A9=[-P_upper'.*feasible_k_next_state(1:gen_num),zeros(1,gen_num+bus_number-1)];% required reserve
% A=[A1;A2;A3;A4;A7;A8];
A=[A1;A2;A3;A4;A8];
% A=[A1;A2;A3;A4;A5;A6;A8;A9]; 
b=[P_upper;P_lower;P_upper;P_lower;line_limitation(limit_branch_index)'];
% b=[P_upper;P_lower;P_upper;P_lower;T;S;line_limitation(limit_branch_index)';line_limitation(limit_branch_index)';(R+sum(load(:,Time)))];
lb=[];
ub=[];

[x1,fval,exitflag,output]=quadprog(f_h,f_obj,A,b,Aeq,beq,lb,ub);

%% calculate power flow and determine infeasible output

if exitflag==1 || exitflag==2
    f_state=feasible_k_next_state(gen_num+1);
    cost=fval+sum(mpc.gencost(:,7));
    ge_out=x1(1:gen_num)';
    answer=x1;
    theta=-x1(gen_num+1:gen_num+bus_number)';
    power_flow=(1./branch_x'.*(theta(branch_from)-theta(branch_to))')*mpc.baseMVA;
else
    f_state=0;
    cost=0;
    ge_out=zeros(1,gen_num);
    answer=zeros(bus_number+gen_num-1,1);
    power_flow=zeros(length(branchdata(1,:)),1);
    theta=zeros(1,bus_number);
end
I=find(feasible_k_next_state(1:gen_num));
infeasible=sum(any(ge_out(I)-P_upper(I)>eps| ge_out(I)-P_lower(I))<eps,2);
if infeasible>0
    ge_out=zeros(1,gen_num);
    cost=0;
    f_state=0;
end
infeasible=0;

