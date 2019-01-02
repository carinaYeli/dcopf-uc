function [cost,f_state,answer,power_flow,theta]=NR_dcopf(feasible_k_next_state)
global mpc binding T load limit_line_num limit_line line_limit
Time=T; % replace the global varibale in order to avios the conflic with previpus dynamic programming function 

eps1=1*10^(-5);
branch=mpc.branch(:,1:2);
branchdata=branch';
branch_from=mpc.branch(:,1)';
branch_to=mpc.branch(:,2)';
bus_number=max(mpc.bus(:,1));
y=zeros(bus_number);
y1=zeros(bus_number);
t=zeros(bus_number);
Vm=mpc.bus(:,8);
branch_x=mpc.branch(:,4)';
slack_bus=find(mpc.bus(:,2)==3);
ge_number=size(mpc.gencost,1);
gen_array=mpc.gen(:,1);
gen_min=mpc.gen(:,10)'; %generators' minimun and maximun output
gen_max=mpc.gen(:,9)';
tap_branch=find(mpc.branch(:,9));
tap=ones(1,length(branch_from));

tap(tap_branch)=mpc.branch(tap_branch,9);
% branch_x_tap=mpc.branch(:,4)'.*tap;

if limit_line_num>=3
    binding_combinaiton=de2bi(1:2^limit_line_num-1);
end

% mpc.gencost(:,5) = [0.0430292599 0.2 0.1 0.05 0.04]';
% mpc.gencost(:,6) = [20 30 25 40 35]';
% mpc.gencost(:,7) = [15 5 20 40 25]';
% generate the initial impedence matrix
%linear indexing to generate y matrix
y_linear_index=sub2ind(size(y),branchdata(1,:),branchdata(2,:));
y(y_linear_index)=1./branch_x';
y=y+y';
t1=diag(sum(y,2));
B_x=t1-y;
% B_x_reduced=B_x;
% B_x_reduced(slack_bus,:)=[];
% B_x_reduced(:,slack_bus


% B_x=t2-y;

%DCOPF 
%% calculate the power generation and power flow without line limitation
ge_num_array=mpc.gen(:,1)';
% A1=zeros(ge_number);
c_gencost=mpc.gencost(:,5)';
bus_from=mpc.branch(:,1)';
bus_to=mpc.branch(:,2)';
A1=diag(c_gencost).*feasible_k_next_state(1:ge_number);
A2=100*B_x;
ref_index=find(mpc.bus(:,2)==3);%determine slack bus 
A3=zeros(1,(ge_number+2*bus_number+1));
A3(ge_number+ref_index)=1;
A4=zeros(bus_number,ge_number);
A4_linear_index=sub2ind(size(A4),ge_num_array,[1:length(ge_num_array)]);
A4(A4_linear_index)=-1;
A4=A4.*feasible_k_next_state(1:ge_number);

%construct right hand side vector
b_gencost=mpc.gencost(:,6);
B1=-b_gencost;
B1=B1.*feasible_k_next_state(1:ge_number)';
B2=zeros(bus_number,1);   
B3=-load(:,Time);
left=zeros(ge_number+2*bus_number+1);
left(1:ge_number,1:ge_number)=A1;
left((ge_number+bus_number+1):(bus_number*2+ge_number),(ge_number+1):(ge_number+bus_number))=A2;
left((ge_number+bus_number+1):(ge_number+2*bus_number),1:ge_number)=A4;
left(ge_number+2*bus_number+1,:)=A3;
left=left+left';

right=zeros((ge_number+2*bus_number+1),1);
right((ge_number+1):(bus_number+ge_number),1)=B2;
right(1:ge_number,1)=B1;
right((ge_number+bus_number+1):(ge_number+2*bus_number))=B3;
answer_no_limit = pinv(left)*right;
% calculate the line power flow 
theta=answer_no_limit((ge_number+1):(ge_number+bus_number));
branch_number=length(mpc.branch(:,1));

Vm=mpc.bus(:,8);
branch_x=mpc.branch(:,4)';
power_flow=(1./branch_x'.*(theta(branch_from)-theta(branch_to))*mpc.baseMVA);
% set an line limitation between bus 4 and 5, bus 6 and 7.
total_line_limitation=zeros(1,branch_number);
if limit_line_num >= 3
% line_limit1=[150,60,70];
    limit_line1=limit_line.*binding_combinaiton(binding,:);
    line_limits=line_limit.*binding_combinaiton(binding,:);
    limit_line_use=limit_line1((find(binding_combinaiton(binding,:))));
    line_limit_use=line_limits((find(binding_combinaiton(binding,:))));
else
    limit_line_use=limit_line;
    line_limit_use=line_limit;
end
    
    

total_line_limitation(limit_line_use)=line_limit_use;

% total_line_limitation(2)=50;
% total_line_limitation(8)=17;
% new_Index_limitation=find(total_line_limitation);
% new_line_limitation=total_line_limitation(new_Index_limitation);
Index_limitation=find((abs(power_flow(limit_line_use)')-total_line_limitation(limit_line_use)>0));
line_limitation=total_line_limitation(limit_line_use(Index_limitation));

% determine if limitations have been 
% % % if isempty(Index_limitation==0)
% % %     answer=answer_no_limit;
% % % else
%% if violate limitation, calculate the generation with line limitations 
% construct left matrix and right vector of dcopf with line limitation
% add line limitations
%     Index_limitation=[7];
%     line_limitation=[150];
    %construct right hand side vector

    B4=line_limitation;
    left=zeros(ge_number+2*bus_number+1+length(Index_limitation));
    left(1:ge_number,1:ge_number)=A1;
    left((ge_number+bus_number+1):(bus_number*2+ge_number),(ge_number+1):(ge_number+bus_number))=A2;
    left((ge_number+bus_number+1):(ge_number+2*bus_number),1:ge_number)=A4;
    left(ge_number+2*bus_number+1,1:(ge_number+2*bus_number+1))=A3;
    if isempty(Index_limitation)==0
        A5=zeros(length(Index_limitation),ge_number+2*bus_number+1+length(Index_limitation));
        p=sign(power_flow(limit_line_use(Index_limitation)));
        index_from=sub2ind(size(A5),[1:length(Index_limitation)],(ge_number+bus_from(limit_line_use(Index_limitation))));
        index_to=sub2ind(size(A5),[1:length(Index_limitation)],(ge_number+bus_to(limit_line_use(Index_limitation))));
        A5(index_from)=100./branch_x(limit_line_use(Index_limitation)).*p';
        A5(index_to)=(-100./branch_x(limit_line_use(Index_limitation))).*p';
        left((ge_number+2*bus_number+2):(ge_number+2*bus_number+1+length(Index_limitation)),:)=A5;
       
    end
    left=left+left';
    right=zeros((ge_number+2*bus_number+1),1);
    right((ge_number+1):(bus_number+ge_number),1)=B2;
    right(1:ge_number,1)=B1;
    right((ge_number+bus_number+1):(ge_number+2*bus_number))=B3;
    if isempty(Index_limitation)==0
         right((ge_number+2*bus_number+2):(ge_number+2*bus_number+1+length(Index_limitation)))=B4;
    end
    answer = pinv(left)*right;
% % end

%% examine if the p_min and P_max are satisfied
ge_out=answer(1:ge_number);
I1=find(feasible_k_next_state(1:ge_number));
infeasible=sum(any(ge_out(I1)'>gen_max(I1) | ge_out(I1)'<gen_min(I1)),1);
if abs(sum(ge_out)-sum(load(:,Time)))>eps1
    infeasible=infeasible+1;
end

if infeasible==0
    f_state=feasible_k_next_state(ge_number+1);
%     theta=ones(1,bus_number);
%     theta(slack_bus)=0;
%     I=find(theta);
    theta=answer((ge_number+1):(ge_number+bus_number));
    cost=sum(c_gencost*answer(1:ge_number).^2)+sum(b_gencost'*answer(1:ge_number))+sum(mpc.gencost(:,7));
    power_flow=(1./branch_x'.*(theta(branch_from)-theta(branch_to))*mpc.baseMVA);
else
    f_state=0;
    cost=0;
    answer=zeros(bus_number+ge_number-1,1);
    power_flow=zeros(length(branchdata(1,:)),1);
    theta=zeros(bus_number,1);
end
if limit_line_num>=3
    if  infeasible==0 &&  any(abs(power_flow(limit_line((find(binding_combinaiton(binding,:)==0)))))-line_limit((find(binding_combinaiton(binding,:)==0)))'>eps1,1)
        infeasible=infeasible+1;
        f_state=0;
        cost=0;
        ge_out=zeros(ge_number,1);
        answer=zeros(bus_number+ge_number-1,1);
        power_flow=zeros(length(branchdata(1,:)),1);
        theta=zeros(bus_number,1);
    end
end


%% examine if the total genertation could satisfy total load 
% total_generation=sum(answer(1:ge_number));
% required_load=sum(load(:,Time));
% if abs(total_generation-required_load)>eps1
%     infeasible=infeasible+1;
% end
%% calculate total production cost

infeasible=0;


