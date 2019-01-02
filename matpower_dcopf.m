function [cost,f_state,ge_out,power_flow,theta]=matpower_dcopf(feasible_k_next_state)
%this function is used for solving dcopf problem in unit commitment. 
% there are (2*gen_num+bus_num-1) variables in total, and the first three
% varibale are generator output in that period, and the next three
% generator output in next period. And the remainning varibales are angles
% of different buses. 
%% initilize variables
global mpc load Time gen_min gen_max limit_line line_limit

eps=1*10^(-6);

bus_number=length(mpc.bus(:,1));
gen_num=length(mpc.gen(:,1));
branch_from=mpc.branch(:,1);
branch_to=mpc.branch(:,2);
branch_num=length(branch_to);
P_upper=gen_max.*feasible_k_next_state(1:gen_num)';
P_lower=gen_min.*feasible_k_next_state(1:gen_num)';

mpc.bus(:,3)=load(:,Time);%change load along with time

mpc.gen(:,9)=P_upper; % limit the infeasible units' output limit to zero
mpc.gen(:,10)=P_lower; % limit the infeasible units' output limit to zero
% add_constraints
mpopt=mpoption('pf.alg','NR','pf.tol',1e-6,'opf.dc.solver','OT','verbose',1); % 'OT' is using matlab Quadprog to solve optimization problem
define_constants;
if isempty(line_limit)==0
    mpc.branch(limit_line,6)=line_limit;
end

results=rundcopf(mpc,mpopt);

%% calculate power flow and determine infeasible output
% if exitflag==1 || exitflag==2
f_state=feasible_k_next_state(gen_num+1);
cost=results.f;
theta=results.x(1:bus_number)';
power_flow=results.branch(:,14)';

% branch_from=mpc.branch(:,1)';
% branch_to=mpc.branch(:,2)';
% tap_branch=find(mpc.branch(:,9));
% tap=ones(1,length(branch_from));
% tap(tap_branch)=mpc.branch(tap_branch,9);
% branch_x=mpc.branch(:,4)'.*tap;
% power_flow=1./branch_x'.*(theta(branch_to)-theta(branch_from))'.*mpc.baseMVA;

% else
%     f_state=0;
%     cost=0;
%     answer=zeros(bus_number+gen_num-1,1);
%     power_flow=zeros(length(branchdata(1,:)),1);
%     
% end
ge_out=results.gen(:,2)'; %generator output
I=find(feasible_k_next_state(1:gen_num));
infeasible=sum(any(ge_out(I)-P_upper(I)>eps| ge_out(I)-P_lower(I))<eps,2);
if abs(sum(ge_out)-sum(load(:,Time)))>eps
    infeasible=infeasible+1;
end
if   any(abs(power_flow(limit_line))- line_limit>eps,1)
    infeasible=infeasible+1;
end
if infeasible>0
    cost=0;
    f_state=0;
end