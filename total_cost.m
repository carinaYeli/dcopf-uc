function [cost_combination]=total_cost(T_total_cost,production_cost_combination,total_cost_combination,k_state,k_next_state)
global Time Max_feasible_states T an
if an == 3
    Time=T;
end
cost_combination=zeros(Max_feasible_states);
num_k_next_state=sum(any(k_next_state,2));
num_k_state=sum(any(k_state,2));
% if Time==1
%     cost_combination=production_cost_combination(Time,:)+T_total_cost(Time,:);
% else
%     cost_combination(1:num_k_state,1:num_k_next_state)=total_cost_combination(Time-1,1:num_k_state)'+production_cost_combination(Time,1:num_k_next_state)+T_total_cost(1:num_k_state,1:num_k_next_state);
% end
I=find(production_cost_combination(Time,:)');
P_cost=production_cost_combination(Time,I)';
if Time==1
    cost_combination=P_cost+T_total_cost(1:num_k_next_state,1:num_k_state);
else
    cost_combination(1:num_k_next_state,1:num_k_state)=total_cost_combination(Time-1,1:num_k_state)+production_cost_combination(Time,1:num_k_next_state)'+T_total_cost(1:num_k_next_state,1:num_k_state);
end

%  cost_combination(1:num_k_state,1:num_k_next_state)=production_cost_combination(Time,1:num_k_next_state)+T_total_cost(1:num_k_state,1:num_k_next_state);   
    