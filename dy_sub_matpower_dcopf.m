function [result]=dp(mpc,m)
% initialization of parameters 
global mpc initial_state Time Max_feasible_states load gen_min gen_max
time=24;
eps=1*10^(-6);
gen_num_list=mpc.gen(:,1)';
bus_num_list=mpc.gen(:,1)';
gen_num=size(mpc.gen,1);
bus_number=length(mpc.bus(:,1));
branch_num=length(mpc.branch(:,1));
states_num=2^gen_num;
combination=de2bi(0:states_num-1);%generate a combination of states according to their maximun capacity
combination_capacity=sum(mpc.gen(:,9)'.*combination,2);
[new_combination_capacity,Index_c]=sort(combination_capacity);%reorder total combinations according to their generation capacity
new_combination=combination(Index_c,:);
theta_combination=zeros(time,bus_number*Max_feasible_states);
initial_state=zeros(1,gen_num);
% initial_state(1)=1;
% initial_state(1)=1;
gen_min=mpc.gen(:,10); %generators' minimun and maximun output
gen_max=mpc.gen(:,9);

[load,total_load]=loadfile;






% determine feasible states in each step by figureing out which combination
% is larger than the total load in each step
Max_feasible_states=length(new_combination_capacity)-find(new_combination_capacity>min(total_load),1)+1;
feasible_Index_stage=zeros(1,time);
for i=1:time
    feasible_Index_stage(i)=find(new_combination_capacity>total_load(i),1);
end
feasible_states_number=states_num+1-feasible_Index_stage;  % accounting feasible combinations in each time
k_state=zeros(Max_feasible_states,gen_num+1); % used for store the feasible states in each time 
k_next_state=zeros(Max_feasible_states,gen_num+1); % used fot store feasible states in next time duration
% dynamic programming
feasible_combination=zeros(time,Max_feasible_states);% used for store feasible states for all time 
production_cost_combination=zeros(time,Max_feasible_states); % used for store production cost for each feasible states for all time, and which can use same index as feasible_combinations
total_cost_combination=zeros(time,Max_feasible_states);% used for store total cost for each feasible states for all time, and which can use same index with previous feasible_combianition 
T_total_cost=zeros(time,Max_feasible_states);% used for store transmission cost for each feasible states for all time, and which can use same index with previous feasible_combianition 
path_record=zeros(Max_feasible_states,2*time);% used for store minimum paths in each states , and do foreard tracking
power_combination=zeros(time,3*Max_feasible_states);
power_flow_combination=zeros(time,branch_num*Max_feasible_states);
result=zeros(time,2*gen_num+1);
pre=1;
Time=1;
while Time<=time 
    node_cost_combination=zeros(Max_feasible_states);%clear content,%used for determine the minimun cost for each node 
    %generate production cost of each feasible combination at each stages
    if Time==1
        k_state(1,1:gen_num)=initial_state;%set up the first state unit combinetion and output, for time one, k_state is the intial state
        k_state(1,gen_num+1)=2;
    end
    k_next_state(1:feasible_states_number(Time),1:gen_num)=new_combination(feasible_Index_stage(Time):(feasible_states_number(Time)+feasible_Index_stage(Time)-1),1:gen_num);
    k_next_state(1:feasible_states_number(Time),gen_num+1)=[feasible_Index_stage(Time):(feasible_Index_stage(Time)+feasible_states_number(Time)-1)]';
        k_state_row=sum(any(k_state,2)); % number of feasible states for previous time
        k_next_state_row=sum(any(k_next_state,2));
        infeasible=0;
        for kns=1:k_next_state_row % determine feasible states and their power output, production cost in each time using dcopf, based on the previpus feasible_combination result 
            feasible_k_next_state=k_next_state(kns,:);
            
            [cost,f_state,answer,power_flow]=quad_dcopf(feasible_k_next_state,Time,load);%generate production cost for each stage
             [cost,f_state,ge_out,power_flow,theta]=matpower_dcopf(feasible_k_next_state);
            if f_state==0 %set the cost of 
                infeasible=infeasible+1;
            end
            feasible_combination(Time,kns)=f_state; % store feasible states' name in feasiblecombination and set infeasible one to 0
            production_cost_combination(Time,kns)=cost; % store feaible states' production cost
            generator_output=ge_out;% store each state's generation output
            m1=(kns-1)*gen_num+1; % index for store each states' optimal power output
            m2=kns*gen_num;
            n1=(kns-1)*branch_num+1; % index for store each states' optimal power flow
            n2=kns*branch_num;
            l1=(kns-1)*bus_number+1;
            l2=kns*bus_number;
            power_combination(Time,m1:m2)=generator_output; 
            power_flow_combination(Time,n1:n2)=power_flow;
            theta_combination(Time,l1:l2)=theta;
        end
        if infeasible~=0 % beacuse some states are infeasible according to dcopf, update the result for feasible_combination and its related matrix by leaving non_zero elements from the begining of each row 
            feasible_k_next=feasible_combination(Time,(find(feasible_combination(Time,1:k_next_state_row))));
            cost_k_next=production_cost_combination(Time,(find(feasible_combination(Time,1:k_next_state_row))));% find non-zeros production cost of feasible state
            feasible_combination(Time,:)=0;
            production_cost_combination(Time,:)=0;
            feasible_combination(Time,1:length(feasible_k_next))=feasible_k_next;
            [index_row,column]=find(k_next_state(:,1+gen_num)==feasible_k_next);% find feasible states after dcopf
            new_k_next=k_next_state(index_row,:);%set a new variable for updating k_next_state
            k_next_state=zeros(Max_feasible_states,gen_num+1); 
            k_next_state(1:length(feasible_k_next),:)=new_k_next;% update k_next_state
            production_cost_combination(Time,1:length(feasible_k_next))=cost_k_next;
            for i=1:length(feasible_k_next)
                power_combination(Time,(i-1)*gen_num+1:i*gen_num)=power_combination(Time,(index_row(i)-1)*gen_num+1:index_row(i)*gen_num);
                power_flow_combination(Time,(i-1)*branch_num+1:i*branch_num)=power_flow_combination(Time,(index_row(i)-1)*branch_num+1:index_row(i)*branch_num);
                theta_combination(Time,(i-1)*bus_number+1:i*bus_number)=theta_combination(Time,(index_row(i)-1)*bus_number+1:index_row(i)*bus_number);
            end
        end
        %
        %do dynamic programming for each stage
        %update k_state, and k_next_state, becasue after dcopf, some states
        %maybe infeasible
        %% calculate transmissiona and total cost
            num_state_row=sum(any(k_state,2)); % number of feasible k_state
            num_next_state_row=sum(any(k_next_state,2));
            best_cost=realmax;
            [T_total_cost]=Transmission_Cost(k_state,k_next_state,Time); % calcuate transmission costs
            [cost_combination]=total_cost(T_total_cost,production_cost_combination, total_cost_combination,k_state,k_next_state); % add transmission cost with production cost and previous total cost 
            if Time==1 % find the minimun for all K states in that time period, and its states' index 
               [min_cost,k_pre_node_index]=min(cost_combination(1:num_next_state_row,1),[],2);
                total_cost_combination(Time,1:num_next_state_row)=min_cost';
            else
                [min_cost,k_pre_node_index]=min(cost_combination(1:num_next_state_row,1:sum(any(production_cost_combination(Time-1,:),1))),[],2); 
                total_cost_combination(Time,1:num_next_state_row)=min_cost';
            end
            k_pre_node=k_state(k_pre_node_index,gen_num+1)';
            k_next_node=k_next_state(:,gen_num+1)';
            k_next_node((find(~k_next_node)))=[];
            if Time==1 % store the current state and next states index in path record matrix
                path_record(1:num_next_state_row,(2*Time-1):2*Time)=[ones(num_next_state_row,1).*2,k_next_node'];
            else
                path_record(1:num_next_state_row,(2*Time-1):2*Time)=[k_pre_node',k_next_node'];
            end
%             k_state=zeros(Max_feasible_states,gen_num+1);
            k_state=k_next_state;
                k_next_state=zeros(Max_feasible_states,gen_num+1);
            Time=Time+1;
            T_total_cost=zeros(Max_feasible_states); % clear content 
end
        
near_zero=find(power_combination<eps);
power_combination(near_zero)=0;
%forward tracking
% find_state=feasible_combination(time,find_state_index);
path=zeros(1,time);
% do forward tracking of path_recod matrix to find the optimal path
for m=1:time
    n=time-m+1;
     if m==1
        [min_cost,find_state]=min(total_cost_combination(time,1:sum(any(total_cost_combination(time,:),1))));
%         from_state=2;
        to_state=path_record(find_state,2*time);
        from_state=path_record(find_state,2*time-m);
        path(1,n)=to_state;
%         find_state=1;
     else
        find_state=find(~(path_record(:,2*n)-from_state));
        to_state=from_state;
        from_state=path_record(find_state,2*n-1);
%         from_state=path_record(find_state,2*m-1);
        path(1,n)=to_state;
     end
        result(n,1:gen_num)=new_combination(path(n),:);
        Index_state=find(feasible_combination(n,:)==path(n));
        result(n,(gen_num+1):2*gen_num)=power_combination(n,((Index_state-1)*gen_num+1):Index_state*gen_num);
        result(n,2*gen_num+1:2*gen_num+branch_num)=power_flow_combination(n,((Index_state-1)*branch_num+1):Index_state*branch_num);
        result(n,2*gen_num+branch_num+1)=sum(power_combination(n,((Index_state-1)*gen_num+1):Index_state*gen_num));
        result(n,2*gen_num+branch_num+2)=total_cost_combination(n,Index_state);
        result(n,2*gen_num+branch_num+3:2*gen_num+branch_num+bus_number+2)=theta_combination(n,((Index_state-1)*bus_number+1):Index_state*bus_number);
end   

filename='projectdata.xlsx';
sheet=1;
xlRange='A2';
xlswrite(filename,result,sheet,xlRange)