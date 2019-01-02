function [T_total_cost]=Transmission_Cost(k_state,k_next_state,Time)
global mpc Max_feasible_states
% mpc.gencost(:,2) = [700 100 300 800 1000]';

start_cost=mpc.gencost(:,2)';
gen_number=length(mpc.gen(:,1));
sd_cost=zeros(1,gen_number);
num_k_next_state=sum(any(k_next_state,2));
num_k_state=sum(any(k_state,2));
k_state((find(~any(k_state,2)))',:)=[]; %delete zeros rows
k_next_state((find(~any(k_next_state,2)))',:)=[];
T_total_cost=zeros(Max_feasible_states);
% for i=1:num_k_next_state
%     transfer=k_next_state(i,1:gen_number)-k_state(:,1:gen_number);
%     transfer2=k_next_state(i,1:gen_number)-k_state(:,1:gen_number);
%     transfer((find(transfer<=0)))=0;%find generator that didn't start and have been shut down
%     transfer2((find(transfer2>=0)))=0;
%     T_total_cost(1:num_k_state,i)=transfer*start_cost'+transfer2*(-sd_cost)';
% end
for i=1:num_k_state
    transfer=k_state(i,1:gen_number)-k_next_state(:,1:gen_number);
    transfer2=k_state(i,1:gen_number)-k_next_state(:,1:gen_number);
    transfer((find(transfer>=0)))=0;%find generator that didn't start and have been shut down
    transfer2((find(transfer2<=0)))=0;
    T_total_cost(1:num_k_next_state,i)=transfer*(-start_cost)'+transfer2*(sd_cost)';
end

    
% if Time==1
%     T=new_k_next_state(next,1:gen_number)-initial_state;
%     T(T==-1)=0;
%     T_cost=T*start_cost';
% else
%     T=new_k_next_state(next,1:gen_number)-k_state(pre,1:gen_number);
%     T(T==-1)=0;
%     T_cost=T*start_cost';
% end
%     
