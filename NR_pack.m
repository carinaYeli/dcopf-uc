function [result] = NR_pack
global mpc binding limit_line_num
gen_number=length(mpc.gen(:,1));
branch_number=length(mpc.branch(:,1));
result_best=[];
lastHourCost_byLimit=zeros(1,2^limit_line_num-1);
eps=1*10^(-5);
best_cost=0+eps;
for binding=1:2^limit_line_num-1
    [result]=DP_NR;
    lastHourCost_byLimit(binding)=result(24,2*gen_number+branch_number+2);
    if binding==1 && lastHourCost_byLimit(binding)~=0
        best_cost=lastHourCost_byLimit(binding);
        result_best=result;
    elseif sum(lastHourCost_byLimit(1:binding-1))==0 && lastHourCost_byLimit(binding)~=0
         best_cost=lastHourCost_byLimit(binding);
         result_best=result;
    end
    if binding>1
        if (lastHourCost_byLimit(binding)<best_cost && lastHourCost_byLimit(binding)~=0)
            best_cost=lastHourCost_byLimit(binding);
            result_best=result;
        end
    end
end
if  isempty(result_best)==0
    result = result_best;
end
