% orgraniztion of data book 
% sheet 1: case 9
% sheet 2: case14
% sheet 3: case 30
% for each sheet line 2 to line 25 matpower result 
% line 27 to line 50 quaprog result
% line 52 to 75 newton raphson result 
close all;
global mpc cn an limit_line_num limit_line line_limit

time=24;
eps=1*10^(-6);
caseNumber=3;
algoNumber=3;
methodNanme=cell(1,3);
methodName={'matpower','quadprog','NR'};
mpcCase=cell(1,caseNumber);
mpcCase{1}=case9;
mpcCase{2}=case14;
mpcCase{3}=case30;
sheet=1;
filename='projectdata.xlsx';
% setup line limit martix 
limit_line=[]; % eneter the line that has limit 
line_limit=[];
limit_line_num=length(limit_line); % number of line that has limits 

for cn=1:caseNumber
    for an=1:algoNumber
        mpc=loadcase(mpcCase{cn});
        if an~=3
            [result]=dp(an);
        elseif an==3 && limit_line_num>=3 % if using NR and the number of line constrains euqal and lager than 3, the dynamic programming function needed to be called acrooding to the constarin combination, we need to call the package of NR algorithm
            [result] = NR_pack;
        elseif an==3 && limit_line_num<3
            [result]=DP_NR;
        end 
   %     write into excel data sheet
        if an == 1
            xlRange = 'A2';
        elseif an == 2
            xlRange = 'A27';
        elseif an == 3
            xlRange= 'A52';
        end      
        xlswrite(filename,result,sheet,xlRange)
    end
    sheet = sheet+1;
end

% do with three constrain

filename='projectdata1.xlsx';
for cn=1:caseNumber
    for an=1:algoNumber
        mpc=loadcase(mpcCase{cn});
        if cn==1
            limit_line=[1,4,5];
            line_limit=[120,120,50];   
        elseif cn==2
            limit_line=[1,3,4];
            line_limit=[150,70,50];
        elseif cn==3
            limit_line=[1,2,3];
            line_limit=[18,17,18];
        end
        limit_line_num=length(limit_line); % number of line that has limits 
        if an~=3
            [result]=dp(an);
        elseif an==3 && limit_line_num>=3 % if using NR and the number of line constrains euqal and lager than 3, the dynamic programming function needed to be called acrooding to the constarin combination, we need to call the package of NR algorithm
            [result] = NR_pack;
        elseif an==3 && limit_line_num<3
            [result]=DP_NR;
        end 
      %  write into excel data sheet
        if an == 1
            xlRange = 'A2';
        elseif an == 2
            xlRange = 'A27';
        elseif an == 3
            xlRange= 'A52';
        end      
        xlswrite(filename,result,sheet,xlRange)
    end
    sheet = sheet+1;
end
