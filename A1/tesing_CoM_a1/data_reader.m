
clear;close;
load('Costs_experiment_case1.mat')
std_Max_Error_x= std(Max_Error_x,0,2);
std_Max_cost_u = std(Max_cost_u,0,2);
std_cost_qr = std(cost_qr);

mean_Max_Error_x= mean(Max_Error_x,2);
mean_Max_cost_u = mean(Max_cost_u,2);
mean_cost_qr = mean(cost_qr);

disp("<strong>Trajectory</strong>: "+ IK_solver)
disp("<strong>Number of experiment:</strong> "+ num_experiments)

 T = table([std_Max_Error_x(4:6)';mean_Max_Error_x(4:6)'],...
     [std_Max_cost_u(4:6)';mean_Max_cost_u(4:6)'],...
     [std_cost_qr;mean_cost_qr],...
     'VariableNames',{'Max error x','Max cost u','cost QR'},'RowName',{'Std','Mean'}); 
disp(T) 
disp("Execution time:"+tEnd+" seconds")


