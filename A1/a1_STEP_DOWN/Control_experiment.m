clear ; close ;

load('IKSolutions_experiment.mat')

% Handler_IK_Solution = SRD_get('Handler_IK_Solution_Ikdyn');
% x_table=

InitialPosition = Init_Poses;
Handler_IK_Solution = IK_Solutions;
for i = 1:length(IK_Solutions(1))    
    disp("Completed: "+i)

[time_table_ode,x_table,K_table, solution_tape,Q,R] = Get_Table_Simulation(InitialPosition(:,1),Handler_IK_Solution(1));

end 


% InitialPosition = SRD_get('InitialPosition');
Handler_IK_Model = SRD_get('Handler_IK_Model');


IC_Task = Handler_IK_Model.get_Task(InitialPosition);

Goal_task = [-0.2;...
             IC_Task(2);...
             -0.35];
         
Obs_pose  = [-0.15;IC_Task(2);-0.3138];
TimeTable = linspace(0,2,50);

Cube_origin = [0.3 -0.2 -0.3138];
Cube_size = [-0.45 0.5 -0.2];

Cube = [Cube_size;Cube_origin];
Task_params = [Goal_task,Obs_pose]; 

Make_Animation(TimeTable,solution_tape(:,1:12),Task_params,Cube,Handler_IK_Solution,Handler_IK_Model)
