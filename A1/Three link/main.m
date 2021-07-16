clear ; close all;


InitialPosition = SRD_get('InitialPosition');
Handler_IK_Model = SRD_get('Handler_IK_Model');
IC_Task = Handler_IK_Model.get_Task(InitialPosition);

experiment_case = 1;

switch experiment_case
    
    case 1
        TimeTable =0:0.1:2;   

        Goal_task = [0.3;1.3];
        IK_solver = 'line' ;
        Task_params = [Goal_task,[]]; 
        Cube = 0;
            
    case 2
        Goal_task = [0.3;IC_Task(3)];

        Obs_pose  = [0.0;0];
        TimeTable = linspace(3,50);   

        Wieght=[3,0];
        Task_params = [Goal_task,[]];
        IK_solver = 'obstracle_avoidance' ;
        Cube = 0;
end


switch IK_solver 
    case 'line'
            Handler_IK_Solution = Get_IK_task_line('Handler_IK_Model',Handler_IK_Model,...
                                                  'InitialPosition',InitialPosition,...
                                                  'Goal_task',Goal_task,...
                                                  'TimeTable',TimeTable,...
                                                  'Enable_tester',true);
            SRD_save(Handler_IK_Solution, 'IK_Solution_line')
            Handler_IK_Solution = SRD_get('IK_Solution_line');

            [time_table_ode,x_table] = Get_Table_Simulation('InitialPosition',InitialPosition,...
                                                            'Handler_IK_Solution',Handler_IK_Solution,... 
                                                             'Enable_tester',true);

    case 'obstracle_avoidance'
            Handler_IK_Solution = Get_Ik_Solution('Handler_IK_Model',Handler_IK_Model,...
                                                  'InitialPosition',InitialPosition,...
                                                  'Goal_task',Goal_task,...
                                                  'Obs_pose',Obs_pose,...
                                                  'TimeTable',TimeTable,...
                                                  'Wieght',Wieght,...
                                                  'Enable_tester',true);
            SRD_save(Handler_IK_Solution, 'IK_Solution')
            Handler_IK_Solution = SRD_get('IK_Solution');
                                              
            [time_table_ode,x_table] = Get_Table_Simulation('InitialPosition',InitialPosition,...
                                                            'Handler_IK_Solution',Handler_IK_Solution,... 
                                                             'Enable_tester',true);
%             SRD_save(Handler_IK_Solution, 'Handler_IK_Solution_obs')
end 

Make_Animation(TimeTable,Handler_IK_Solution.State.IK_Table,Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'Default')

Make_Animation(time_table_ode,x_table(:,1:3),Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'Default')



