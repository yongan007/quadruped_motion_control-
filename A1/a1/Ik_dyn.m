clear ; close ;

%Get initial positionSRD_get_handler__IK_solution__interp1
InitialPosition = SRD_get('InitialPosition');
Handler_IK_Model = SRD_get('Handler_IK_Model');
Task = Handler_IK_Model.get_Task;
IC_Task = Task(InitialPosition);
% Handler_dynamics = SRD_get("Handler_dynamics_generalized_coordinates_model");


% H = Handler_dynamics.get_joint_space_inertia_matrix;
% t0 = 0;
% tf = 3;
% dt = 0.1;

TimeTable = linspace(0,2,50);


Goal_task = [0.55;...
             IC_Task(2);...
             -0.015];
         
  
Obs_pose  = [0.5;-0.1325;-0.13];     
Task_params = [Goal_task,Obs_pose];
Wieght = [7,1];
         
% [dist,diff_dist]= get_distance(FK_fun,q,xb);


IC_Task = Handler_IK_Model.get_Task(InitialPosition);


ZeroOrderDerivativeNodes = {IC_Task(1), Goal_task(1);
                            IC_Task(2), Goal_task(2);  
                            IC_Task(3), Goal_task(3)};  
%                             IC_Task(4), IC_Task(4);  
%                             IC_Task(5), IC_Task(5);  %IC_Task(5)-IC_Task(5)*2
%                             IC_Task(6), IC_Task(6)}; %IC_Task(6)+IC_Task(5)*2};
%                         
                        
FirstOrderDerivativeNodes = {0, 0; 
                             0, 0;  
                             0, 0;  
                             0, 0;  
                             0, 0;  
                             0, 0}; 
SecondOrderDerivativeNodes = {0, 0; 
                              0, 0; 
                              0, 0;  
                              0, 0;  
                              0, 0;  
                              0, 0}; 

TimeOfOneStage = 2;
TimeEnd = (size(ZeroOrderDerivativeNodes, 2) - 1)*TimeOfOneStage;
NodeTimes = (0:TimeOfOneStage:TimeEnd)';

Handler_IK_task = SRD_get_handler__IK_task__splines('NodeTimes', NodeTimes, ...
    'ZeroOrderDerivativeNodes', ZeroOrderDerivativeNodes, ...
    'FirstOrderDerivativeNodes', FirstOrderDerivativeNodes, ...
    'SecondOrderDerivativeNodes', SecondOrderDerivativeNodes, ...
    'OutOfBoundariesBehaviour', 'LastValue');

SRD_save(Handler_IK_task, 'Handler_IK_task');

% Task_velocity = task_velocity(IC_Task,Goal_task,tf);

[IK_Table,V_Table] = SRD_InverseKinematics_GenerateTable_ode(...
    'Task_params',Task_params,...
    'Wieght',Wieght,...
    'Handler_IK_task',Handler_IK_task,...
    'Handler_IK_Model', Handler_IK_Model, ...
    'InitialGuess', InitialPosition, ...
    'method', @SRD_InversePositionProblemSolver_Ode_Dynamics,...
    'TimeTable', TimeTable);

SRD_InverseKinematics_GenerateTable_tester_ode(...
    'Handler_IK_Model', Handler_IK_Model, ...
    'TimeTable', TimeTable, ...
    'IK_Table', IK_Table,...
    'V_Table', V_Table);

Handler_IK_Solution = SRD_get_handler__IK_solution__interp1_ode(...
    'Handler_IK_Model_name', 'Handler_IK_Model', ...
    'Handler_IK_task_name', 'Handler_IK_task', ...
    'IK_Table', IK_Table, ...
    'V_Table', V_Table, ...
    'TimeTable', TimeTable, ...
    'method', 'linear');

SRD_save(Handler_IK_Solution, 'Handler_IK_Solution_Ikdyn')


disp("goal pose: "+Goal_task);
disp("desired: "+Handler_IK_Model.get_Task(IK_Table(end,:)'));




