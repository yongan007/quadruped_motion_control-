
function Handler_IK_Solution = Get_Ik_Solution(varargin)

Parser = inputParser;
Parser.FunctionName = 'Get_Ik_Solution';
Parser.addOptional('Handler_IK_Model', []);
Parser.addOptional('InitialPosition', []);
Parser.addOptional('Goal_task', []);
Parser.addOptional('Obs_pose', []);
Parser.addOptional('TimeTable', []);
Parser.addOptional('Wieght', [3,0]);
Parser.addOptional('Enable_tester', true);
% Parser.addOptional('method', @SRD_InversePositionProblemSolver_Ode_Dynamics);


Parser.parse(varargin{:});

InitialPosition = Parser.Results.InitialPosition;
Goal_task = Parser.Results.Goal_task;
Obs_pose = Parser.Results.Obs_pose;
Handler_IK_Model = Parser.Results.Handler_IK_Model;
TimeTable = Parser.Results.TimeTable;


Task_params = [Goal_task,Obs_pose];
Wieght = Parser.Results.Wieght;

if size(Obs_pose,2)>1
    solver = @SRD_InverseKinematics_GenerateTable_ode_m;
else
    solver = @SRD_InverseKinematics_GenerateTable_ode;
end
         

[IK_Table,V_Table] = solver(...
    'Task_params',Task_params,...
    'Wieght',Wieght,...
    'Handler_IK_Model', Handler_IK_Model, ...
    'InitialGuess', InitialPosition, ...
    'method', @SRD_InversePositionProblemSolver_Ode_Dynamics,...
    'TimeTable', TimeTable);

if Parser.Results.Enable_tester  
    SRD_InverseKinematics_GenerateTable_tester_ode(...
        'Handler_IK_Model', Handler_IK_Model, ...
        'TimeTable', TimeTable, ...
        'IK_Table', IK_Table,...
        'V_Table', V_Table);
    
disp("goal pose: "+Goal_task);
disp("desired: "+Handler_IK_Model.get_Task(IK_Table(end,:)'));
end

Handler_IK_Solution = SRD_get_handler__IK_solution__interp1_ode(...
    'Handler_IK_Model_name', 'Handler_IK_Model', ...
    'Handler_IK_task_name', 'Handler_IK_task', ...
    'IK_Table', IK_Table, ...
    'V_Table', V_Table, ...
    'TimeTable', TimeTable, ...
    'method', 'linear');


end


