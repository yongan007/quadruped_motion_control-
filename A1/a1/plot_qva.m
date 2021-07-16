
clear;close;
Handler_IK_task = SRD_get('Handler_IK_task');

Handler_IK_Solution = SRD_get('Handler_IK_Solution_Ikdyn_m');


dof = Handler_IK_Solution.dof_robot;
q = [];
v = [];
a=[];

time_table = v
for t = time_table
qi = Handler_IK_Solution.get_position(t);
q = [q,qi];

get_position_velocity_acceleration = Handler_IK_Solution.get_position_velocity_acceleration(t);
v = [v,get_position_velocity_acceleration(:,2)];
a = [a,get_position_velocity_acceleration(:,3)];

end

figure('Color', 'w', 'Name', 'GenerateTable tester 2')
subplot(2, 1, 1)
SRDgraphic_PlotGeneric(time_table, q', ...
    'NewFigure', false, ...
    'Title', 'IK solution', ...
    'LableVariable', 'q');
subplot(2, 1, 2)
SRDgraphic_PlotGeneric(time_table, v', ...
    'NewFigure', false, ...
    'Title', 'Joint Velocity', ...
    'LableVariable', '\dot{q}');

drawnow;

SRDgraphic_PlotGeneric(time_table, a', ...
    'NewFigure', true, ...
    'Title', 'Joint Acceleration', ...
    'LableVariable', '\ddot{q}');
