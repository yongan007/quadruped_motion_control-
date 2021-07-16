function TaskJacobian = g_InverseKinematics_TaskJacobian(in1)
%G_INVERSEKINEMATICS_TASKJACOBIAN
%    TASKJACOBIAN = G_INVERSEKINEMATICS_TASKJACOBIAN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    11-Mar-2021 14:55:43

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = q1+q2;
t3 = q3+t2;
t4 = sin(t2);
t5 = sin(t3);
t6 = t4./2.0;
t7 = -t6;
t8 = t5./2.0;
t9 = -t8;
TaskJacobian = reshape([1.0,0.0,t7+t9-sin(q1)./2.0,0.0,1.0,t7+t9,0.0,0.0,t9],[3,3]);
