function Task = g_Constraint(in1)
%G_CONSTRAINT
%    TASK = G_CONSTRAINT(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    11-Mar-2021 14:55:41

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q1);
t3 = q1+q2;
t4 = cos(t3);
t5 = q3+t3;
t6 = cos(t5);
Task = [t2./2.0+t4./2.0+t6./2.0;t2+t4+t6];
