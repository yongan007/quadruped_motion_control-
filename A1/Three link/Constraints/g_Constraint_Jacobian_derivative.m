function TaskJacobian_derivative = g_Constraint_Jacobian_derivative(in1,in2)
%G_CONSTRAINT_JACOBIAN_DERIVATIVE
%    TASKJACOBIAN_DERIVATIVE = G_CONSTRAINT_JACOBIAN_DERIVATIVE(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    11-Mar-2021 14:55:42

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
v1 = in2(1,:);
v2 = in2(2,:);
v3 = in2(3,:);
t2 = cos(q1);
t3 = q1+q2;
t6 = v1+v2+v3;
t4 = cos(t3);
t5 = q3+t3;
t7 = cos(t5);
t9 = t4./2.0;
t8 = t7.*v3;
t10 = t7./2.0;
t12 = t4+t7;
t11 = -t8;
t13 = t8./2.0;
t15 = t12.*v2;
t17 = t9+t10;
t14 = -t13;
t16 = -t15;
t18 = t17.*v2;
t19 = -t18;
TaskJacobian_derivative = reshape([t14+t19-v1.*(t2./2.0+t17),t11+t16-v1.*(t2+t12),t14+t19-t17.*v1,t11+t16-t12.*v1,t6.*t7.*(-1.0./2.0),-t6.*t7],[2,3]);