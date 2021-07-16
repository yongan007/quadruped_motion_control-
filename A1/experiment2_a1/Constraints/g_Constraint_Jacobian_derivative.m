function TaskJacobian_derivative = g_Constraint_Jacobian_derivative(in1,in2)
%G_CONSTRAINT_JACOBIAN_DERIVATIVE
%    TASKJACOBIAN_DERIVATIVE = G_CONSTRAINT_JACOBIAN_DERIVATIVE(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    06-Mar-2021 22:10:50

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
v1 = in2(1,:);
v2 = in2(2,:);
v3 = in2(3,:);
t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = sin(q1);
t6 = sin(q2);
t7 = sin(q3);
t8 = q2+q3;
t9 = cos(t8);
t10 = sin(t8);
t11 = t3.*3.602879701896397e+16;
t12 = t6.*3.602879701896397e+16;
t13 = -t12;
t14 = t9.*8.410755629360493e+15;
t15 = t10.*8.410755629360493e+15;
t16 = t9.*3.503331848935935e+16;
t17 = t10.*3.503331848935935e+16;
t18 = -t14;
t19 = -t17;
t20 = t15+t16;
t21 = t14+t19;
t22 = t11+t17+t18;
t23 = t13+t20;
t24 = (t2.*t21.*v3)./1.801439850948198e+17;
t25 = (t5.*t21.*v3)./1.801439850948198e+17;
t26 = -t24;
TaskJacobian_derivative = reshape([0.0,v1.*(t2.*8.505e-2-(t3.*t5)./5.0+(t3.*t5.*(t4.*2.334453638552917e-1-t7.*9.723699203978239e-1))./5.0-(t5.*t6.*(t4.*9.723699203978239e-1+t7.*2.334453638552917e-1))./5.0)+(t2.*t20.*v3)./1.801439850948198e+17+(t2.*t23.*v2)./1.801439850948198e+17,v1.*(t5.*8.505e-2+(t2.*t3)./5.0-t2.*t3.*t4.*4.668907277105834e-2+t2.*t3.*t7.*1.944739840795648e-1+t2.*t4.*t6.*1.944739840795648e-1+t2.*t6.*t7.*4.668907277105834e-2)+(t5.*t20.*v3)./1.801439850948198e+17+(t5.*t23.*v2)./1.801439850948198e+17,(t6.*v2)./5.0-t9.*v2.*1.944739840795648e-1-t9.*v3.*1.944739840795648e-1-t10.*v2.*4.668907277105834e-2-t10.*v3.*4.668907277105834e-2,t25+(t2.*t23.*v1)./1.801439850948198e+17-(t5.*t22.*v2)./1.801439850948198e+17,t26+(t2.*t22.*v2)./1.801439850948198e+17+(t5.*t23.*v1)./1.801439850948198e+17,(v2+v3).*(t3.*t4.*3.503331848935935e+16+t3.*t7.*8.410755629360493e+15+t4.*t6.*8.410755629360493e+15-t6.*t7.*3.503331848935935e+16).*(-5.551115123125783e-18),t25+(t2.*t20.*v1)./1.801439850948198e+17+(t5.*t21.*v2)./1.801439850948198e+17,t26-(t2.*t21.*v2)./1.801439850948198e+17+(t5.*t20.*v1)./1.801439850948198e+17],[3,3]);
