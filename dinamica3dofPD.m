function [Xout, qerror, Ut] = dinamica3dofPD(robot3dof, X, xd, dt, gp, gd, Ut) 
%  PD y PD + g

%Condiciones Iniciales
%X0 = [0; 0; 0; 0];

%Variables de Articulacion
q1 = X(1);
q2 = X(2);
q3 = X(3);

%error de posicion
qerror = [xd(1)- q1; xd(2)- q2; xd(3)-q3];

%Variables de velocidad
dq1 = X(4);
dq2 = X(5);
dq3 = X(6);

dqerror = [X(4); X(5); X(6)];

%Matriz de inercia

M = robot3dof.inertia([q1 q2 q3]);

%Matriz de coreolis

c = robot3dof.coriolis([(q1) (q2) (q3)], [dq1 dq2 dq3]);

%Pares gravitacionales

g = robot3dof.gravload([q1, q2, q3], [0,0,9.81]);
g = transpose(g);

%ganancias
kp = [gp, 0, 0; 0, gp, 0; 0, 0, gp];
kd = [gd, 0, 0; 0, gd, 0; 0, 0, gd];

%ley de control
u = kp*qerror - kd*dqerror;                            %PD +g
Ut = [Ut, u];

%Comportamiento
val = -inv(M)*(((c*[dq1; dq2; dq3])+g)-u);
dX = [dq1; dq2; dq3; val];
Xout = X + (dt*dX);
end