dt = 0.05;
t = 0:dt:5;

X = [deg2rad(0); deg2rad(0); deg2rad(0); 0; 0; 0];

xd = [deg2rad(90); deg2rad(90); deg2rad(90)];

Xt = [];
Ut = [];
gp = 500; 
gd = 250;
qerrort = [];

RobotArticulado= SerialLink([
            Revolute('alpha', pi/2, 'a', 0, 'd', 0.04);
            Revolute('alpha', pi, 'a', 0.125, 'd', 0);
            Revolute('alpha', 0, 'a', 0.09, 'd', 0)]); % Modelo del robot

RobotArticulado.links(1).m = 0.1;
RobotArticulado.links(2).m = 0.06437;
RobotArticulado.links(2).m = 0.09;

RobotArticulado.links(1).r = [0.0041,0.0223,0.025];
RobotArticulado.links(2).r = [0.0586741,0.0001303,0.0077633];
RobotArticulado.links(3).r = [0.0249,0.0013,-0.0187];

RobotArticulado.links(1).I = [20.1352, 12.6255, 14.2124];
RobotArticulado.links(2).I = [1.3124,33.5593, 33.6655];
RobotArticulado.links(3).I = [6.3307, 17.8049, 12.9774];

for i = 1:length(t)
   [X, qerror, Ut] = dinamica3dofPD(RobotArticulado, X, xd, dt, gp, gd, Ut);   %PD 
   
   RobotArticulado.plot([X(1),X(2),X(3)]);
   title(t(i)+ " Segundos")
   
   %Vectores de estado, de señal de control y de error
   Xt = [Xt, X];
   qerrort = [qerrort, qerror];
   
   
end

figure(2)
subplot(2, 3, 1), plot(t, Xt(1, :)), title('q_1'), grid on
subplot(2, 3, 2), plot(t, Xt(2, :)), title('q_2'), grid on
subplot(2, 3, 3), plot(t, Xt(3, :)), title('q_3'), grid on
subplot(2, 3, 4), plot(t, qerrort(1, :)), title('q_1 error'), grid on
subplot(2, 3, 5), plot(t, qerrort(2, :)), title('q_2 error'), grid on
subplot(2, 3, 6), plot(t, qerrort(3, :)), title('q_3 error'), grid on
