pitch = 10;
yaw = 0;
roll = 20;

deg2rad = pi/180;

C_bi = angle2dcm(yaw*deg2rad, pitch*deg2rad, roll*deg2rad);
bodyX = C_bi'*[1;0;0];
bodyY = C_bi'*[0;1;0];
bodyZ = C_bi'*[0;0;1];

alpha = -10;
beta = -20;
C_pb = angle2dcm(0, beta*deg2rad, alpha*deg2rad);
ipUnitVec = [0, 0, 1]'; % Unit vector of pendulum in its own frame
ipUnitVec_body = C_pb'*C_bi'*ipUnitVec;

% Inertial frame
quiver3(0, 0, 0, 1, 0, 0, color='k');
hold on;
quiver3(0, 0, 0, 0, 1, 0, color='k');
quiver3(0, 0, 0, 0, 0, 1, color='k');

% Body frame
quiver3(0, 0, 0, bodyX(1), bodyX(2), bodyX(3), color='r')
quiver3(0, 0, 0, bodyY(1), bodyY(2), bodyY(3), color=[0, 0.5, 0])
quiver3(0, 0, 0, bodyZ(1), bodyZ(2), bodyZ(3), color='b')

% Pendulum
quiver3(0, 0, 0, ...
    ipUnitVec_body(1), ...
    ipUnitVec_body(2), ...
    ipUnitVec_body(3), ...
    color=[1, 0, 1]);



C_test = angle2dcm(0, (pitch+alpha)*deg2rad, (roll+beta)*deg2rad);
ipUnitVec_test = C_test'*ipUnitVec;


quiver3(0, 0, 0, ...
    ipUnitVec_test(1), ...
    ipUnitVec_test(2), ...
    ipUnitVec_test(3), ...
    color=[0, 0.5, 1]);

hold off;

