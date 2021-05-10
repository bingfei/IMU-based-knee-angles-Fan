
% fsm fusion algorithm
% remove the gyro bias using no motion update

time = XSENS.time;
Accelerometer = XSENS.Accelerometer;
Gyroscope = XSENS.Gyroscope;
Magnetometer = XSENS.Magnetometer;
datalength = size(XSENS.Gyroscope,1);

Gyroscope(:,1) = Gyroscope(:,1) - currentbias(1);
Gyroscope(:,2) = Gyroscope(:,2) - currentbias(2);
Gyroscope(:,3) = Gyroscope(:,3) - currentbias(3);

AHRS = FSM_AHRS('SamplePeriod', 1/100, 'Kp', 0.1); %, 'Ki', 0.01
cleanflag = 0;
quaternion = zeros(length(time), 4);
AHRS.Quaternion = InitquatCalc(Accelerometer(1,:), Magnetometer(1,:));
quaternion(1,:) = AHRS.Quaternion;
for t = 2:length(time)
    AHRS.Updatefan2step(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:),0.005,0.2);
    quaternion(t, :) = AHRS.Quaternion;
end

euler = quatern2euler(quaternion) * (180/pi);	
if ~exist('autofiguredisable','var')   

% plot euler angle in one figure.
figure('Name', 'Euler Angles Comparison');
hold on;
plot(time, XSENS.euler(:,1), 'r');
plot(time, XSENS.euler(:,2), 'g');
plot(time, XSENS.euler(:,3), 'b');

plot(time, euler(:,1), 'r--');
plot(time, euler(:,2), 'g--');
plot(time, euler(:,3), 'b--');
title('XSENS VS Off-line');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;
end
