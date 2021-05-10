% import XSENS data.
%% Start of script

temp = importdata(XSENSPath, '\t', 6);
XSENSDATA = temp.data;

[row,column] = size(XSENSDATA);

SamplePeriod = 1/100;

XSENS = struct;
XSENS.time = ((0:row-1)*SamplePeriod)';
XSENS.Accelerometer = XSENSDATA(:,2:4)/9.8;  % convert the unit to 'g'  9.83757  9.80075

XSENS.Gyroscope = XSENSDATA(:,5:7)*180/pi;   % convert the unit to degree, compatible with XIMU and YISHEKUO.
XSENS.Magnetometer = XSENSDATA(:,8:10);
XSENS.Ref_quaternion = XSENSDATA(:,11:14);   
for t=1:row
    XSENS.Ref_quaternion(t,:) = XSENS.Ref_quaternion(t,:)/norm(XSENS.Ref_quaternion(t,:));
end

XSENS.euler = quatern2euler(XSENS.Ref_quaternion) * (180/pi);

% plot all the data.
if ~exist('autofiguredisable','var')   
    
    
figure('Name', 'XSENS Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(XSENS.time, XSENS.Gyroscope(:,1), 'r');
plot(XSENS.time, XSENS.Gyroscope(:,2), 'g');
plot(XSENS.time, XSENS.Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(XSENS.time, XSENS.Accelerometer(:,1), 'r');
plot(XSENS.time, XSENS.Accelerometer(:,2), 'g');
plot(XSENS.time, XSENS.Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(XSENS.time, XSENS.Magnetometer(:,1), 'r');
plot(XSENS.time, XSENS.Magnetometer(:,2), 'g');
plot(XSENS.time, XSENS.Magnetometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'x');


figure('Name', 'XSENS euler Angles');
hold on;
plot(XSENS.time, XSENS.euler(:,1), 'r');
plot(XSENS.time, XSENS.euler(:,2), 'g');
plot(XSENS.time, XSENS.euler(:,3), 'b');
title('XSENS euler Angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;
%
end

% plot Magnetometer magnitude and dipangle
if 1
    len = size(XSENS.Magnetometer,1);
    for t = 1:len
        XSENS.mag(t) = norm(XSENS.Magnetometer(t,:));
        XSENS.acc(t) = norm(XSENS.Accelerometer(t,:));
        XSENS.gyro(t) = norm(XSENS.Gyroscope(t,:));

    end
end


