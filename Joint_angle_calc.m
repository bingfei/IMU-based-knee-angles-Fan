

XSENSPath = Uppath; 
run XSENS_Adapter

ModifyPath = strrep(XSENSPath,'.txt','.csv'); % change prefix
ModifyPara = csvread(ModifyPath); 
    
upGyrobias = ModifyPara(2,2:4);
lowGyrobias = ModifyPara(3,2:4);
    
currentbias = upGyrobias;
run fsm_fusion;

uLEG2.time = XSENS.time;
uLEG2.Ref_quaternion = quaternion;
uLEG2.euler = euler;   

uLEG = uLEG2;     

XSENSPath = Lowpath; 
run XSENS_Adapter

currentbias = lowGyrobias;
run fsm_fusion;

lLEG2.time = XSENS.time;
lLEG2.Ref_quaternion = quaternion;
lLEG2.euler = euler;   

lLEG = lLEG2;

%% Knee angle from visual3d
temp = importdata(V3DKneeAnglePath, '\t'); % 2 skip 2 rows.
KneeAngleV3D(:,1) = Knee_sign(1)*temp.data(:,4); % invert the order
KneeAngleV3D(:,2) = Knee_sign(2)*temp.data(:,3);
KneeAngleV3D(:,3) = Knee_sign(3)*temp.data(:,2);
[row,colum]=size(KneeAngleV3D);
SamplePeriod = 1/100;
time = ((0:row-1)*SamplePeriod)';

Kneebias = mean(KneeAngleV3D(200:300,:));
KneeAngleV3D(:,1) = KneeAngleV3D(:,1) - Kneebias(1);
KneeAngleV3D(:,2) = KneeAngleV3D(:,2) - Kneebias(2);
KneeAngleV3D(:,3) = KneeAngleV3D(:,3) - Kneebias(3);

if ~exist('kneefiguredisable','var')   
    figure('Name', 'V3D Knee angle');
    hold on;
    plot(time, KneeAngleV3D(:,1), 'r');
    plot(time, KneeAngleV3D(:,2), 'g');
    plot(time, KneeAngleV3D(:,3), 'b');
    
    title('V3D Knee angle');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    legend('I/E','A/A','F/E');
    hold off;
end

cali_euler = UpPoseRef/180*pi;
u0LEGq = (euler2quatern(cali_euler(1),cali_euler(2),cali_euler(3)))';
upQerror = quaternProd(quaternConj(uLEG.Ref_quaternion(100,:)),u0LEGq);

cali_euler = LowPoseRef/180*pi; 
l0LEGq = (euler2quatern(cali_euler(1),cali_euler(2),cali_euler(3)))';
lowQerror = quaternProd(quaternConj(lLEG.Ref_quaternion(100,:)),l0LEGq);	

LB_q_u = quaternProd(uLEG.Ref_quaternion,upQerror); 
LB_q_l = quaternProd(lLEG.Ref_quaternion,lowQerror); 

%%
if ~exist('autofiguredisable','var')   

Upeuler = quatern2euler(LB_q_u) * (180/pi);
figure('Name', 'Upeuler');
hold on;
plot(XSENS.time, Upeuler(:,1), 'r');
plot(XSENS.time, Upeuler(:,2), 'g');
plot(XSENS.time, Upeuler(:,3), 'b');
title('Upeuler');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;
end

if ~exist('autofiguredisable','var')  
Upeuler = quatern2euler(LB_q_l) * (180/pi);
figure('Name', 'Loweuler');
hold on;
plot(XSENS.time, Upeuler(:,1), 'r');
plot(XSENS.time, Upeuler(:,2), 'g');
plot(XSENS.time, Upeuler(:,3), 'b');
title('Loweuler');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;
end

q_knee = quaternProd(quaternConj(LB_q_u),LB_q_l);  % calculate the knee angles

KneeXYZ = quatern2eulerXYZ(q_knee) * (180/pi);

KneeXYZ(:,1) = Knee_sign(1)*KneeXYZ(:,1); 
KneeXYZ(:,2) = Knee_sign(2)*KneeXYZ(:,2);
KneeXYZ(:,3) = Knee_sign(3)*KneeXYZ(:,3);

IMUKneebias = mean(KneeXYZ(200:300,:));
KneeXYZ(:,1) = KneeXYZ(:,1) - IMUKneebias(1);
KneeXYZ(:,2) = KneeXYZ(:,2) - IMUKneebias(2);
KneeXYZ(:,3) = KneeXYZ(:,3) - IMUKneebias(3);

if ~exist('kneefiguredisable','var')   
figure('Name', 'IMU knee angles');
hold on;
plot(XSENS.time, KneeXYZ(:,1), 'r');
plot(XSENS.time, KneeXYZ(:,2), 'g');
plot(XSENS.time, KneeXYZ(:,3), 'b');

title('IMU knee angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('I/E','A/A','F/E');
hold off;

%% comparison
% plot as Euler angle form
figure('Name', 'Knee Angles comparison');
hold on;

plot(time, KneeAngleV3D(:,1), 'r--');
plot(time, KneeAngleV3D(:,2), 'g--');
h1 = plot(time, KneeAngleV3D(:,3), 'b--');

plot(time, KneeXYZ(:,1), 'r');
plot(time, KneeXYZ(:,2), 'g');
h2 = plot(time, KneeXYZ(:,3), 'b');

title('Knee Angles comparison');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend([h1,h2],'Vicon F/E', 'IMU F/E');
hold off;

% plot euler angle in one figure.
figure('Name', 'Euler Angles Comparison');
hold on;
plot(time, KneeAngleV3D(:,1), 'r--');
plot(time, KneeXYZ(:,1), 'k','LineWidth',2);
xlabel('Time (s)');
ylabel('I/E (deg)');
legend('Ref','Meas');
hold off;
    
figure('Name', 'Euler Angles Comparison');
hold on;
plot(time, KneeAngleV3D(:,2), 'r--');
plot(time, KneeXYZ(:,2), 'k','LineWidth',2);
xlabel('Time (s)');
ylabel('A/A (deg)');
legend('Ref','Meas');
hold off;

figure('Name', 'Euler Angles Comparison');
hold on;
plot(time, KneeAngleV3D(:,3), 'k--','LineWidth',2);
plot(time, KneeXYZ(:,3), 'r--','LineWidth',2);
xlabel('Time (s)');
ylabel('F/E(deg)');
legend('Ref','Meas');
hold off;
end

    ReferrorX = KneeAngleV3D(:,1) - KneeXYZ(:,1);
    ReferrorY = KneeAngleV3D(:,2) - KneeXYZ(:,2);
    ReferrorZ = KneeAngleV3D(:,3) - KneeXYZ(:,3);    
    if ~exist('kneefiguredisable','var')   
        figure('Name', 'IMU VS Vicon');
        hold on;
        plot(time, ReferrorX, 'r');
        plot(time, ReferrorY, 'g');
        plot(time, ReferrorZ, 'b');
        title('Error between IMU and Vicon');
        xlabel('Time (s)');
        ylabel('Angle (deg)');
        legend('\phi', '\theta', '\psi');
        hold off;  
    end    
    
 
%% End of script