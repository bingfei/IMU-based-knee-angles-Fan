% Initial quaternion calculation using acc and mag data.
function Initquat = InitquatCalc(Accelerometer, Magnetometer)
    length = size(Accelerometer,1);
    ax = Accelerometer(1,1);
    ay = Accelerometer(1,2);
    az = Accelerometer(1,3);
    
    mx = Magnetometer(1,1);
    my = Magnetometer(1,2);
    mz = Magnetometer(1,3);
    
    TempEuler(1) = atan2(ay,az);  
    TempEuler(2) = -atan(ax/sqrt(ay^2+az^2));
    % yaw calc, using the reference angle.
    phi = TempEuler(1,1);
    theta = TempEuler(1,2);
    TempEuler(3) = atan2((-my*cos(phi) + mz*sin(phi)),(mx*cos(theta)+my*sin(phi)*sin(theta)+mz*cos(phi)*sin(theta)));
    Initquat = euler2quatern(TempEuler(1),TempEuler(2),TempEuler(3))';
end



