function euler = quatern2euler(q)
%QUATERN2EULER Converts a quaternion orientation to XYZ Euler angles
%updated, 20191202, match with diebel 2006, 8.11 Euler Angle Sequence (3,2,1)

    phi = atan2(-2.*q(:,2).*q(:,3)+2.*q(:,1).*q(:,4), -2.*q(:,3).^2-2.*q(:,4).^2 + 1);
    theta = asin(2.*q(:,2).*q(:,4)+2.*q(:,1).*q(:,3));
	psi = atan2(-2.*q(:,3).*q(:,4)+2.*q(:,1).*q(:,2), -2.*q(:,2).^2-2.*q(:,3).^2 + 1);
	
    euler = [phi(:,1) theta(:,1) psi(:,1)];
end

