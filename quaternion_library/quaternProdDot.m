function ab = quaternProdDot(a, b)
%QUATERNPROD Calculates the quaternion dot product

    ab(:,1) = a(:,1).*b(:,1)+a(:,2).*b(:,2)+a(:,3).*b(:,3)+a(:,4).*b(:,4);
end

