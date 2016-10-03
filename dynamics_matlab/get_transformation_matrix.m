%%Input arguemets: theta, d, a, alpha
function [T] = get_transformation_matrix(theta, d, a ,alpha)
    T = [cos(theta), -sin(theta)*cosd(alpha), sin(theta)*sind(alpha), a*cos(theta);
        sin(theta), cos(theta)*cosd(alpha), -cos(theta)*sind(alpha), a*sin(theta);
        0, sind(alpha), cosd(alpha), d;
        0, 0, 0, 1];
end