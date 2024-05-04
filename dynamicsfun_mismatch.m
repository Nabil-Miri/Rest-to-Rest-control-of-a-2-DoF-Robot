function [Y]=dynamicsfun_mismatch(t,X,U)
b1=300; % was 200
b2=20; % was 50
b3=23.5;
b4=25;
b5=122.5;
c1=-25;
g1=784.8;
g2=245.3;
Y1=  X(2);
Y2= -(b5*U(1) - b3*U(2) - b5*g1*cos(X(1)) - b4*U(2)*cos(X(3)) + b3*g2*cos(X(1) + X(2)) - b5*g2*cos(X(1) + X(2)) + b4*g2*cos(X(1) + X(2))*cos(X(3)) + b3*c1*X(2)^2*sin(X(3)) + b5*c1*X(4)^2*sin(X(3)) + b4*c1*X(2)^2*cos(X(1))*sin(X(3)) + 2*b5*c1*X(2)*X(4)*sin(X(3)))/(b4^2*cos(X(3))^2 - b1*b5 + b3^2 - b2*b5*cos(X(3)) + 2*b3*b4*cos(X(3)));
Y3=  X(4);
Y4=  (b3*U(1) - b1*U(2) - b3*g1*cos(X(1)) - b2*U(2)*cos(X(3)) + b4*U(1)*cos(X(3)) + b1*g2*cos(X(1) + X(2)) - b3*g2*cos(X(1) + X(2)) + b2*g2*cos(X(1) + X(2))*cos(X(3)) - b4*g2*cos(X(1) + X(2))*cos(X(3)) - b4*g1*cos(X(1))*cos(X(3)) + b1*c1*X(2)^2*sin(X(3)) + b3*c1*X(4)^2*sin(X(3)) + b2*c1*X(2)^2*cos(X(3))*sin(X(3)) + b4*c1*X(4)^2*cos(X(3))*sin(X(3)) + 2*b3*c1*X(2)*X(4)*sin(X(3)) + 2*b4*c1*X(2)*X(4)*cos(X(3))*sin(X(3)))/(b4^2*cos(X(3))^2 - b1*b5 + b3^2 - b2*b5*cos(X(3)) + 2*b3*b4*cos(X(3)));
Y=[Y1;Y2;Y3;Y4];
end