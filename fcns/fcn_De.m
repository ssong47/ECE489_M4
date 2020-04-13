function [De] = fcn_De(q,p)

De = zeros(4,4);

  De(1,1)=p(24)/2 + p(30)/2 + p(36)/2 + p(31)/4 + p(37)/4 + p(26) + p(26)/2 + p(32)/4 + p(38)/4 +...
          p(4)^2*p(9) + p(4)^2*p(10) + p(4)^2*p(11) + (p(3)^2*p(9))/2 + (p(3)^2*p(10))/2 + (p(3)^2*p(11))/2 + (3*...
         p(5)^2*p(10))/4 + (3*p(5)^2*p(11))/4 + (3*p(11)*(p(6)^2 + p(7)^2))/4 + p(8)*p(12)^2 + (p(9)*p(15)^2)/2 +...
          (p(10)*p(18)^2)/2 + (p(11)*p(21)^2)/2 + p(8)*p(13)^2 + p(9)*p(16)^2 + (3*p(10)*p(19)^2)/4 + (3*p(11)*...
         p(22)^2)/4 + (p(9)*p(17)^2)/2 + (3*p(10)*p(20)^2)/4 + (3*p(11)*p(23)^2)/4 - (p(24)*cos(2*q(2)))/2 - (p(30)*...
         cos(2*q(2)))/2 - (p(36)*cos(2*q(2)))/2 + (p(31)*cos(2*q(2)))/4 - (p(31)*cos(2*q(3)))/4 + (p(37)*cos(2*...
         q(2)))/4 + (p(26)*cos(2*q(2)))/2 + (p(32)*cos(2*q(2)))/4 + (p(32)*cos(2*q(3)))/4 + (p(38)*cos(2*...
         q(2)))/4 - p(28)*sin(2*q(2)) + (p(35)*sin(2*q(3)))/2 - (p(31)*cos(2*q(2))*cos(2*q(3)))/4 - (p(37)*cos(2*q(3))*cos(2*...
         q(4)))/4 + (p(32)*cos(2*q(2))*cos(2*q(3)))/4 + (p(38)*cos(2*q(3))*cos(2*q(4)))/4 + (p(35)*cos(2*q(2))*sin(2*...
         q(3)))/2 + (p(41)*cos(2*q(3))*sin(2*q(4)))/2 + (p(41)*cos(2*q(4))*sin(2*q(3)))/2 - (3*p(11)*p(23)*(p(6)^2 +...
          p(7)^2)^(1/2))/2 + (p(37)*sin(2*q(3))*sin(2*q(4)))/4 - (p(38)*sin(2*q(3))*sin(2*q(4)))/4 + 2*p(4)*p(9)*p(16) + p(3)*...
         p(9)*p(15) + p(3)*p(10)*p(18) + p(3)*p(11)*p(21) - (3*p(5)*p(10)*p(20))/2 + (p(3)^2*p(9)*cos(2*q(2)))/2 +...
          (p(3)^2*p(10)*cos(2*q(2)))/2 + (p(3)^2*p(11)*cos(2*q(2)))/2 - (p(5)^2*p(10)*cos(2*q(2)))/4 - (p(5)^2*p(10)*...
         cos(2*q(3)))/4 - (p(5)^2*p(11)*cos(2*q(2)))/4 - (p(5)^2*p(11)*cos(2*q(3)))/4 - (p(11)*cos(2*q(2))*...
         (p(6)^2 + p(7)^2))/4 + (p(9)*p(15)^2*cos(2*q(2)))/2 + (p(10)*p(18)^2*cos(2*q(2)))/2 + (p(11)*p(21)^2*cos(2*...
         q(2)))/2 - (p(10)*p(19)^2*cos(2*q(2)))/4 + (p(10)*p(19)^2*cos(2*q(3)))/4 - (p(11)*p(22)^2*cos(2*q(2)))/4 - (p(9)*...
         p(17)^2*cos(2*q(2)))/2 - (p(10)*p(20)^2*cos(2*q(2)))/4 - (p(10)*p(20)^2*cos(2*q(3)))/4 - (p(11)*p(23)^2*...
         cos(2*q(2)))/4 - p(34)*sin(2*q(2))*cos(q(3)) - p(33)*sin(2*q(2))*sin(q(3)) - (p(11)*p(22)^2*sin(2*q(3))*...
         sin(2*q(4)))/4 + (p(11)*p(23)^2*sin(2*q(3))*sin(2*q(4)))/4 + 2*p(4)*p(5)*p(10)*sin(q(3)) + 2*p(4)*p(5)*...
         p(11)*sin(q(3)) + 2*p(4)*p(10)*p(19)*cos(q(3)) - (3*p(5)*p(11)*p(23)*cos(q(4)))/2 - (p(37)*cos(2*q(2))*...
         cos(2*q(3))*cos(2*q(4)))/4 + (p(38)*cos(2*q(2))*cos(2*q(3))*cos(2*q(4)))/4 - 2*p(4)*p(10)*p(20)*...
         sin(q(3)) - (3*p(5)*p(11)*p(22)*sin(q(4)))/2 + (p(41)*cos(2*q(2))*cos(2*q(3))*sin(2*q(4)))/2 + (p(41)*cos(2*q(2))*...
         cos(2*q(4))*sin(2*q(3)))/2 + (p(11)*p(23)*cos(2*q(2))*(p(6)^2 + p(7)^2)^(1/2))/2 + (p(37)*cos(2*q(2))*...
         sin(2*q(3))*sin(2*q(4)))/4 - (p(38)*cos(2*q(2))*sin(2*q(3))*sin(2*q(4)))/4 - p(40)*sin(2*q(2))*cos(q(3))*...
         cos(q(4)) + p(3)*p(9)*p(15)*cos(2*q(2)) + p(3)*p(10)*p(18)*cos(2*q(2)) + p(3)*p(11)*p(21)*cos(2*q(2)) + (p(5)*...
         p(10)*p(20)*cos(2*q(2)))/2 + (p(5)*p(10)*p(20)*cos(2*q(3)))/2 - p(39)*sin(2*q(2))*cos(q(3))*...
         sin(q(4)) - p(39)*sin(2*q(2))*cos(q(4))*sin(q(3)) + p(3)*p(9)*p(17)*sin(2*q(2)) + (p(5)*p(10)*p(19)*sin(2*q(3)))/2 +...
          p(40)*sin(2*q(2))*sin(q(3))*sin(q(4)) - (p(5)^2*p(10)*cos(2*q(2))*cos(2*q(3)))/4 - (p(5)^2*p(11)*cos(2*...
         q(2))*cos(2*q(3)))/4 - (p(11)*cos(2*q(3))*cos(2*q(4))*(p(6)^2 + p(7)^2))/4 + p(9)*p(15)*p(17)*sin(2*...
         q(2)) - (p(10)*p(19)*p(20)*sin(2*q(3)))/2 + (3*p(5)*p(11)*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2))/2 + (p(11)*sin(2*...
         q(3))*sin(2*q(4))*(p(6)^2 + p(7)^2))/4 + (p(10)*p(19)^2*cos(2*q(2))*cos(2*q(3)))/4 + (p(11)*p(22)^2*cos(2*...
         q(3))*cos(2*q(4)))/4 - (p(10)*p(20)^2*cos(2*q(2))*cos(2*q(3)))/4 - (p(11)*p(23)^2*cos(2*q(3))*cos(2*...
         q(4)))/4 - (p(10)*p(19)*p(20)*cos(2*q(2))*sin(2*q(3)))/2 - (p(11)*p(22)*p(23)*cos(2*q(3))*sin(2*q(4)))/2 - (p(11)*...
         p(22)*p(23)*cos(2*q(4))*sin(2*q(3)))/2 + 2*p(4)*p(11)*p(22)*cos(q(3))*cos(q(4)) - (p(5)*p(11)*cos(2*q(2))*...
         cos(q(4))*(p(6)^2 + p(7)^2)^(1/2))/2 - (p(5)*p(11)*cos(2*q(3))*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2))/2 + (p(11)*...
         cos(2*q(2))*sin(2*q(3))*sin(2*q(4))*(p(6)^2 + p(7)^2))/4 + (p(11)*p(22)^2*cos(2*q(2))*cos(2*q(3))*cos(2*...
         q(4)))/4 - (p(11)*p(23)^2*cos(2*q(2))*cos(2*q(3))*cos(2*q(4)))/4 - 2*p(4)*p(11)*p(23)*cos(q(3))*sin(q(4)) - 2*p(4)*...
         p(11)*p(23)*cos(q(4))*sin(q(3)) - 2*p(4)*p(11)*p(22)*sin(q(3))*sin(q(4)) + (p(5)*p(11)*sin(2*q(3))*...
         sin(q(4))*(p(6)^2 + p(7)^2)^(1/2))/2 - (p(11)*p(22)^2*cos(2*q(2))*sin(2*q(3))*sin(2*q(4)))/4 + (p(11)*p(23)^2*...
         cos(2*q(2))*sin(2*q(3))*sin(2*q(4)))/4 - p(3)*p(5)*p(10)*sin(2*q(2))*cos(q(3)) - p(3)*p(5)*p(11)*sin(2*...
         q(2))*cos(q(3)) + (p(5)*p(11)*p(23)*cos(2*q(2))*cos(q(4)))/2 + (p(5)*p(11)*p(23)*cos(2*q(3))*...
         cos(q(4)))/2 + p(3)*p(10)*p(20)*sin(2*q(2))*cos(q(3)) - p(5)*p(10)*p(18)*sin(2*q(2))*cos(q(3)) - p(5)*p(11)*p(21)*...
         sin(2*q(2))*cos(q(3)) + (p(5)*p(11)*p(22)*cos(2*q(2))*sin(q(4)))/2 + (p(5)*p(11)*p(22)*cos(2*q(3))*...
         sin(q(4)))/2 + (p(5)*p(11)*p(22)*sin(2*q(3))*cos(q(4)))/2 + p(3)*p(10)*p(19)*sin(2*q(2))*sin(q(3)) - (p(5)*p(11)*...
         p(23)*sin(2*q(3))*sin(q(4)))/2 + (p(11)*p(23)*cos(2*q(3))*cos(2*q(4))*(p(6)^2 + p(7)^2)^(1/2))/2 + (p(11)*...
         p(22)*cos(2*q(3))*sin(2*q(4))*(p(6)^2 + p(7)^2)^(1/2))/2 + (p(11)*p(22)*cos(2*q(4))*sin(2*q(3))*(p(6)^2 +...
          p(7)^2)^(1/2))/2 + p(10)*p(18)*p(20)*sin(2*q(2))*cos(q(3)) - (p(11)*p(23)*sin(2*q(3))*sin(2*q(4))*(p(6)^2 +...
          p(7)^2)^(1/2))/2 + 2*p(4)*p(11)*cos(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + 2*p(4)*p(11)*cos(q(4))*sin(q(3))*...
         (p(6)^2 + p(7)^2)^(1/2) + p(10)*p(18)*p(19)*sin(2*q(2))*sin(q(3)) + (p(5)*p(10)*p(20)*cos(2*q(2))*cos(2*...
         q(3)))/2 + (p(5)*p(10)*p(19)*cos(2*q(2))*sin(2*q(3)))/2 - (p(11)*cos(2*q(2))*cos(2*q(3))*cos(2*q(4))*(p(6)^2 +...
          p(7)^2))/4 + p(3)*p(11)*sin(2*q(2))*sin(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(11)*p(21)*sin(2*q(2))*...
         cos(q(3))*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2) + p(11)*p(21)*sin(2*q(2))*sin(q(3))*sin(q(4))*(p(6)^2 +...
          p(7)^2)^(1/2) - (p(11)*p(22)*p(23)*cos(2*q(2))*cos(2*q(3))*sin(2*q(4)))/2 - (p(11)*p(22)*p(23)*cos(2*q(2))*cos(2*q(4))*...
         sin(2*q(3)))/2 - (p(5)*p(11)*cos(2*q(2))*cos(2*q(3))*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2))/2 + p(3)*p(11)*...
         p(23)*sin(2*q(2))*cos(q(3))*cos(q(4)) + (p(5)*p(11)*cos(2*q(2))*sin(2*q(3))*sin(q(4))*(p(6)^2 +...
          p(7)^2)^(1/2))/2 + p(3)*p(11)*p(22)*sin(2*q(2))*cos(q(3))*sin(q(4)) + p(3)*p(11)*p(22)*sin(2*q(2))*cos(q(4))*...
         sin(q(3)) - p(3)*p(11)*p(23)*sin(2*q(2))*sin(q(3))*sin(q(4)) + p(11)*p(21)*p(23)*sin(2*q(2))*cos(q(3))*cos(q(4)) +...
          p(11)*p(21)*p(22)*sin(2*q(2))*cos(q(3))*sin(q(4)) + p(11)*p(21)*p(22)*sin(2*q(2))*cos(q(4))*...
         sin(q(3)) - p(11)*p(21)*p(23)*sin(2*q(2))*sin(q(3))*sin(q(4)) + (p(5)*p(11)*p(23)*cos(2*q(2))*cos(2*q(3))*...
         cos(q(4)))/2 + (p(5)*p(11)*p(22)*cos(2*q(2))*cos(2*q(3))*sin(q(4)))/2 + (p(5)*p(11)*p(22)*cos(2*q(2))*sin(2*q(3))*...
         cos(q(4)))/2 - (p(5)*p(11)*p(23)*cos(2*q(2))*sin(2*q(3))*sin(q(4)))/2 + (p(11)*p(23)*cos(2*q(2))*cos(2*q(3))*cos(2*q(4))*...
         (p(6)^2 + p(7)^2)^(1/2))/2 + (p(11)*p(22)*cos(2*q(2))*cos(2*q(3))*sin(2*q(4))*(p(6)^2 + p(7)^2)^(1/2))/2 +...
          (p(11)*p(22)*cos(2*q(2))*cos(2*q(4))*sin(2*q(3))*(p(6)^2 + p(7)^2)^(1/2))/2 - (p(11)*p(23)*cos(2*q(2))*...
         sin(2*q(3))*sin(2*q(4))*(p(6)^2 + p(7)^2)^(1/2))/2 - p(3)*p(11)*sin(2*q(2))*cos(q(3))*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2);
  De(1,2)=p(29)*cos(q(2)) - p(27)*sin(q(2)) - p(33)*cos(q(3))*sin(q(2)) + p(34)*sin(q(2))*...
         sin(q(3)) + p(35)*cos(2*q(3))*cos(q(2)) + (p(31)*sin(2*q(3))*cos(q(2)))/2 - (p(32)*sin(2*q(3))*...
         cos(q(2)))/2 - p(41)*sin(2*q(3))*sin(2*q(4))*cos(q(2)) + p(4)*p(3)*p(9)*sin(q(2)) + p(4)*p(3)*p(10)*sin(q(2)) + p(4)*...
         p(3)*p(11)*sin(q(2)) - p(39)*cos(q(3))*cos(q(4))*sin(q(2)) - p(4)*p(9)*p(17)*cos(q(2)) + p(40)*cos(q(3))*...
         sin(q(2))*sin(q(4)) + p(40)*cos(q(4))*sin(q(2))*sin(q(3)) + p(4)*p(9)*p(15)*sin(q(2)) + p(4)*p(10)*p(18)*...
         sin(q(2)) + p(4)*p(11)*p(21)*sin(q(2)) + p(3)*p(9)*p(16)*sin(q(2)) + p(39)*sin(q(2))*sin(q(3))*...
         sin(q(4)) - p(9)*p(16)*p(17)*cos(q(2)) + (p(5)^2*p(10)*sin(2*q(3))*cos(q(2)))/2 + (p(5)^2*p(11)*sin(2*q(3))*...
         cos(q(2)))/2 + p(9)*p(15)*p(16)*sin(q(2)) - (p(10)*p(19)^2*sin(2*q(3))*cos(q(2)))/2 + (p(10)*p(20)^2*sin(2*q(3))*...
         cos(q(2)))/2 + p(41)*cos(2*q(3))*cos(2*q(4))*cos(q(2)) + (p(37)*cos(2*q(3))*sin(2*q(4))*cos(q(2)))/2 + (p(37)*...
         cos(2*q(4))*sin(2*q(3))*cos(q(2)))/2 - (p(38)*cos(2*q(3))*sin(2*q(4))*cos(q(2)))/2 - (p(38)*cos(2*q(4))*...
         sin(2*q(3))*cos(q(2)))/2 - p(4)*p(10)*p(20)*cos(q(2))*cos(q(3)) + p(3)*p(5)*p(10)*sin(q(2))*sin(q(3)) +...
          p(3)*p(5)*p(11)*sin(q(2))*sin(q(3)) - p(4)*p(10)*p(19)*cos(q(2))*sin(q(3)) + p(3)*p(10)*p(19)*cos(q(3))*...
         sin(q(2)) - p(3)*p(10)*p(20)*sin(q(2))*sin(q(3)) + p(5)*p(10)*p(18)*sin(q(2))*sin(q(3)) + p(5)*p(11)*p(21)*sin(q(2))*...
         sin(q(3)) + p(10)*p(18)*p(19)*cos(q(3))*sin(q(2)) - p(10)*p(18)*p(20)*sin(q(2))*sin(q(3)) + p(5)*p(10)*p(19)*...
         cos(2*q(3))*cos(q(2)) - p(5)*p(10)*p(20)*sin(2*q(3))*cos(q(2)) - p(10)*p(19)*p(20)*cos(2*q(3))*cos(q(2)) +...
          (p(11)*cos(2*q(3))*sin(2*q(4))*cos(q(2))*(p(6)^2 + p(7)^2))/2 + (p(11)*cos(2*q(4))*sin(2*q(3))*cos(q(2))*...
         (p(6)^2 + p(7)^2))/2 - (p(11)*p(22)^2*cos(2*q(3))*sin(2*q(4))*cos(q(2)))/2 - (p(11)*p(22)^2*cos(2*q(4))*...
         sin(2*q(3))*cos(q(2)))/2 + (p(11)*p(23)^2*cos(2*q(3))*sin(2*q(4))*cos(q(2)))/2 + (p(11)*p(23)^2*cos(2*...
         q(4))*sin(2*q(3))*cos(q(2)))/2 + p(4)*p(5)*p(10)*cos(q(2))*cos(q(3)) + p(4)*p(5)*p(11)*cos(q(2))*...
         cos(q(3)) + p(4)*p(11)*p(23)*cos(q(2))*sin(q(3))*sin(q(4)) - p(3)*p(11)*p(23)*cos(q(3))*sin(q(2))*...
         sin(q(4)) - p(3)*p(11)*p(23)*cos(q(4))*sin(q(2))*sin(q(3)) - p(3)*p(11)*p(22)*sin(q(2))*sin(q(3))*sin(q(4)) + p(11)*...
         p(21)*p(22)*cos(q(3))*cos(q(4))*sin(q(2)) - p(11)*p(21)*p(23)*cos(q(3))*sin(q(2))*sin(q(4)) - p(11)*p(21)*...
         p(23)*cos(q(4))*sin(q(2))*sin(q(3)) - p(11)*p(21)*p(22)*sin(q(2))*sin(q(3))*sin(q(4)) + p(5)*p(11)*p(22)*...
         cos(2*q(3))*cos(q(2))*cos(q(4)) - p(5)*p(11)*p(23)*cos(2*q(3))*cos(q(2))*sin(q(4)) - p(5)*p(11)*p(23)*...
         sin(2*q(3))*cos(q(2))*cos(q(4)) - p(5)*p(11)*p(22)*sin(2*q(3))*cos(q(2))*sin(q(4)) + p(11)*p(22)*cos(2*...
         q(3))*cos(2*q(4))*cos(q(2))*(p(6)^2 + p(7)^2)^(1/2) - p(11)*p(23)*cos(2*q(3))*sin(2*q(4))*cos(q(2))*...
         (p(6)^2 + p(7)^2)^(1/2) - p(11)*p(23)*cos(2*q(4))*sin(2*q(3))*cos(q(2))*(p(6)^2 + p(7)^2)^(1/2) + p(4)*p(11)*...
         cos(q(2))*cos(q(3))*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(11)*p(22)*sin(2*q(3))*sin(2*q(4))*cos(q(2))*...
         (p(6)^2 + p(7)^2)^(1/2) - p(4)*p(11)*cos(q(2))*sin(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + p(3)*p(11)*...
         cos(q(3))*sin(q(2))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + p(3)*p(11)*cos(q(4))*sin(q(2))*sin(q(3))*(p(6)^2 +...
          p(7)^2)^(1/2) + p(11)*p(21)*cos(q(3))*sin(q(2))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + p(11)*p(21)*cos(q(4))*...
         sin(q(2))*sin(q(3))*(p(6)^2 + p(7)^2)^(1/2) - p(11)*p(22)*p(23)*cos(2*q(3))*cos(2*q(4))*cos(q(2)) - p(4)*...
         p(11)*p(23)*cos(q(2))*cos(q(3))*cos(q(4)) + p(11)*p(22)*p(23)*sin(2*q(3))*sin(2*q(4))*cos(q(2)) - p(4)*...
         p(11)*p(22)*cos(q(2))*cos(q(3))*sin(q(4)) - p(4)*p(11)*p(22)*cos(q(2))*cos(q(4))*sin(q(3)) + p(5)*p(11)*...
         cos(2*q(3))*cos(q(2))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + p(5)*p(11)*sin(2*q(3))*cos(q(2))*cos(q(4))*...
         (p(6)^2 + p(7)^2)^(1/2) + p(3)*p(11)*p(22)*cos(q(3))*cos(q(4))*sin(q(2));
  De(1,3)=p(34)*cos(q(2))*cos(q(3)) - p(36)*sin(q(2)) - p(5)^2*p(10)*sin(q(2)) - p(5)^2*p(11)*...
         sin(q(2)) - p(11)*sin(q(2))*(p(6)^2 + p(7)^2) - p(30)*sin(q(2)) + p(33)*cos(q(2))*sin(q(3)) - p(10)*p(19)^2*...
         sin(q(2)) - p(11)*p(22)^2*sin(q(2)) - p(10)*p(20)^2*sin(q(2)) - p(11)*p(23)^2*sin(q(2)) + 2*p(11)*p(23)*sin(q(2))*...
         (p(6)^2 + p(7)^2)^(1/2) + p(40)*cos(q(2))*cos(q(3))*cos(q(4)) + p(39)*cos(q(2))*cos(q(3))*sin(q(4)) + p(39)*...
         cos(q(2))*cos(q(4))*sin(q(3)) - p(40)*cos(q(2))*sin(q(3))*sin(q(4)) + 2*p(5)*p(10)*p(20)*sin(q(2)) - p(4)*...
         p(5)*p(10)*sin(q(2))*sin(q(3)) - p(4)*p(5)*p(11)*sin(q(2))*sin(q(3)) - p(3)*p(10)*p(20)*cos(q(2))*...
         cos(q(3)) + p(5)*p(10)*p(18)*cos(q(2))*cos(q(3)) + p(5)*p(11)*p(21)*cos(q(2))*cos(q(3)) - p(4)*p(10)*p(19)*...
         cos(q(3))*sin(q(2)) - p(3)*p(10)*p(19)*cos(q(2))*sin(q(3)) + 2*p(5)*p(11)*p(23)*cos(q(4))*sin(q(2)) + p(4)*...
         p(10)*p(20)*sin(q(2))*sin(q(3)) + 2*p(5)*p(11)*p(22)*sin(q(2))*sin(q(4)) - p(10)*p(18)*p(20)*cos(q(2))*...
         cos(q(3)) - p(10)*p(18)*p(19)*cos(q(2))*sin(q(3)) - 2*p(5)*p(11)*cos(q(4))*sin(q(2))*(p(6)^2 + p(7)^2)^(1/2) + p(3)*...
         p(5)*p(10)*cos(q(2))*cos(q(3)) + p(3)*p(5)*p(11)*cos(q(2))*cos(q(3)) + p(4)*p(11)*p(23)*cos(q(3))*...
         sin(q(2))*sin(q(4)) + p(4)*p(11)*p(23)*cos(q(4))*sin(q(2))*sin(q(3)) + p(3)*p(11)*p(23)*cos(q(2))*sin(q(3))*...
         sin(q(4)) - p(11)*p(21)*p(23)*cos(q(2))*cos(q(3))*cos(q(4)) + p(4)*p(11)*p(22)*sin(q(2))*sin(q(3))*sin(q(4)) - p(11)*...
         p(21)*p(22)*cos(q(2))*cos(q(3))*sin(q(4)) - p(11)*p(21)*p(22)*cos(q(2))*cos(q(4))*sin(q(3)) + p(11)*p(21)*...
         p(23)*cos(q(2))*sin(q(3))*sin(q(4)) + p(3)*p(11)*cos(q(2))*cos(q(3))*cos(q(4))*(p(6)^2 +...
          p(7)^2)^(1/2) - p(4)*p(11)*cos(q(3))*sin(q(2))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(4)*p(11)*cos(q(4))*sin(q(2))*...
         sin(q(3))*(p(6)^2 + p(7)^2)^(1/2) - p(3)*p(11)*cos(q(2))*sin(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + p(11)*...
         p(21)*cos(q(2))*cos(q(3))*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(11)*p(21)*cos(q(2))*sin(q(3))*sin(q(4))*...
         (p(6)^2 + p(7)^2)^(1/2) - p(3)*p(11)*p(23)*cos(q(2))*cos(q(3))*cos(q(4)) - p(4)*p(11)*p(22)*cos(q(3))*...
         cos(q(4))*sin(q(2)) - p(3)*p(11)*p(22)*cos(q(2))*cos(q(3))*sin(q(4)) - p(3)*p(11)*p(22)*cos(q(2))*cos(q(4))*sin(q(3));
  De(1,4)=2*p(11)*p(23)*sin(q(2))*(p(6)^2 + p(7)^2)^(1/2) - p(11)*sin(q(2))*(p(6)^2 +...
          p(7)^2) - p(11)*p(22)^2*sin(q(2)) - p(11)*p(23)^2*sin(q(2)) - p(36)*sin(q(2)) + p(40)*cos(q(2))*cos(q(3))*...
         cos(q(4)) + p(39)*cos(q(2))*cos(q(3))*sin(q(4)) + p(39)*cos(q(2))*cos(q(4))*sin(q(3)) - p(40)*cos(q(2))*...
         sin(q(3))*sin(q(4)) + p(5)*p(11)*p(23)*cos(q(4))*sin(q(2)) + p(5)*p(11)*p(22)*sin(q(2))*sin(q(4)) - p(5)*...
         p(11)*cos(q(4))*sin(q(2))*(p(6)^2 + p(7)^2)^(1/2) + p(4)*p(11)*p(23)*cos(q(3))*sin(q(2))*sin(q(4)) + p(4)*...
         p(11)*p(23)*cos(q(4))*sin(q(2))*sin(q(3)) + p(3)*p(11)*p(23)*cos(q(2))*sin(q(3))*sin(q(4)) - p(11)*p(21)*...
         p(23)*cos(q(2))*cos(q(3))*cos(q(4)) + p(4)*p(11)*p(22)*sin(q(2))*sin(q(3))*sin(q(4)) - p(11)*p(21)*p(22)*...
         cos(q(2))*cos(q(3))*sin(q(4)) - p(11)*p(21)*p(22)*cos(q(2))*cos(q(4))*sin(q(3)) + p(11)*p(21)*p(23)*cos(q(2))*...
         sin(q(3))*sin(q(4)) + p(3)*p(11)*cos(q(2))*cos(q(3))*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(4)*p(11)*cos(q(3))*...
         sin(q(2))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(4)*p(11)*cos(q(4))*sin(q(2))*sin(q(3))*(p(6)^2 +...
          p(7)^2)^(1/2) - p(3)*p(11)*cos(q(2))*sin(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + p(11)*p(21)*cos(q(2))*cos(q(3))*...
         cos(q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(11)*p(21)*cos(q(2))*sin(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(3)*...
         p(11)*p(23)*cos(q(2))*cos(q(3))*cos(q(4)) - p(4)*p(11)*p(22)*cos(q(3))*cos(q(4))*sin(q(2)) - p(3)*p(11)*...
         p(22)*cos(q(2))*cos(q(3))*sin(q(4)) - p(3)*p(11)*p(22)*cos(q(2))*cos(q(4))*sin(q(3));
  De(2,1)=p(29)*cos(q(2)) - p(27)*sin(q(2)) - p(33)*cos(q(3))*sin(q(2)) + p(34)*sin(q(2))*...
         sin(q(3)) + p(35)*cos(2*q(3))*cos(q(2)) + (p(31)*sin(2*q(3))*cos(q(2)))/2 - (p(32)*sin(2*q(3))*...
         cos(q(2)))/2 - p(41)*sin(2*q(3))*sin(2*q(4))*cos(q(2)) + p(4)*p(3)*p(9)*sin(q(2)) + p(4)*p(3)*p(10)*sin(q(2)) + p(4)*...
         p(3)*p(11)*sin(q(2)) - p(39)*cos(q(3))*cos(q(4))*sin(q(2)) - p(4)*p(9)*p(17)*cos(q(2)) + p(40)*cos(q(3))*...
         sin(q(2))*sin(q(4)) + p(40)*cos(q(4))*sin(q(2))*sin(q(3)) + p(4)*p(9)*p(15)*sin(q(2)) + p(4)*p(10)*p(18)*...
         sin(q(2)) + p(4)*p(11)*p(21)*sin(q(2)) + p(3)*p(9)*p(16)*sin(q(2)) + p(39)*sin(q(2))*sin(q(3))*...
         sin(q(4)) - p(9)*p(16)*p(17)*cos(q(2)) + (p(5)^2*p(10)*sin(2*q(3))*cos(q(2)))/2 + (p(5)^2*p(11)*sin(2*q(3))*...
         cos(q(2)))/2 + p(9)*p(15)*p(16)*sin(q(2)) - (p(10)*p(19)^2*sin(2*q(3))*cos(q(2)))/2 + (p(10)*p(20)^2*sin(2*q(3))*...
         cos(q(2)))/2 + p(41)*cos(2*q(3))*cos(2*q(4))*cos(q(2)) + (p(37)*cos(2*q(3))*sin(2*q(4))*cos(q(2)))/2 + (p(37)*...
         cos(2*q(4))*sin(2*q(3))*cos(q(2)))/2 - (p(38)*cos(2*q(3))*sin(2*q(4))*cos(q(2)))/2 - (p(38)*cos(2*q(4))*...
         sin(2*q(3))*cos(q(2)))/2 - p(4)*p(10)*p(20)*cos(q(2))*cos(q(3)) + p(3)*p(5)*p(10)*sin(q(2))*sin(q(3)) +...
          p(3)*p(5)*p(11)*sin(q(2))*sin(q(3)) - p(4)*p(10)*p(19)*cos(q(2))*sin(q(3)) + p(3)*p(10)*p(19)*cos(q(3))*...
         sin(q(2)) - p(3)*p(10)*p(20)*sin(q(2))*sin(q(3)) + p(5)*p(10)*p(18)*sin(q(2))*sin(q(3)) + p(5)*p(11)*p(21)*sin(q(2))*...
         sin(q(3)) + p(10)*p(18)*p(19)*cos(q(3))*sin(q(2)) - p(10)*p(18)*p(20)*sin(q(2))*sin(q(3)) + p(5)*p(10)*p(19)*...
         cos(2*q(3))*cos(q(2)) - p(5)*p(10)*p(20)*sin(2*q(3))*cos(q(2)) - p(10)*p(19)*p(20)*cos(2*q(3))*cos(q(2)) +...
          (p(11)*cos(2*q(3))*sin(2*q(4))*cos(q(2))*(p(6)^2 + p(7)^2))/2 + (p(11)*cos(2*q(4))*sin(2*q(3))*cos(q(2))*...
         (p(6)^2 + p(7)^2))/2 - (p(11)*p(22)^2*cos(2*q(3))*sin(2*q(4))*cos(q(2)))/2 - (p(11)*p(22)^2*cos(2*q(4))*...
         sin(2*q(3))*cos(q(2)))/2 + (p(11)*p(23)^2*cos(2*q(3))*sin(2*q(4))*cos(q(2)))/2 + (p(11)*p(23)^2*cos(2*...
         q(4))*sin(2*q(3))*cos(q(2)))/2 + p(4)*p(5)*p(10)*cos(q(2))*cos(q(3)) + p(4)*p(5)*p(11)*cos(q(2))*...
         cos(q(3)) + p(4)*p(11)*p(23)*cos(q(2))*sin(q(3))*sin(q(4)) - p(3)*p(11)*p(23)*cos(q(3))*sin(q(2))*...
         sin(q(4)) - p(3)*p(11)*p(23)*cos(q(4))*sin(q(2))*sin(q(3)) - p(3)*p(11)*p(22)*sin(q(2))*sin(q(3))*sin(q(4)) + p(11)*...
         p(21)*p(22)*cos(q(3))*cos(q(4))*sin(q(2)) - p(11)*p(21)*p(23)*cos(q(3))*sin(q(2))*sin(q(4)) - p(11)*p(21)*...
         p(23)*cos(q(4))*sin(q(2))*sin(q(3)) - p(11)*p(21)*p(22)*sin(q(2))*sin(q(3))*sin(q(4)) + p(5)*p(11)*p(22)*...
         cos(2*q(3))*cos(q(2))*cos(q(4)) - p(5)*p(11)*p(23)*cos(2*q(3))*cos(q(2))*sin(q(4)) - p(5)*p(11)*p(23)*...
         sin(2*q(3))*cos(q(2))*cos(q(4)) - p(5)*p(11)*p(22)*sin(2*q(3))*cos(q(2))*sin(q(4)) + p(11)*p(22)*cos(2*...
         q(3))*cos(2*q(4))*cos(q(2))*(p(6)^2 + p(7)^2)^(1/2) - p(11)*p(23)*cos(2*q(3))*sin(2*q(4))*cos(q(2))*...
         (p(6)^2 + p(7)^2)^(1/2) - p(11)*p(23)*cos(2*q(4))*sin(2*q(3))*cos(q(2))*(p(6)^2 + p(7)^2)^(1/2) + p(4)*p(11)*...
         cos(q(2))*cos(q(3))*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(11)*p(22)*sin(2*q(3))*sin(2*q(4))*cos(q(2))*...
         (p(6)^2 + p(7)^2)^(1/2) - p(4)*p(11)*cos(q(2))*sin(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + p(3)*p(11)*...
         cos(q(3))*sin(q(2))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + p(3)*p(11)*cos(q(4))*sin(q(2))*sin(q(3))*(p(6)^2 +...
          p(7)^2)^(1/2) + p(11)*p(21)*cos(q(3))*sin(q(2))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + p(11)*p(21)*cos(q(4))*...
         sin(q(2))*sin(q(3))*(p(6)^2 + p(7)^2)^(1/2) - p(11)*p(22)*p(23)*cos(2*q(3))*cos(2*q(4))*cos(q(2)) - p(4)*...
         p(11)*p(23)*cos(q(2))*cos(q(3))*cos(q(4)) + p(11)*p(22)*p(23)*sin(2*q(3))*sin(2*q(4))*cos(q(2)) - p(4)*...
         p(11)*p(22)*cos(q(2))*cos(q(3))*sin(q(4)) - p(4)*p(11)*p(22)*cos(q(2))*cos(q(4))*sin(q(3)) + p(5)*p(11)*...
         cos(2*q(3))*cos(q(2))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + p(5)*p(11)*sin(2*q(3))*cos(q(2))*cos(q(4))*...
         (p(6)^2 + p(7)^2)^(1/2) + p(3)*p(11)*p(22)*cos(q(3))*cos(q(4))*sin(q(2));
  De(2,2)=p(25) + p(31)/2 + p(37)/2 + p(32)/2 + p(38)/2 + (p(37)*cos(2*q(3) + 2*q(4)))/2 - (p(38)*...
         cos(2*q(3) + 2*q(4)))/2 - p(41)*sin(2*q(3) + 2*q(4)) + p(3)^2*p(9) + p(3)^2*p(10) + p(3)^2*p(11) +...
          (p(5)^2*p(10))/2 + (p(5)^2*p(11))/2 + (p(11)*(p(6)^2 + p(7)^2))/2 + p(9)*p(15)^2 + p(10)*p(18)^2 + p(11)*...
         p(21)^2 + (p(10)*p(19)^2)/2 + (p(11)*p(22)^2)/2 + p(9)*p(17)^2 + (p(10)*p(20)^2)/2 + (p(11)*p(23)^2)/2 +...
          (p(31)*cos(2*q(3)))/2 - (p(32)*cos(2*q(3)))/2 - p(35)*sin(2*q(3)) - p(11)*p(23)*(p(6)^2 + p(7)^2)^(1/2) +...
          2*p(3)*p(9)*p(15) + 2*p(3)*p(10)*p(18) + 2*p(3)*p(11)*p(21) - p(5)*p(10)*p(20) + (p(5)^2*p(10)*cos(2*...
         q(3)))/2 + (p(5)^2*p(11)*cos(2*q(3)))/2 - (p(10)*p(19)^2*cos(2*q(3)))/2 + (p(10)*p(20)^2*cos(2*q(3)))/2 +...
          (p(11)*cos(2*q(3) + 2*q(4))*(p(6)^2 + p(7)^2))/2 - (p(11)*p(22)^2*cos(2*q(3) + 2*q(4)))/2 + (p(11)*p(23)^2*...
         cos(2*q(3) + 2*q(4)))/2 + p(11)*p(22)*p(23)*sin(2*q(3) + 2*q(4)) + p(5)*p(11)*cos(2*q(3) + q(4))*(p(6)^2 +...
          p(7)^2)^(1/2) - p(5)*p(11)*p(23)*cos(q(4)) - p(5)*p(11)*p(22)*sin(q(4)) - p(5)*p(11)*p(23)*cos(2*q(3) + q(4)) - p(5)*...
         p(11)*p(22)*sin(2*q(3) + q(4)) - p(11)*p(23)*cos(2*q(3) + 2*q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(5)*p(10)*...
         p(20)*cos(2*q(3)) - p(11)*p(22)*sin(2*q(3) + 2*q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(5)*p(10)*p(19)*sin(2*...
         q(3)) + p(10)*p(19)*p(20)*sin(2*q(3)) + p(5)*p(11)*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2);
  De(2,3)=p(39)*cos(q(3) + q(4)) - p(40)*sin(q(3) + q(4)) + p(33)*cos(q(3)) - p(34)*...
         sin(q(3)) - p(3)*p(11)*p(22)*cos(q(3) + q(4)) + p(3)*p(11)*p(23)*sin(q(3) + q(4)) - p(3)*p(5)*p(10)*sin(q(3)) - p(3)*...
         p(5)*p(11)*sin(q(3)) - p(11)*p(21)*p(22)*cos(q(3) + q(4)) - p(3)*p(10)*p(19)*cos(q(3)) + p(11)*p(21)*...
         p(23)*sin(q(3) + q(4)) + p(3)*p(10)*p(20)*sin(q(3)) - p(5)*p(10)*p(18)*sin(q(3)) - p(5)*p(11)*p(21)*...
         sin(q(3)) - p(10)*p(18)*p(19)*cos(q(3)) + p(10)*p(18)*p(20)*sin(q(3)) - p(3)*p(11)*sin(q(3) + q(4))*(p(6)^2 +...
          p(7)^2)^(1/2) - p(11)*p(21)*sin(q(3) + q(4))*(p(6)^2 + p(7)^2)^(1/2);
  De(2,4)=p(39)*cos(q(3) + q(4)) - p(40)*sin(q(3) + q(4)) - p(3)*p(11)*p(22)*cos(q(3) + q(4)) +...
          p(3)*p(11)*p(23)*sin(q(3) + q(4)) - p(11)*p(21)*p(22)*cos(q(3) + q(4)) + p(11)*p(21)*p(23)*sin(q(3) +...
          q(4)) - p(3)*p(11)*sin(q(3) + q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(11)*p(21)*sin(q(3) + q(4))*(p(6)^2 + p(7)^2)^(1/2);
  De(3,1)=p(34)*cos(q(2))*cos(q(3)) - p(36)*sin(q(2)) - p(5)^2*p(10)*sin(q(2)) - p(5)^2*p(11)*...
         sin(q(2)) - p(11)*sin(q(2))*(p(6)^2 + p(7)^2) - p(30)*sin(q(2)) + p(33)*cos(q(2))*sin(q(3)) - p(10)*p(19)^2*...
         sin(q(2)) - p(11)*p(22)^2*sin(q(2)) - p(10)*p(20)^2*sin(q(2)) - p(11)*p(23)^2*sin(q(2)) + 2*p(11)*p(23)*sin(q(2))*...
         (p(6)^2 + p(7)^2)^(1/2) + p(40)*cos(q(2))*cos(q(3))*cos(q(4)) + p(39)*cos(q(2))*cos(q(3))*sin(q(4)) + p(39)*...
         cos(q(2))*cos(q(4))*sin(q(3)) - p(40)*cos(q(2))*sin(q(3))*sin(q(4)) + 2*p(5)*p(10)*p(20)*sin(q(2)) - p(4)*...
         p(5)*p(10)*sin(q(2))*sin(q(3)) - p(4)*p(5)*p(11)*sin(q(2))*sin(q(3)) - p(3)*p(10)*p(20)*cos(q(2))*...
         cos(q(3)) + p(5)*p(10)*p(18)*cos(q(2))*cos(q(3)) + p(5)*p(11)*p(21)*cos(q(2))*cos(q(3)) - p(4)*p(10)*p(19)*...
         cos(q(3))*sin(q(2)) - p(3)*p(10)*p(19)*cos(q(2))*sin(q(3)) + 2*p(5)*p(11)*p(23)*cos(q(4))*sin(q(2)) + p(4)*...
         p(10)*p(20)*sin(q(2))*sin(q(3)) + 2*p(5)*p(11)*p(22)*sin(q(2))*sin(q(4)) - p(10)*p(18)*p(20)*cos(q(2))*...
         cos(q(3)) - p(10)*p(18)*p(19)*cos(q(2))*sin(q(3)) - 2*p(5)*p(11)*cos(q(4))*sin(q(2))*(p(6)^2 + p(7)^2)^(1/2) + p(3)*...
         p(5)*p(10)*cos(q(2))*cos(q(3)) + p(3)*p(5)*p(11)*cos(q(2))*cos(q(3)) + p(4)*p(11)*p(23)*cos(q(3))*...
         sin(q(2))*sin(q(4)) + p(4)*p(11)*p(23)*cos(q(4))*sin(q(2))*sin(q(3)) + p(3)*p(11)*p(23)*cos(q(2))*sin(q(3))*...
         sin(q(4)) - p(11)*p(21)*p(23)*cos(q(2))*cos(q(3))*cos(q(4)) + p(4)*p(11)*p(22)*sin(q(2))*sin(q(3))*sin(q(4)) - p(11)*...
         p(21)*p(22)*cos(q(2))*cos(q(3))*sin(q(4)) - p(11)*p(21)*p(22)*cos(q(2))*cos(q(4))*sin(q(3)) + p(11)*p(21)*...
         p(23)*cos(q(2))*sin(q(3))*sin(q(4)) + p(3)*p(11)*cos(q(2))*cos(q(3))*cos(q(4))*(p(6)^2 +...
          p(7)^2)^(1/2) - p(4)*p(11)*cos(q(3))*sin(q(2))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(4)*p(11)*cos(q(4))*sin(q(2))*...
         sin(q(3))*(p(6)^2 + p(7)^2)^(1/2) - p(3)*p(11)*cos(q(2))*sin(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + p(11)*...
         p(21)*cos(q(2))*cos(q(3))*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(11)*p(21)*cos(q(2))*sin(q(3))*sin(q(4))*...
         (p(6)^2 + p(7)^2)^(1/2) - p(3)*p(11)*p(23)*cos(q(2))*cos(q(3))*cos(q(4)) - p(4)*p(11)*p(22)*cos(q(3))*...
         cos(q(4))*sin(q(2)) - p(3)*p(11)*p(22)*cos(q(2))*cos(q(3))*sin(q(4)) - p(3)*p(11)*p(22)*cos(q(2))*cos(q(4))*sin(q(3));
  De(3,2)=p(39)*cos(q(3) + q(4)) - p(40)*sin(q(3) + q(4)) + p(33)*cos(q(3)) - p(34)*...
         sin(q(3)) - p(3)*p(11)*p(22)*cos(q(3) + q(4)) + p(3)*p(11)*p(23)*sin(q(3) + q(4)) - p(3)*p(5)*p(10)*sin(q(3)) - p(3)*...
         p(5)*p(11)*sin(q(3)) - p(11)*p(21)*p(22)*cos(q(3) + q(4)) - p(3)*p(10)*p(19)*cos(q(3)) + p(11)*p(21)*...
         p(23)*sin(q(3) + q(4)) + p(3)*p(10)*p(20)*sin(q(3)) - p(5)*p(10)*p(18)*sin(q(3)) - p(5)*p(11)*p(21)*...
         sin(q(3)) - p(10)*p(18)*p(19)*cos(q(3)) + p(10)*p(18)*p(20)*sin(q(3)) - p(3)*p(11)*sin(q(3) + q(4))*(p(6)^2 +...
          p(7)^2)^(1/2) - p(11)*p(21)*sin(q(3) + q(4))*(p(6)^2 + p(7)^2)^(1/2);
  De(3,3)=p(30) + p(36) + p(5)^2*p(10) + p(5)^2*p(11) + p(11)*(p(6)^2 + p(7)^2) + p(10)*p(19)^2 +...
          p(11)*p(22)^2 + p(10)*p(20)^2 + p(11)*p(23)^2 - 2*p(11)*p(23)*(p(6)^2 + p(7)^2)^(1/2) - 2*p(5)*p(10)*...
         p(20) - 2*p(5)*p(11)*p(23)*cos(q(4)) - 2*p(5)*p(11)*p(22)*sin(q(4)) + 2*p(5)*p(11)*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2);
  De(3,4)=p(36) + p(11)*(p(6)^2 + p(7)^2) + p(11)*p(22)^2 + p(11)*p(23)^2 - 2*p(11)*p(23)*(p(6)^2 +...
          p(7)^2)^(1/2) - p(5)*p(11)*p(23)*cos(q(4)) - p(5)*p(11)*p(22)*sin(q(4)) + p(5)*p(11)*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2);
  De(4,1)=2*p(11)*p(23)*sin(q(2))*(p(6)^2 + p(7)^2)^(1/2) - p(11)*sin(q(2))*(p(6)^2 +...
          p(7)^2) - p(11)*p(22)^2*sin(q(2)) - p(11)*p(23)^2*sin(q(2)) - p(36)*sin(q(2)) + p(40)*cos(q(2))*cos(q(3))*...
         cos(q(4)) + p(39)*cos(q(2))*cos(q(3))*sin(q(4)) + p(39)*cos(q(2))*cos(q(4))*sin(q(3)) - p(40)*cos(q(2))*...
         sin(q(3))*sin(q(4)) + p(5)*p(11)*p(23)*cos(q(4))*sin(q(2)) + p(5)*p(11)*p(22)*sin(q(2))*sin(q(4)) - p(5)*...
         p(11)*cos(q(4))*sin(q(2))*(p(6)^2 + p(7)^2)^(1/2) + p(4)*p(11)*p(23)*cos(q(3))*sin(q(2))*sin(q(4)) + p(4)*...
         p(11)*p(23)*cos(q(4))*sin(q(2))*sin(q(3)) + p(3)*p(11)*p(23)*cos(q(2))*sin(q(3))*sin(q(4)) - p(11)*p(21)*...
         p(23)*cos(q(2))*cos(q(3))*cos(q(4)) + p(4)*p(11)*p(22)*sin(q(2))*sin(q(3))*sin(q(4)) - p(11)*p(21)*p(22)*...
         cos(q(2))*cos(q(3))*sin(q(4)) - p(11)*p(21)*p(22)*cos(q(2))*cos(q(4))*sin(q(3)) + p(11)*p(21)*p(23)*cos(q(2))*...
         sin(q(3))*sin(q(4)) + p(3)*p(11)*cos(q(2))*cos(q(3))*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(4)*p(11)*cos(q(3))*...
         sin(q(2))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(4)*p(11)*cos(q(4))*sin(q(2))*sin(q(3))*(p(6)^2 +...
          p(7)^2)^(1/2) - p(3)*p(11)*cos(q(2))*sin(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + p(11)*p(21)*cos(q(2))*cos(q(3))*...
         cos(q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(11)*p(21)*cos(q(2))*sin(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(3)*...
         p(11)*p(23)*cos(q(2))*cos(q(3))*cos(q(4)) - p(4)*p(11)*p(22)*cos(q(3))*cos(q(4))*sin(q(2)) - p(3)*p(11)*...
         p(22)*cos(q(2))*cos(q(3))*sin(q(4)) - p(3)*p(11)*p(22)*cos(q(2))*cos(q(4))*sin(q(3));
  De(4,2)=p(39)*cos(q(3) + q(4)) - p(40)*sin(q(3) + q(4)) - p(3)*p(11)*p(22)*cos(q(3) + q(4)) +...
          p(3)*p(11)*p(23)*sin(q(3) + q(4)) - p(11)*p(21)*p(22)*cos(q(3) + q(4)) + p(11)*p(21)*p(23)*sin(q(3) +...
          q(4)) - p(3)*p(11)*sin(q(3) + q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(11)*p(21)*sin(q(3) + q(4))*(p(6)^2 + p(7)^2)^(1/2);
  De(4,3)=p(36) + p(11)*(p(6)^2 + p(7)^2) + p(11)*p(22)^2 + p(11)*p(23)^2 - 2*p(11)*p(23)*(p(6)^2 +...
          p(7)^2)^(1/2) - p(5)*p(11)*p(23)*cos(q(4)) - p(5)*p(11)*p(22)*sin(q(4)) + p(5)*p(11)*cos(q(4))*(p(6)^2 + p(7)^2)^(1/2);
  De(4,4)=p(36) + p(11)*(p(6)^2 + p(7)^2) + p(11)*p(22)^2 + p(11)*p(23)^2 - 2*p(11)*p(23)*(p(6)^2 + p(7)^2)^(1/2);

 