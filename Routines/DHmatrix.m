function DH = DHmatrix(theta, d, a, alpha)

%%output: DH matrix
%%input: DH parameters:
%%theta: rotation along x(n) axis
%%d: traslation along z(n) axis
%%a: translation along x(n+1) axis
%%alpha: rotation along x(n+1) axis
%%all angles expressed in degrees

DH = [cos(theta)   -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)   a*cos(theta);
      sin(theta)   cos(theta)*cos(alpha)   -cos(theta)*sin(alpha)  a*sin(theta);
      0            sin(alpha)              cos(alpha)              d;
      0            0                       0                       1];


end