function [A,B,xe,ue,Ad,Bd] = serial_leg_state_space(p,phi_e,Ts)
% Fixed-shape eccentric leg, SI units, state [theta;dtheta;x;dx;phi;dphi].
% Required p fields: R,Lt,L,y,l,mw,mp,M,Iw,Ip,IM,g.
% M and IM must use the SAME single-side/full-robot convention as mp,mw.
% Ip = ONE leg's pitch-axis inertia ABOUT THAT LEG'S OWN CENTER OF MASS.
% Enter Ip directly: do not divide by two or add mp*(L^2+y^2) beforehand.
% This function adds the parallel-axis term once in H(1,1).
% No provisional physical parameters are silently supplied by this function.
% phi_e defaults to zero. Optional Ts computes exact zero-order-hold matrices.
if nargin < 2, phi_e = 0; end
if nargin < 3, Ts = []; end
names = {'R','Lt','L','y','l','mw','mp','M','Iw','Ip','IM','g'};
for i=1:numel(names)
    validateattributes(p.(names{i}),{'numeric'},{'real','finite','scalar'});
end
assert(all([p.R,p.Lt,p.mw,p.mp,p.M,p.Iw,p.Ip,p.IM,p.g] > 0));
c = p.mp*p.L+p.M*p.Lt;
d = p.mp*p.y;
arg = p.M*p.l*sin(phi_e)/hypot(c,d);
assert(abs(arg)<=1,'Requested pitch has no static equilibrium.');
t = asin(arg)-atan2(d,c);
xe = [t;0;0;0;phi_e;0];
ue = [0;-p.M*p.g*p.l*sin(phi_e)];
b = p.L*cos(t)-p.y*sin(t);
H = [p.Ip+p.mp*(p.L^2+p.y^2)+p.M*p.Lt^2, p.mp*b+p.M*p.Lt*cos(t), -p.M*p.Lt*p.l*cos(t+phi_e);
     p.mp*b+p.M*p.Lt*cos(t), p.mw+p.Iw/p.R^2+p.mp+p.M, -p.M*p.l*cos(phi_e);
     -p.M*p.Lt*p.l*cos(t+phi_e), -p.M*p.l*cos(phi_e), p.IM+p.M*p.l^2];
S = diag([p.g*(p.mp*b+p.M*p.Lt*cos(t)),0,p.M*p.g*p.l*cos(phi_e)]);
D = [-1,1;1/p.R,0;0,1];
[~,flag] = chol(H);
assert(flag==0,'Mass matrix must be positive definite.');
A = zeros(6); B = zeros(6,2);
A(1,2)=1; A(3,4)=1; A(5,6)=1;
A([2,4,6],[1,3,5]) = H\S;
B([2,4,6],:) = H\D;
Ad=[]; Bd=[];
if ~isempty(Ts)
    validateattributes(Ts,{'numeric'},{'real','finite','scalar','positive'});
    E = expm([A,B;zeros(2,8)]*Ts);
    Ad=E(1:6,1:6); Bd=E(1:6,7:8);
end
end
