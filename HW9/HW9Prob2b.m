A=[0 1; 1000 0];
B=[0; -20];
C=[1 0];
D=[0]
% rdot = 0 Constant commands
% Build up the wiggle system
Aw = [ 0 C
0.*ones(2,1) A];
Bw = [ 0 B']';
R = 10;
Q=0.*Aw;
Q(1,1)=10.;
%[K,S,E]=lqr(Aw,Bw,Q,R);
%Closed loop system is now
% xdot = (Aw-Bw*K)x + Fr
% ycl = x(2)
K=[-375 -108.75 -3];     
F=[-1 0 0]';
Ccl=[0 1 0 ];
Dcl = 0.*Ccl*F;
%simulate system
t=[0.:.01:10.]';
r=ones(size(t));
[ycl,xcl] = lsim(Aw-Bw*K,F,Ccl,Dcl,10*r,t);
figure;plot(t,xcl);
xlabel('time');title('States vs. Time');legend('Control', 'Angle of attack', 'q');