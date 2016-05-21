%Calculate frequency response at plant input
A = [0 1; 1000 0];
B = [0; -20];
C = [1 0];
Kc = [-144.49 -3];
L = [160; 13800];
% Ac = A-B*Kc-Ko*C;
Ac = A-B*Kc-L*C;
Bc= L;
Cc = Kc;
w=logspace(-1,3,100);
[mag_sf,phase_sf]=bode(A,B,Kc,0.,1,w);
[re_sf,im_sf]=nyquist(A,B,Kc,0.,1,w);

%Form Observer Feedback system
Ao= [A 0.*B*Cc;Bc*C Ac];Bo = [B; 0.*Bc];
Co = [0.*C Cc];
Do = 0.*Co*Bo;

[mag_of,phase_of]=bode(Ao,Bo,Co,Do,1,w);
[re_of,im_of]=nyquist(Ao,Bo,Co,Do,1,w);

figure;semilogx(w,20*log10(mag_sf),w,20*log10(mag_of));
grid;xlabel('Frequency (rps)');ylabel('Magnitude (dB)');
title('Frequency Response At Input');
pause
% clg
figure;semilogx(w,phase_sf,w,phase_of);
grid;xlabel('Frequency (rps)');ylabel('Phase (deg)');
title('Frequency Response At Input');
pause
% clg

% % % 


ax = 3;
axis([-ax,ax,-ax,ax]);
figure;plot(re_sf,im_sf,re_of,im_of);grid;
xlabel('Real');ylabel('Imaginary');
title('Frequency Response At Input');
axis;
pause
% clg

[Gm,Pm,Wcg,Wcp] = margin(mag_sf,phase_sf,w)
figure;margin(mag_sf,phase_sf,w);
title('State Feedback');
pause
% clg
[Gm,Pm,Wcg,Wcp] = margin(mag_of,phase_of,w)
figure;margin(mag_of,phase_of,w)
title('Observer Feedback');
pause
% clg

[re_g,im_g]=nyquist(A,B,C,0.,1,w);%Plant Only
[re_c,im_c]=nyquist(Ac,Bc,Cc,0.,1,w);%Controller Only
freq_g = re_g + sqrt(-1)*im_g;
freq_c = re_c + sqrt(-1)*im_c;
G = freq_c.*freq_g;

axis([-ax,ax,-ax,ax]);
figure;plot(real(G),imag(G),re_of,im_of);grid;
xlabel('Real');ylabel('Imaginary');
title('Frequency Response At Input (Two Methods)');
axis;
pause
% clg


for i=1:numel(w),
    s = sqrt(-1)*w(i);
    GG = C*inv(s*eye(size(A))-A)*B;
    KK = Cc*inv(s*eye(size(Ac))-Ac)*Bc;
    Lu_lqr(i)  = -KK*GG;
    RDu_lqr(i)  = 1. + Lu_lqr(i);
    SRu_lqr(i) = 1. + 1./Lu_lqr(i);
end

figure('Name','Return Difference at Plant Input'),
semilogx(w,20*log10(abs(RDu_lqr)),'r--','LineWidth',2);grid
legend('I+Lu','Location','Best');
xlabel('Frequency (rps)')
ylabel('Mag dB')
title('Return Difference at Plant Input')


figure('Name','Stability Robustness at Plant Input'),
semilogx(w,20*log10(abs(SRu_lqr)),'r--','LineWidth',2);grid
legend('I+invLu','Location','Best');
xlabel('Frequency (rps)')
ylabel('Mag dB')
title('Stability Robustness at Plant Input')
