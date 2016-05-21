%Calculate frequency response at plant input
A = [0 1; 1000 0];
B = [0; -20];
C = [1 0];
Kc = [-144.49 -3];
L = [160; 13800];
Ac = A-B*Kc-L*C;
Bc= L;
Cc = Kc;
w=logspace(-1,3,100);
[mag_sf,phase_sf]=bode(A,B,Kc,0.,1,w);
[re_sf,im_sf]=nyquist(A,B,Kc,0.,1,w);
%Form Observer Feedback system
Ao= [A 0.*B*Cc;Bc*C Ac];
Bo = [B; 0.*Bc];
Co = [0.*C Cc];
Do = 0.*Co*Bo;
[mag_of,phase_of]=bode(Ao,Bo,Co,Do,1,w);
[re_of,im_of]=nyquist(Ao,Bo,Co,Do,1,w);
figure(1)
semilogx(w,20*log10(mag_sf),w,20*log10(mag_of));
grid;xlabel('Frequency (rps)');ylabel('Magnitude (dB)');
title('Frequency Response At Input'); 
pause
% clg
figure(2)
semilogx(w,phase_sf,w,phase_of);
grid;xlabel('Frequency (rps)');ylabel('Phase (deg)');
title('Frequency Response At Input');
pause
% clg
ax = 3;
figure(3)
axis([-ax,ax,-ax,ax]);
plot(re_sf,im_sf,re_of,im_of);grid;
xlabel('Real');ylabel('Imaginary');
title('Frequency Response At Input');
axis;
pause
% clg
figure(4)
[Gm,Pm,Wcg,Wcp] = margin(mag_sf,phase_sf,w)
margin(mag_sf,phase_sf,w);
title('State Feedback');
pause
% clg
figure(5)
[Gm,Pm,Wcg,Wcp] = margin(mag_of,phase_of,w)
margin(mag_of,phase_of,w)
title('Observer Feedback');
pause
% clg
[re_g,im_g]=nyquist(A,B,C,0.,1,w);%Plant Only
[re_c,im_c]=nyquist(Ac,Bc,Cc,0.,1,w);%Controller Only
freq_g = re_g + sqrt(-1)*im_g;
freq_c = re_c + sqrt(-1)*im_c;
G = freq_c.*freq_g;
figure(6)
axis([-ax,ax,-ax,ax]);
plot(real(G),imag(G),re_of,im_of);grid;
xlabel('Real');ylabel('Imaginary');
title('Frequency Response At Input (Two Methods)');
axis;
pause
% clg