
u_sat_plus=1;
u_sat_minus=-1;


M_shad=0.106;R_shad=0.015;J_shad=0.5*M_shad*R_shad^2;

M_load=1.65;R1_load=0.015;R2_load=0.0375;
J_load=0.5*M_load*(R1_load^2+R2_load^2);
J=J_shad+J_load;


R=2.7; uax=6.0;ix=0.63;wx=2*%pi*113/60;
Km=(uax-R*ix)/wx;
Bf=Km*ix/wx;

Kw=6.0;


A=[-Bf/J-Km^2/(R*J) 0;1 0];
B=[Km*Kw/(R*J)1/J;0 0];
Bu=B(:,1);
C=[1 0;0 1];D=[0];
sys_c=syslin('c',A,B,C); sys_c_sim=syslin('c',A,Bu,C);

p_c=spec(A);
disp(p_c,"Nominal model poles in s:");

z=2;
x0=[0 0];

tk=0:0.01:z;
tk_size=size(tk);
N=tk_size(2);
rect_pulse=zeros(tk); rect_pulse(1:N/2+1)=1;
[y,x]=csim(rect_pulse,tk,sys_c_sim);

scf(0);clf;drawlater();a=gca();
plot(tk',x');
a.children.children.mark_mode='on'
a.children.children.mark_style=0
a.children.children.mark_size=3
a.children.children(1).mark_foreground=5
a.children.children(1).mark_background=5
a.children.children(2).mark_foreground=2
a.children.children(2).mark_background=2
a.children.children(1).foreground=5
a.children.children(2).foreground=2
a.grid=[0 0];

a.x_label.text='t';
a.x_label.font_size=3;
a.y_label.font_size=3;
legend(['omega','teta'],1);
drawnow();
