
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


//Sampling	period

h=0.02;

disp("Sampling period:",h)
//Discrete Dme nominal model and closed loop poles
sys_d=dscr(sys_c,h); Fi=sys_d(2);Gama=sys_d(3)
Gamau=Gama(:,1)//Gama column for thecommand input 
p_d=spec(Fi); disp("Nominal	model	poles	in	z	for	h:",p_d);

poles_s=[-50 -50];disp('Equivalent closed loop poles in the s plane:',poles_s);

//Feedbac system poles in z 
poles_z=exp(poles_s*h);disp('Feedback system poles in the z plane:', poles_z)

//Feedback gain vector 
K=ppol(Fi,Gamau,poles_z); disp('Feedback gains K1 and K2 for the regulator:',K)

tf=0
x0=[1 1]' //IniDal state 
p_amp=0 //Step disturbance amplitude

//3. ----------	SimulaDon	
tk=0:h:tf//Vector of Dme instants 
tk_size=size(tk)
N=tk_size(2) //Number of sampling instants in simulaDon

//External inputs
p=p_amp*ones(1,N)//Disturbance
//IniDal values
xk=x0

//SimulaDon loop 
for	k=0:1:N-1
//Inputs calculaDon
pk=p(k+1)
//CalculaDon of command
uk=-K*xk
if uk>u_sat_plus then uk=u_sat_plus end
if uk<u_sat_minus then uk=u_sat_minus end

//CalculaDon of average current at the instant uk is applied
ik=(uk*Kw-Km*xk(1,1))/R
//CalculaDon of next state
xkplus1=Fi*xk+Gama*[uk pk]'
//Storing values
u(1,k+1)=uk
i(1,k+1)=ik
x(:,k+1)=xk
//Shir	for next iteraDon
xk=xkplus1
end

//4.---------- PloRng responses
scf(0)
clf
UfScale=1
drawlater()
a=gca();
plot(tk',x',tk',i',tk',(u*UfScale)')

a.children.children.mark_mode='on' //Desenha pontos
a.children.children.mark_style=0 //Marca	de	ponto é círculo
a.children.children.mark_size=3 //Tamanho	da	marca	de	ponto
a.children.children(1).mark_foreground=5 //Cor	da	marca	de	ponto		
a.children.children(1).mark_background=5 //Cor	da	marca	de	ponto
a.children.children(2).mark_foreground=7 //Cor	da	marca	de	ponto		
a.children.children(2).mark_background=7 //Cor	da	marca	de	ponto
a.children.children(3).mark_foreground=2 //Cor	da	marca	de	ponto
a.children.children(3).mark_foreground=2 //Cor	da	marca	de	ponto		
a.children.children(4).mark_background=3 //Cor	da	marca	de	ponto
a.children.children(4).mark_foreground=3 //Cor da marca de ponto
a.children.children(1).foreground=5
a.children.children(2).foreground=7
a.children.children(3).foreground=2
a.children.children(4).foreground=3
a.grid=[0 0];a.x_label.text='t';a.x_label.font_size=3;a.y_label.font_size=3;
legend(['omega','teta','i','u/'+string(1/UfScale)],4)
drawnow()

//tf=1;x0=[0 0]';p_amp=0.1

//Q=]floor(θ(k)/q)×q,ceiling(θ(k)/q)×q[

