clear all
close all


load Traj;   %  charger le trajectoire
plot(trajectoire(1,:),trajectoire(2,:),'o-b')
pause(0.1)
hold on

ts=1; % la p�riode d'�chantillionage
sigmau=2; % �cart type du vecteur d'acceleration (bruit d'�tat)
sigmav=0.0000001; % �cart type du vecteur bruit d'observation (mesure...)
A=[1 0 ts 0;0 1 0 ts;0 0 1 0;0 0 0 1]; % matrice de transition
B=[ts^2/2 0;0 ts^2/2;ts 0;0 ts]; % matrice pour transformation lin�aire du bruit d'�tat


xi=[20000;30000]; % initialisation du vecteur position
plot(xi(1),xi(2),'*-r');hold on;
V=zeros(2,1);% vecteur initialisation de la vitesse correspond à V(0|0)
Xi=[xi;V];% vecteur d��tat  correspond à z(0|0) 
Mi=1000*eye(4);% matrice de covariance initiale   correspond à P(0|0)

Mk1=[-1500;1500];  % position de la station 1
Mk2=[20000;20000]; % position de la station 2
Mk3=[25000;500];   % position de la station 3


% visualisation des stations
plot(Mk1(1),Mk1(2),'s-k');hold on;
plot(Mk2(1),Mk2(2),'s-k');hold on;
plot(Mk3(1),Mk3(2),'s-k');hold on;
plot(Mk1(1),Mk1(2),'*-k');hold on;
plot(Mk2(1),Mk2(2),'*-k');hold on;
plot(Mk3(1),Mk3(2),'*-k');hold on;

c=3*10^8; % la c�l�rit� de la lumi�re dans le vide
c=1/c;

for l=1:length(trajectoire),

% étape de pr�diction    
Xpredit=A*Xi; % vecteur de position correspond à x(n|n-1) dans l'algorithme de Kalman
Mpredit=A*Mi*A'+sigmau^2*B*B';% matrice de covariance correspond à P(n|n-1) dans l'algorithme de Kalman

% calcul du matrice inverse de transition   de l'observation y(n|n-1) car on a besoin de 
% l'équation d'observation afin d'appliquer le filtre de Kalman
h1=c*(Xpredit(1:2)-Mk1)/(norm(Xpredit(1:2)-Mk1)) ;
h2=c*(Xpredit(1:2)-Mk2)/(norm(Xpredit(1:2)-Mk2)) ;
h3=c*(Xpredit(1:2)-Mk3)/(norm(Xpredit(1:2)-Mk3)) ;
% étendre les vecteurs par 2 zéros car Xpredit a des 0 dans l'indice 3 et
% 4 (calculer A*Xi)
h1=[h1;0;0];
h2=[h2;0;0];
h3=[h3;0;0];
% remplir la matrice de transition d'observation inverse
h=[h1 h2 h3];

%calcul de l'observation pr�dite correspond à y(n|n-1) dans le filtre de Kalman 
Ypredit=c*[norm(Xpredit(1:2)-Mk1);norm(Xpredit(1:2)-Mk2);norm(Xpredit(1:2)-Mk3)];
%calcul de la covariance de l'observation pr�dite correspond à F(n|n-1) dans le filtre de Kalman
var_sortie=h'*Mpredit*h+sigmav^2*eye(3);


n1=sigmav*randn;   % génération du bruit de mesure
n2=sigmav*randn;
n3=sigmav*randn;

% génération des vraies observations qu'on va utiliser pour la localisation
% correspond é y(n) dans le modèle d'état
Yk=[c*norm(trajectoire(:,l)-Mk1)+n1 ; c*norm(trajectoire(:,l)-Mk2)+n2 ; c*norm(trajectoire(:,l)-Mk3)+n3] ;%Yk(n)

%TOA1(l)=abs(c*norm(trajectoire(:,l)-Mk1));
%relative_err1(l)=abs(n1)/TOA1(l);
u1(:,l) = (trajectoire(:,l)-Mk1)'*(trajectoire(:,l)-Mk1);
u2(:,l) = (trajectoire(:,l)-Mk2)'*(trajectoire(:,l)-Mk2);
u3(:,l) = (trajectoire(:,l)-Mk3)'*(trajectoire(:,l)-Mk3);


%étape de filtrage
gain=Mpredit*h*inv(var_sortie); % calcul du gain de Kalman correspond ? K(n) dans l'alg.
err=Yk-Ypredit; % correspond ? y(n)-y(n|n-1) dans l'alg.

Xestime=Xpredit+gain*err; %  vecteur de l'�tat filtr� correspond à x(n|n) dans l'alg.
Mestime=Mpredit-gain*h'*Mpredit;%  matrice de covariance de l'�tat filtr� correspond à P(n|n) dans l'alg.

% visualisation de l'�stimation 
%if mod(l,3)==0, % display true and estimated trajectory every 5Ts (here 5 sec in real time)
hold on
plot(Xestime(1),Xestime(2),'*-r')
pause(0.01)
%end

en(l)=norm(trajectoire(:,l)-Xestime(1:2)); % calcul de l'erreur
e(l)=en(l).^2;
DtoTP(l)=mean(en); % calcul de DtoP moyenne (distance to true position)
EQM(l)=mean(e); % calcul de l'erreur quadratique moyenne

%traceP(l)=trace(Mestime);
%tracePM(l)=mean(traceP);

Xi=Xestime;
Mi=Mestime;


end
RSB = 10*log10(c*sum(u1+u2+u3)/(3*length(trajectoire)*sigmav))

threshhold = max(EQM)/100;
periode_transitoire = 0;
for i=1:length(EQM),
    if ( EQM(i) <= threshhold)
        periode_transitoire = i
        break;
    end 
end

figure(2)
plot(en);hold on;plot(EQM,'r');hold off