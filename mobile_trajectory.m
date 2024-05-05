clear all
close all

randn('seed',50); % initialisation du générateur pseudo-aléatoire de matlab à la valeur 50,
                  % cette initialisation sert àgarantir la reproductibilit� des r�sultats

ts=1; % la p�riode d'�chantillionage
A=[1 0 ts 0;0 1 0 ts;0 0 1 0;0 0 0 1];% matrice de transition
B=[ts^2/2 0;0 ts^2/2;ts 0;0 ts];% B*b(n): bruit d'�tat
sigma = 2;% % �cart type du vecteur d'acceleration
N=500;% % nombre de transitions

x=1000*ones(2,1);%vecteur initialisation de la position de la cible correspond à  x(0|0)
V=zeros(2,1);    %vecteur initialisation de la vitesse correspond à V(0|0) 
z=[x;V];         % vecteur d��tat
trajectoire=[];  % vecteur pour stocker la position de la cible

for n = 1:N,
   u= sigma*randn(2,1);% génération du vecteur d'acceleration, sigma est l'�cart type du vecteur
   z= A*z+B*u;         % équation de transition d'�tat pour l'�tape n
   x1=z(1);
   x2=z(2);
   hold on
   plot(x1,x2,'*-r')    % visualisation du trajectoire
   pause(0.001)         % attendre 1 milli sec avant de conyinuer
   trajectoire=[trajectoire [x1;x2]];  % trajectoire contient la position pour l'indice n
 end


save Traj trajectoire;  % enregistrer la variable trajectoire dans Traj.mat
