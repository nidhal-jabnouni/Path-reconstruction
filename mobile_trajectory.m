clear all
close all

randn('seed',50); % initialisation du g√©n√©rateur pseudo-al√©atoire de matlab √† la valeur 50,
                  % cette initialisation sert √†garantir la reproductibilitÈ des rÈsultats

ts=1; % la pÈriode d'Èchantillionage
A=[1 0 ts 0;0 1 0 ts;0 0 1 0;0 0 0 1];% matrice de transition
B=[ts^2/2 0;0 ts^2/2;ts 0;0 ts];% B*b(n): bruit d'Ètat
sigma = 2;% % Ècart type du vecteur d'acceleration
N=500;% % nombre de transitions

x=1000*ones(2,1);%vecteur initialisation de la position de la cible correspond √†  x(0|0)
V=zeros(2,1);    %vecteur initialisation de la vitesse correspond √† V(0|0) 
z=[x;V];         % vecteur díÈtat
trajectoire=[];  % vecteur pour stocker la position de la cible

for n = 1:N,
   u= sigma*randn(2,1);% g√©n√©ration du vecteur d'acceleration, sigma est l'Ècart type du vecteur
   z= A*z+B*u;         % √©quation de transition d'Ètat pour l'Ètape n
   x1=z(1);
   x2=z(2);
   hold on
   plot(x1,x2,'*-r')    % visualisation du trajectoire
   pause(0.001)         % attendre 1 milli sec avant de conyinuer
   trajectoire=[trajectoire [x1;x2]];  % trajectoire contient la position pour l'indice n
 end


save Traj trajectoire;  % enregistrer la variable trajectoire dans Traj.mat
