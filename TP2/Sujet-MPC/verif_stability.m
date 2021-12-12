function ok = verif_stability(x_verif)
  try
    pkg load control
  end
  %parametres
  mu = 0.5;
  u0 = 0;
  x10 = 0;
  x20 = 0;
  
  %matrice de poids
  Q = [0.5,0;0,0.5];
  R = [1];
  
  %TODO ecrire les matrics A et B de la linearisation Jacobienne
  %linearises jacobienne
  A = [u0*(1-mu), 1; 1, -4*u0*(1-mu)];
  
  B = [mu+(1-mu)*x10; mu-4*(1-mu)*x20];

  %TODO avec riccati, trouver une commande stabilisante
  %essai riccati : A'P+PA-PB inv(R) B'P + Q =0
  %[x, l, g] = icare(A, B, Q, R); %Matlab
  [x, l, g] = care(A, B, Q, R); %Octave
  K = -g
  
  %TODO calculer l'equation du systeme avec rebouclage
  %systeme rebouclage
  Ak = A + B*K;

  %eigs(Ak);
  
  M = [-1,0;0,-1] - (Ak);
  %disp(det(M));
  
  %disp(eig(Ak));
  
  %TODO calculer la borne lambda et la borne alpha a 95 % de lambda
  %calcul de la borne, on retrouve bien celle de l'article
  lambda = -max(eigs(Ak));
  % borne a 95 % 
  alpha = lambda-0.05*lambda;
  
  %TODO ecrire les matrices de l'equation de Lyapunov et la resoudre pour obtenir la matrice P
  %matrice pour equation lyap
  Al = (Ak+[alpha, 0;0, alpha])';
  Bl = (Q+K'*R*K);
  
  P = lyap(Al,Bl);   
  
  %verif lyap
  %Al*P + P*Al' + Bl;
  %eigs(P);
  
  %ici on calcul la borne du probleme quadratique beta
  [x1, obj] = qp([0;0],-2*P,[],[],[],[-0.8;-0.8],[0.8;0.8],-2,K,2)
  beta = -obj
  
  %TODO ecrire le test qui valide ou non si le point est dans la zone de stabilite du controleur  
  test = x_verif' * P * x_verif;
  ok = (test < beta);
  
  
endfunction