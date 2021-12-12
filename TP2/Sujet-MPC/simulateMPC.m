function simulateMPC(xinit,K,mu)
  
  dt = 0.01;
  x = xinit;
  xstore = NaN*zeros(2,10000);
  k = 1;
  u = 1;
  
  n = 4;
  while (norm(x) > 0.001) && (k<2000)
    xstore(:,k) = x;
    
    
    %TODO linearisation en x et t
    A = eye(2,2)+dt*[u*(1-mu), 1;1, -u*4*(1-mu)];
    B = dt*[mu+(1-mu)*x(1);mu-4*(1-mu)*x(2)];
    
    %vecteur d'entrées
    U = [u,u,u,u]';
    H = eye(n,n)*2;
    
    %TODO écrire les matrices de la commande prédictive linéaire
    Aqp = -[A;A*A;A*A*A;A*A*A*A]*x;
    
    Bqp = [B, zeros(2, n-1);
           A*B,B,zeros(2, n-2);
           A*A*B, A*B, B, zeros(2,n-3);
           A*A*A*B, A*A*B, A*B, B];
      
    %TODO avec une pseudo inverse, calculer le vecteur d'entrées
    U = (Bqp'*Bqp)\Bqp'*Aqp;

    
    if size(K)==0
      u = U(1);
    else
      u = -K*x;
    end
    
    if (u > 2)
      u = 2;
    elseif (u<-2)
      u = -2;
    end
    
    %simu avec euler
    x1 = x(1);
    x2 = x(2);
    x(1) = x1 + dt*(x2 + u*(mu + (1-mu)*x1));
    x(2) = x2 + dt*(x1+u*(mu-4*(1-mu)*x2));
    
    k = k+1;
  end
  
  if norm(x) < 0.01
    plot(xstore(1,:), xstore(2,:),'+');
  else
    disp("fail!")
  end
end
