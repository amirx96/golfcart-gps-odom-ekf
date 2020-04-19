clear;
clc;
syms phi_k theta_k psi_k 
syms x_k y_k z_k 
syms vx_k vy_k vz_k 
syms ax_k ay_k az_k 
syms phik_dot thetak_dot psik_dot dt 
syms g

Rx = [1 0 0;
      0 cos(phi_k) -sin(phi_k);
      0 sin(phi_k)  cos(phi_k)];
       
Ry = [ cos(theta_k) 0 sin(theta_k);
               0  1 0;
      -sin(theta_k) 0 cos(theta_k)];
  
Rz = [cos(psi_k) -sin(psi_k) 0;
      sin(psi_k)  cos(psi_k) 0;
              0           0  1];
          
Rotn = simplify(Rz * Ry * Rx);
g = 9.801;
aNED = Rotn*[ax_k; ay_k; az_k] + [0;0;g];

f = [x_k + vx_k*dt + 0.5*dt*dt*aNED(1);
     y_k + vy_k*dt + 0.5*dt*dt*aNED(2);
     z_k + vz_k*dt + 0.5*dt*dt*aNED(3);
     vx_k + dt*aNED(1);
     vy_k + dt*aNED(2);
     vz_k + dt*aNED(3);
     phi_k + dt*phik_dot;
     theta_k + dt*thetak_dot;
     psi_k + dt*psik_dot];
 
f = simplify(f);
       
F = simplify(jacobian(f, [x_k, y_k, z_k, vx_k, vy_k, vz_k, phi_k, theta_k, psi_k]))

B = simplify(jacobian(f, [ax_k, ay_k, az_k, phik_dot, thetak_dot, psik_dot]))
  