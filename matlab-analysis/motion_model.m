function [state_out, P_out] = motion_model(state_in, u_k, P_in, Q, dt)
    x_k = state_in(1);
    y_k = state_in(2);
    z_k = state_in(3);
    vx_k = state_in(4);
    vy_k = state_in(5);
    vz_k = state_in(6);
    phi_k = atan2(sin(state_in(7)), cos(state_in(7)));
    theta_k = atan2(sin(state_in(8)), cos(state_in(8)));
    psi_k = atan2(sin(state_in(9)), cos(state_in(9)));
    ax_k = u_k(1);
    ay_k = u_k(2);
    az_k = u_k(3);
    phik_dot = u_k(4);
    thetak_dot = u_k(5);
    psik_dot = u_k(6);
    
    Rx = [1 0 0;
          0 cos(phi_k) -sin(phi_k);
          0 sin(phi_k)  cos(phi_k)];
       
    Ry = [ cos(theta_k)  0 sin(theta_k);
                      0  1 0;
           -sin(theta_k) 0 cos(theta_k)];
  
    Rz = [cos(psi_k) -sin(psi_k) 0;
          sin(psi_k)  cos(psi_k) 0;
                  0           0  1];
          
    Rotn = Rz * Ry * Rx;    
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
    
    F = [ 1, 0, 0, dt,  0,  0,  (dt^2*(ay_k*(sin(phi_k)*sin(psi_k) + cos(phi_k)*cos(psi_k)*sin(theta_k)) + az_k*(cos(phi_k)*sin(psi_k) - cos(psi_k)*sin(phi_k)*sin(theta_k))))/2, (dt^2*cos(psi_k)*(az_k*cos(phi_k)*cos(theta_k) - ax_k*sin(theta_k) + ay_k*cos(theta_k)*sin(phi_k)))/2, -(dt^2*(ay_k*(cos(phi_k)*cos(psi_k) + sin(phi_k)*sin(psi_k)*sin(theta_k)) - az_k*(cos(psi_k)*sin(phi_k) - cos(phi_k)*sin(psi_k)*sin(theta_k)) + ax_k*cos(theta_k)*sin(psi_k)))/2;
          0, 1, 0,  0, dt,  0, -(dt^2*(ay_k*(cos(psi_k)*sin(phi_k) - cos(phi_k)*sin(psi_k)*sin(theta_k)) + az_k*(cos(phi_k)*cos(psi_k) + sin(phi_k)*sin(psi_k)*sin(theta_k))))/2, (dt^2*sin(psi_k)*(az_k*cos(phi_k)*cos(theta_k) - ax_k*sin(theta_k) + ay_k*cos(theta_k)*sin(phi_k)))/2,  (dt^2*(az_k*(sin(phi_k)*sin(psi_k) + cos(phi_k)*cos(psi_k)*sin(theta_k)) - ay_k*(cos(phi_k)*sin(psi_k) - cos(psi_k)*sin(phi_k)*sin(theta_k)) + ax_k*cos(psi_k)*cos(theta_k)))/2;
          0, 0, 1,  0,  0, dt,                                                                                         (dt^2*cos(theta_k)*(ay_k*cos(phi_k) - az_k*sin(phi_k)))/2,           -(dt^2*(ax_k*cos(theta_k) + az_k*cos(phi_k)*sin(theta_k) + ay_k*sin(phi_k)*sin(theta_k)))/2,                                                                                                                                                                                0;
          0, 0, 0,  1,  0,  0,        dt*(ay_k*(sin(phi_k)*sin(psi_k) + cos(phi_k)*cos(psi_k)*sin(theta_k)) + az_k*(cos(phi_k)*sin(psi_k) - cos(psi_k)*sin(phi_k)*sin(theta_k))),       dt*cos(psi_k)*(az_k*cos(phi_k)*cos(theta_k) - ax_k*sin(theta_k) + ay_k*cos(theta_k)*sin(phi_k)),       -dt*(ay_k*(cos(phi_k)*cos(psi_k) + sin(phi_k)*sin(psi_k)*sin(theta_k)) - az_k*(cos(psi_k)*sin(phi_k) - cos(phi_k)*sin(psi_k)*sin(theta_k)) + ax_k*cos(theta_k)*sin(psi_k));
          0, 0, 0,  0,  1,  0,       -dt*(ay_k*(cos(psi_k)*sin(phi_k) - cos(phi_k)*sin(psi_k)*sin(theta_k)) + az_k*(cos(phi_k)*cos(psi_k) + sin(phi_k)*sin(psi_k)*sin(theta_k))),       dt*sin(psi_k)*(az_k*cos(phi_k)*cos(theta_k) - ax_k*sin(theta_k) + ay_k*cos(theta_k)*sin(phi_k)),        dt*(az_k*(sin(phi_k)*sin(psi_k) + cos(phi_k)*cos(psi_k)*sin(theta_k)) - ay_k*(cos(phi_k)*sin(psi_k) - cos(psi_k)*sin(phi_k)*sin(theta_k)) + ax_k*cos(psi_k)*cos(theta_k));
          0, 0, 0,  0,  0,  1,                                                                                               dt*cos(theta_k)*(ay_k*cos(phi_k) - az_k*sin(phi_k)),                 -dt*(ax_k*cos(theta_k) + az_k*cos(phi_k)*sin(theta_k) + ay_k*sin(phi_k)*sin(theta_k)),                                                                                                                                                                                0;
          0, 0, 0,  0,  0,  0,                                                                                                                                                 1,                                                                                                     0,                                                                                                                                                                                0;
          0, 0, 0,  0,  0,  0,                                                                                                                                                 0,                                                                                                     1,                                                                                                                                                                                0;
          0, 0, 0,  0,  0,  0,                                                                                                                                                 0,                                                                                                     0,                                                                                                                                                                                1]; 

    B = [ (dt^2*cos(psi_k)*cos(theta_k))/2, -(dt^2*(cos(phi_k)*sin(psi_k) - cos(psi_k)*sin(phi_k)*sin(theta_k)))/2,  (dt^2*(sin(phi_k)*sin(psi_k) + cos(phi_k)*cos(psi_k)*sin(theta_k)))/2,  0,  0,  0;
          (dt^2*cos(theta_k)*sin(psi_k))/2,  (dt^2*(cos(phi_k)*cos(psi_k) + sin(phi_k)*sin(psi_k)*sin(theta_k)))/2, -(dt^2*(cos(psi_k)*sin(phi_k) - cos(phi_k)*sin(psi_k)*sin(theta_k)))/2,  0,  0,  0;
          -(dt^2*sin(theta_k))/2,                                       (dt^2*cos(theta_k)*sin(phi_k))/2,                                       (dt^2*cos(phi_k)*cos(theta_k))/2,  0,  0,  0;
           dt*cos(psi_k)*cos(theta_k),       dt*cos(psi_k)*sin(phi_k)*sin(theta_k) - dt*cos(phi_k)*sin(psi_k),        dt*(sin(phi_k)*sin(psi_k) + cos(phi_k)*cos(psi_k)*sin(theta_k)),  0,  0,  0;
           dt*cos(theta_k)*sin(psi_k),        dt*(cos(phi_k)*cos(psi_k) + sin(phi_k)*sin(psi_k)*sin(theta_k)),       dt*cos(phi_k)*sin(psi_k)*sin(theta_k) - dt*cos(psi_k)*sin(phi_k),  0,  0,  0;
                  -dt*sin(theta_k),                                             dt*cos(theta_k)*sin(phi_k),                                             dt*cos(phi_k)*cos(theta_k),  0,  0,  0;
                                 0,                                                                      0,                                                                      0, dt,  0,  0;
                                 0,                                                                      0,                                                                      0,  0, dt,  0;
                                 0,                                                                      0,                                                                      0,  0,  0, dt];
                             
     state_out = f;
     state_out(7) = atan2(sin(state_out(7)), cos(state_out(7)));
     state_out(8) = atan2(sin(state_out(8)), cos(state_out(8)));
     state_out(9) = atan2(sin(state_out(9)), cos(state_out(9)));
     P_out = F*P_in*F' + B*Q*B';
end