%% Control law and differentiation
function xdot = dv_BPE_Rescaling(t, x)

    global N na kd kp n1 m g e3 Tsim w Q v
    
    waitbar(t/Tsim);

    % Convert x to known parameters
    P    = reshape(      x(1: 3*na), [3,1,na]); % Position
    V    = reshape( x(3*na+1: 6*na), [3,1,na]); % Velocity
    R    = reshape( x(6*na+1:15*na), [3,3,na]); % Attitude
    Pdes = reshape(x(15*na+1:18*na), [3,1,na]); % Desired position
    
    if Q == 1
        % Define desired motion    
        Pd(:,:,1) = [                           0; v*t;                            0];
        Pd(:,:,2) = [       (3-4/Tsim*t)*sin(w*t); v*t;        (3-4/Tsim*t)*cos(w*t)];
        Pd(:,:,3) = [(3-4/Tsim*t)*sin(w*t-2*pi/3); v*t; (3-4/Tsim*t)*cos(w*t-2*pi/3)];
        Pd(:,:,4) = [(3-4/Tsim*t)*sin(w*t-4*pi/3); v*t; (3-4/Tsim*t)*cos(w*t-4*pi/3)];

        Vd(:,:,1) = [                                                    0; v;                                                      0];
        Vd(:,:,2) = [              w*(3-4/Tsim*t)*cos(w*t)-4/Tsim*sin(w*t); v;               -w*(3-4/Tsim*t)*sin(w*t)-4/Tsim*cos(w*t)];
        Vd(:,:,3) = [w*(3-4/Tsim*t)*cos(w*t-2*pi/3)-4/Tsim*sin(w*t-2*pi/3); v; -w*(3-4/Tsim*t)*sin(w*t-2*pi/3)-4/Tsim*cos(w*t-2*pi/3)];
        Vd(:,:,4) = [w*(3-4/Tsim*t)*cos(w*t-4*pi/3)-4/Tsim*sin(w*t-4*pi/3); v; -w*(3-4/Tsim*t)*sin(w*t-4*pi/3)-4/Tsim*cos(w*t-4*pi/3)];

        Ud(:,:,1) = [                                                         0; 0;                                                         0];
        Ud(:,:,2) = [              -w^2*(3-4/Tsim*t)*sin(w*t)-w*8/Tsim*cos(w*t); 0;               -w^2*(3-4/Tsim*t)*cos(w*t)+w*8/Tsim*sin(w*t)];
        Ud(:,:,3) = [-w^2*(3-4/Tsim*t)*sin(w*t-2*pi/3)-w*8/Tsim*cos(w*t-2*pi/3); 0; -w^2*(3-4/Tsim*t)*cos(w*t-2*pi/3)+w*8/Tsim*sin(w*t-2*pi/3)];
        Ud(:,:,4) = [-w^2*(3-4/Tsim*t)*sin(w*t-4*pi/3)-w*8/Tsim*cos(w*t-4*pi/3); 0; -w^2*(3-4/Tsim*t)*cos(w*t-4*pi/3)+w*8/Tsim*sin(w*t-4*pi/3)];

        Uddot(:,:,1) = [                                                            0; 0;                                                             0];
        Uddot(:,:,2) = [              -w^3*(3-4/Tsim*t)*cos(w*t)+w^2*12/Tsim*sin(w*t); 0;               -w^3*(3-4/Tsim*t)*cos(w*t)+w^2*12/Tsim*sin(w*t)];
        Uddot(:,:,3) = [-w^3*(3-4/Tsim*t)*cos(w*t-2*pi/3)+w^2*12/Tsim*sin(w*t-2*pi/3); 0; -w^3*(3-4/Tsim*t)*cos(w*t-2*pi/3)+w^2*12/Tsim*sin(w*t-2*pi/3)];
        Uddot(:,:,4) = [-w^3*(3-4/Tsim*t)*cos(w*t-4*pi/3)+w^2*12/Tsim*sin(w*t-4*pi/3); 0; -w^3*(3-4/Tsim*t)*cos(w*t-4*pi/3)+w^2*12/Tsim*sin(w*t-4*pi/3)];
        
    elseif Q == 2
        % Define desired motion    
        Pd(:,:,1) = [                            0; v*t;                             0];
        Pd(:,:,2) = [       (-1+4/Tsim*t)*sin(w*t); v*t;        (-1+4/Tsim*t)*cos(w*t)];
        Pd(:,:,3) = [(-1+4/Tsim*t)*sin(w*t-2*pi/3); v*t; (-1+4/Tsim*t)*cos(w*t-2*pi/3)];
        Pd(:,:,4) = [(-1+4/Tsim*t)*sin(w*t-4*pi/3); v*t; (-1+4/Tsim*t)*cos(w*t-4*pi/3)];

        Vd(:,:,1) = [                                                    0; v;                                                      0];
        Vd(:,:,2) = [              w*(-1+4/Tsim*t)*cos(w*t)+4/Tsim*sin(w*t); v;               -w*(-1+4/Tsim*t)*sin(w*t)+4/Tsim*cos(w*t)];
        Vd(:,:,3) = [w*(-1+4/Tsim*t)*cos(w*t-2*pi/3)+4/Tsim*sin(w*t-2*pi/3); v; -w*(-1+4/Tsim*t)*sin(w*t-2*pi/3)+4/Tsim*cos(w*t-2*pi/3)];
        Vd(:,:,4) = [w*(-1+4/Tsim*t)*cos(w*t-4*pi/3)+4/Tsim*sin(w*t-4*pi/3); v; -w*(-1+4/Tsim*t)*sin(w*t-4*pi/3)+4/Tsim*cos(w*t-4*pi/3)];

        Ud(:,:,1) = [                                                         0; 0;                                                         0];
        Ud(:,:,2) = [              -w^2*(-1+4/Tsim*t)*sin(w*t)+w*8/Tsim*cos(w*t); 0;               -w^2*(-1+4/Tsim*t)*cos(w*t)-w*8/Tsim*sin(w*t)];
        Ud(:,:,3) = [-w^2*(-1+4/Tsim*t)*sin(w*t-2*pi/3)+w*8/Tsim*cos(w*t-2*pi/3); 0; -w^2*(-1+4/Tsim*t)*cos(w*t-2*pi/3)-w*8/Tsim*sin(w*t-2*pi/3)];
        Ud(:,:,4) = [-w^2*(-1+4/Tsim*t)*sin(w*t-4*pi/3)+w*8/Tsim*cos(w*t-4*pi/3); 0; -w^2*(-1+4/Tsim*t)*cos(w*t-4*pi/3)-w*8/Tsim*sin(w*t-4*pi/3)];

        Uddot(:,:,1) = [                                                            0; 0;                                                             0];
        Uddot(:,:,2) = [              -w^3*(-1+4/Tsim*t)*cos(w*t)-w^2*12/Tsim*sin(w*t); 0;               -w^3*(-1+4/Tsim*t)*cos(w*t)-w^2*12/Tsim*sin(w*t)];
        Uddot(:,:,3) = [-w^3*(-1+4/Tsim*t)*cos(w*t-2*pi/3)-w^2*12/Tsim*sin(w*t-2*pi/3); 0; -w^3*(-1+4/Tsim*t)*cos(w*t-2*pi/3)-w^2*12/Tsim*sin(w*t-2*pi/3)];
        Uddot(:,:,4) = [-w^3*(-1+4/Tsim*t)*cos(w*t-4*pi/3)-w^2*12/Tsim*sin(w*t-4*pi/3); 0; -w^3*(-1+4/Tsim*t)*cos(w*t-4*pi/3)-w^2*12/Tsim*sin(w*t-4*pi/3)];
        
    elseif Q == 3
        % Define desired motion    
        Pd(:,:,1) = [                 0; v*t;                   0];
        Pd(:,:,2) = [    2*cos(w*t) - 1; v*t;            sin(w*t)];
        Pd(:,:,3) = [     cos(w*t-pi/2); v*t; 2*sin(w*t-pi/2) + 1];
        Pd(:,:,4) = [ 2*cos(w*t-pi) + 1; v*t;         sin(w*t-pi)];

        Vd(:,:,1) = [               0; v;                 0];
        Vd(:,:,2) = [   -2*sin(w*t)*w; v;        cos(w*t)*w];
        Vd(:,:,3) = [-sin(w*t-pi/2)*w; v; 2*cos(w*t-pi/2)*w];
        Vd(:,:,4) = [-2*sin(w*t-pi)*w; v;     cos(w*t-pi)*w];

        Ud(:,:,1) = [                 0; 0;                    0];
        Ud(:,:,2) = [   -2*cos(w*t)*w^2; 0;        -sin(w*t)*w^2];
        Ud(:,:,3) = [-cos(w*t-pi/2)*w^2; 0; -2*sin(w*t-pi/2)*w^2];
        Ud(:,:,4) = [-2*cos(w*t-pi)*w^2; 0;     -sin(w*t-pi)*w^2];

        Uddot(:,:,1) = [                 0; 0;                    0];
        Uddot(:,:,2) = [    2*sin(w*t)*w^3; 0;        -cos(w*t)*w^3];
        Uddot(:,:,3) = [ sin(w*t-pi/2)*w^3; 0; -2*cos(w*t-pi/2)*w^3];
        Uddot(:,:,4) = [ 2*sin(w*t-pi)*w^3; 0;     -cos(w*t-pi)*w^3];
        
    elseif Q == 4
        % Define desired motion    
        Pd(:,:,1) = [                 0; v*t;                   0];
        Pd(:,:,2) = [    2*cos(w*t) - 1; v*t;            sin(w*t)];
        Pd(:,:,3) = [     cos(w*t-pi/2); v*t; 2*sin(w*t-pi/2) + 1];
        Pd(:,:,4) = [ 2*cos(w*t-pi) + 1; v*t;         sin(w*t-pi)];

        Vd(:,:,1) = [               0; v;                 0];
        Vd(:,:,2) = [   -2*sin(w*t)*w; v;        cos(w*t)*w];
        Vd(:,:,3) = [-sin(w*t-pi/2)*w; v; 2*cos(w*t-pi/2)*w];
        Vd(:,:,4) = [-2*sin(w*t-pi)*w; v;     cos(w*t-pi)*w];

        Ud(:,:,1) = [                 0; 0;                    0];
        Ud(:,:,2) = [   -2*cos(w*t)*w^2; 0;        -sin(w*t)*w^2];
        Ud(:,:,3) = [-cos(w*t-pi/2)*w^2; 0; -2*sin(w*t-pi/2)*w^2];
        Ud(:,:,4) = [-2*cos(w*t-pi)*w^2; 0;     -sin(w*t-pi)*w^2];

        Uddot(:,:,1) = [                 0; 0;                    0];
        Uddot(:,:,2) = [    2*sin(w*t)*w^3; 0;        -cos(w*t)*w^3];
        Uddot(:,:,3) = [ sin(w*t-pi/2)*w^3; 0; -2*cos(w*t-pi/2)*w^3];
        Uddot(:,:,4) = [ 2*sin(w*t-pi)*w^3; 0;     -cos(w*t-pi)*w^3];
    end

    % Feedforward
    U    = Ud;
    
    % Control system for each agent
    for i = 1:na

        % Position control
        for j = N{i}
            pij = P(:,:,j) - P(:,:,i);
            vij = V(:,:,j) - V(:,:,i);
            gij = pij/norm(pij); % In practice this will be the bearing measurement
            pi_gij = eye(3) - gij*gij.';
            pij_d = Pd(:,:,j) - Pd(:,:,i);
            u_n = -kp(i)*pi_gij*pij_d;
            U(:,:,i) = U(:,:,i) + u_n;
        end
        vti = V(:,:,i) - Vd(:,:,i);
        U(:,:,i) = U(:,:,i) - kd(i)*vti;
        
        if i == 1
            pt = P(:,:,i) - Pd(:,:,i);
            vt = V(:,:,i) - Vd(:,:,i);
            u_n = (-kp(i)*pt - kd(i)*vt);
            U(:,:,i) = U(:,:,i) + u_n;
        end
        
        TRd = -(m(i)*U(:,:,i) - m(i)*g*e3);
        T   = norm(TRd);
        
        % Desired attitude computation (PE implementation)
        Rde3  = TRd/T;

        % Attitude controller (unchanged)
        Omega = n1*skew(e3)*R(:,:,i).'*Rde3 + m(i)/T*(eye(3) - e3*e3')*R(:,:,i).'*skew(Rde3)*Uddot(:,:,i);

        % System derivatives
        Pdot(:,:,i) = V(:,:,i);
        Vdot(:,:,i) = -T/m(i)*R(:,3,i) + g*e3;
        Rdot(:,:,i) = R(:,:,i)*skew(Omega);
        Pdesdot(:,:,i) = Vd(:,:,i);
    
    end

    xdot = [Pdot(:); Vdot(:); Rdot(:); Pdesdot(:)];
    
end
