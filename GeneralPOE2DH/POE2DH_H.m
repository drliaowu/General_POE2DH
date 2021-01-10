function [DH_par, h_par, q_bar_par] = POE2DH_H (POE_par)
%POE2DH_H Conversion from POE parameters to DH parameters (with helical joints)
%
%   DH_par = POE2DH_H (POE_par)
%   POE_par:    POE parameters,     6 x n+1
%   DH_par:     DH parameters,      n+2 x 4
%   h_par:      h parameters,       n x 1
%   q_bar_par:  q_bar parameters,   n x 1

    n=size(POE_par,2)-1; %number of joints 

    DH_par = zeros(n+2,4);

    h_par = zeros(n,1);
    
    q_bar_par = zeros(n,1);
    
    POE_normalized = POE_par;

    for i=1:n
        
        q_bar_par(i) = norm(POE_par(1:3,i));
        
        POE_normalized(:,i) = POE_par(:,i)/q_bar_par(i);
        
    end
    
    xi = POE_normalized(:,1);

    g = eye(4);

    for i=1:n

        [theta, d, alpha, a, h]=POE2DH_Joint_H (xi);

        DH_par(i,:) = [theta; d; alpha; a];

        h_par(i) = h;

        g = g*DH (DH_par(i,1:4),'std');

        if i~=n
            
            xi = adM(inv(g))*POE_normalized(:,i+1);
        
        end

    end

    [theta1, d1, alpha1, a1, theta2, d2] = POE2DH_Tool (g\se3Exp(POE_normalized(:,n+1)));

    DH_par(n+1,:) = [theta1; d1; alpha1; a1];

    DH_par(n+2,:) = [theta2; d2; NaN; NaN];

end