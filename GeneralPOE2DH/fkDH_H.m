function g = fkDH_H (DH_par, h_par, H_base, H_tool, q, JointType, ModelVersion)
%fkDH Forward kinematic using DH parameters with helical joints
%
%   g = fkDH (DH_par, h_par, H_base, H_tool, q, JointType, ModelVersion)
%   DH_par:         DH parameters, n x 4
%   h_par:          h parameters, n x 1
%   H_base:         Homogeneous transformation from base frame to first
%                   link frame
%   H_tool:         Homogeneous transformation from last link frame to tool
%                   frame
%   q:              Joint variables, n x 1
%   JointType:      'R'-revolute joint
%                   'P'-prismatic joint
%   ModelVersion:   'std'-standard version
%                   'mdf'-modified version
%   g:              Homogeneous transformation, 4 x 4

switch ModelVersion
    
    case 'std' % standard model

        n = size(DH_par,1);
        
        g = H_base;

        for i=1:n
        
            if JointType(i)=='R' % revolute joint
            
                g=g*DH(DH_par(i,:)+[q(i),0,0,0],'std');
            
            elseif JointType(i)=='P' % prismatic joint
            
                g=g*DH(DH_par(i,:)+[0,q(i),0,0],'std');
            
            elseif JointType(i)=='H' % helical joint
            
                g=g*DH(DH_par(i,:)+[q(i),h_par(i)*q(i),0,0],'std');
            
            else
                
                error('Illegal Joint Type.')
            
            end
            
        end

        g = g*H_tool;

    case 'mdf' % modified model
        
        n = size(DH_par,1);
        
        g = H_base;

        for i=1:n
        
            if JointType(i)=='R' % revolute joint
            
                g=g*DH(DH_par(i,:)+[0,0,q(i),0],'mdf');
            
            elseif JointType(i)=='P' % prismatic joint
            
                g=g*DH(DH_par(i,:)+[0,0,0,q(i)],'mdf');
            
            elseif JointType(i)=='H' % helical joint
            
                g=g*DH(DH_par(i,:)+[0,0,q(i),h_par(i)*q(i)],'mdf');
            
            else
                
                error('Illegal Joint Type.')
            
            end
            
        end

        g = g*H_tool;
        
    otherwise
        
        error('Illegal model version.')
        
end