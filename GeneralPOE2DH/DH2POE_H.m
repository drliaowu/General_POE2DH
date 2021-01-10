function [ POE_par ] = DH2POE_H( DH_par, h_par, H_base, H_tool, JointType, ModelVersion)
%DH2POE_H Conversion from standar DH parameters to POE parameters with
%helical joints
%
%   POE_par = DH2POE (DH_par, h_par, H_base, H_tool, JointType, ModelVersion)
%   DH_par:         DH parameters, n x 4
%   h_par:          h parameters, n x 1
%   H_base:         Homogeneous transformation from base frame to first
%                   link frame
%   H_tool:         Homogeneous transformation from last link frame to tool
%                   frame
%   JointType:      'R'-revolute joint
%                   'P'-prismatic joint
%                   'H'-helical joint
%   ModelVersion:   'std'-standard version
%                   'mdf'-modified version
%   POE_par:        POE parameters,  6 x n+1

switch ModelVersion

    case 'std'  %standard version
        
        n = size(DH_par,1);

        POE_par = zeros(6,n+1);

        H=H_base;

        if JointType(1)=='R'

            POE_par(:,1)=adM(H)*[0;0;1;0;0;0];

        elseif JointType(1)=='P'

            POE_par(:,1)=adM(H)*[0;0;0;0;0;1];

        elseif JointType(1)=='H'

            POE_par(:,1)=adM(H)*[0;0;1;0;0;h_par(1)];

        else

            error('Illegal Joint Type.')

        end

        for i=1:n-1

            H=H*DH(DH_par(i,:),'std');

            if JointType(i+1)=='R'  %revolute joint

                POE_par(:,i+1)=adM(H)*[0;0;1;0;0;0];

            elseif JointType(i+1)=='P'  %prismatic joint

                POE_par(:,i+1)=adM(H)*[0;0;0;0;0;1];

            elseif JointType(i+1)=='H'  %helical joint

                POE_par(:,i+1)=adM(H)*[0;0;1;0;0;h_par(i+1)];

            else

                error('Illegal Joint Type.')

            end

        end

        H=H*DH(DH_par(n,:),'std')*H_tool;

        POE_par(:,n+1)=vlog(H);
        
    case 'mdf'

        n = size(DH_par,1); %number of joints

        POE_par = zeros(6,n+1);

        H=H_base*DH(DH_par(1,:),'mdf');

        if JointType(1)=='R'    %revolute joint

            POE_par(:,1)=adM(H)*[0;0;1;0;0;0];

        elseif JointType(1)=='P'    %prismatic joint

            POE_par(:,1)=adM(H)*[0;0;0;0;0;1];
                        
        elseif JointType(1)=='H'    %helical joint

            POE_par(:,1)=adM(H)*[0;0;1;0;0;h_par(1)];

        else

            error('Illegal Joint Type.')

        end

        for i=1:n-1

            H=H*DH(DH_par(i+1,:),'mdf');

            if JointType(i+1)=='R'

                POE_par(:,i+1)=adM(H)*[0;0;1;0;0;0];

            elseif JointType(i+1)=='P'

                POE_par(:,i+1)=adM(H)*[0;0;0;0;0;1];
                            
            elseif JointType(i+1)=='H'

                POE_par(:,i+1)=adM(H)*[0;0;1;0;0;h_par(i+1)];

            else

                error('Illegal Joint Type.')

            end

        end

        H=H*H_tool;

        POE_par(:,n+1)=vlog(H);

    otherwise
        
        error('Illegal model version.')

end