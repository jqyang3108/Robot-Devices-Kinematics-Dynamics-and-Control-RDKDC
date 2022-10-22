% input the desired pose, gain K, ur5 and the input mode
function finalerr = ur5RRcontrol(gdesired, K, ur5, mode)
    % Implement a discrete-time resolved rate control system.
     
    % set up parameters
    vk_thres = 0.001;  
    wk_thres = pi/100;
    tstep = 0.0005;  
    pause(1);
    qini = ur5.get_current_joints();
    
    gini = ur5FwdKin(qini);    %gst
    
    qk = qini;
    k = 1;
    % get initial xik
    xiK = getXi(inv(gdesired)*gini);

    % iteratively implement the control system 
    for T= 0:tstep:1-tstep
        fprintf('iteration: %d\n',k);
        normv = norm(xiK(1:3,:));
        normw = norm(xiK(4:6,:));
        if (normv < vk_thres && normw < wk_thres)
            disp('ABORT: The norm of vk or wk is less than threshold');
            finalerr = -1;
            return 
        else
            % calculate the joints configuration for the next step 
            if mode == 2 
                grad = inv(ur5BodyJacobian(qk));
            elseif mode ==3
                grad = transpose(ur5BodyJacobian(qk));
            end
            qk = qk-(K*tstep*grad*xiK); %discrete-time resolved-rate control
            error = inv(gdesired)*ur5FwdKin(qk);
            xiK = getXi(error); %get xiK of Qk+1
            
            % check for singularity             
            Jdet = abs(manipulability(ur5BodyJacobian(xiK),'invcond'));
            if norm(Jdet) < 0.000000000000000001
                disp('ABORT: Close to singularity');
                finalerr = -1;
                return
            else
                finalerr = norm(error(1:3,4));
                k = k +1;
            end
            % check if the move will hit the boundary 
            check = safetycheck(qk);
            if check == 1
                ur5.move_joints(qk,1)
            else
                return
            end 
        end
    end
    global qk;
end