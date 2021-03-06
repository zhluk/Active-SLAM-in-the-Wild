function u = getU(op, step, range, Le, k, i, x, P_last, P, u, hgoal)
    load paraU flag goal wk wn c range_goal ku
    flag_last = flag;   % indicate last goal point selecting state
    goal_last = goal;   % last goal point
    k_last = ku;        % last number of detected landmarks
    
    pmin = inf;     % initialize lowest landmark uncertainty
    pmax = 0;       % initialize hight landmark uncertainty
    id_good = 1;    % initialize good landmark index
    id_poor = k;    % initialize poor landmark index
    xr = x(2:3);    % robot xy position

    if flag ~= 0
        num_r = 0;  % not used
        % scan for all detected landmarks
        for j = 1:k
            p = trace(P(3+2*j-1:3+2*j,3+2*j-1:3+2*j));
            delta = x(3 + 2*j - 1:3 + 2*j) - xr;
                    d = real(sqrt(delta' * delta));
            % if landmark uncertainty is higher than pmax and 
            % inside the acceptable returnning distance
            % update pmax and set landmark as return point for funture
            %  'map' state
            if p > pmax && d <= range_goal
                pmax = p;
                id_poor = j;
            end
    
            % if landmark uncertainty is lower than pmin and 
            % inside the acceptable returnning distance
            % update pmin and set landmark as return point for funture
            % 'localization' state
            if p < pmin && d <= range_goal
                pmin = p;
                id_good = j;
            end
            % not used
            if  d <= 20
                num_r = num_r + 1;
            end
        end
        
        delta = goal_last - xr;
        d_last = real(sqrt(delta' * delta));
    
        trace_P = trace(P)
    %     upperbound = w * k + 10
    %     lowerbound = w * k
       
        % state swichting thresholds
        upperbound = wk * k + wn*i
        lowerbound = upperbound /2
%         upperbound = inf
%         lowerbound = inf
    
%         upperbound = wk * (k + 1) + wn*i
%         lowerbound = wk * (k + 1) + wn*i - c
%         if k == k_last && num_r == 0
%             upperbound = w * k + 100 * i
%             lowerbound = w * k
%         end
    
        % Explore state
        % set goal point to the closest exploration point
        if trace(P) <= lowerbound
            flag = 1;
            dmin = inf;
            if length(Le) ~= 0
                for j = 1:length(Le)/2
                    delta = x(2:3) - Le(2*j - 1:2*j)';
                    d = real(sqrt(delta' * delta));
                    if d < dmin
                        dmin = d;
                        goal_e  = Le(2*j - 1:2*j)';
                    end
                end
                d_e = dmin;
                % if the selected exploration point is too far or length(Le)=0
                % set state to 'map'
                if d_e > 100
                    flag = 3;
                end
            else
                flag = 3;
                range_goal = range_goal + 30;
            end
    
        % Switch to improve localization state when trace(P) > upperbound
        elseif trace(P) > upperbound
            flag = 2;
    
        % Switch to improve map when robot is close to the last goal point with
        % in certain range (10 is changable) otherwise keep in current state
        else
            if d_last < 10
                flag = 3;
            else
                flag = 4;
            end
        end
        
        % switch state according to flag and set goal point
        if flag == 1
            state = 'explore'
            goal = goal_e
        elseif flag == 2
            state = 'localization'
            if flag_last == 2
                goal = goal_last
    %             if d_last < 10
    %                 wk = wk + 0.5;
    %             end
            else
                goal = x(3+2*id_good-1:3+2*id_good);
            end
        elseif flag == 3
            state = 'map'
            goal = x(3+2*id_poor-1:3+2*id_poor)
    %         if flag_last == 3 && d_last < 10
    %             wk = wk - 1;
    %         end
        elseif flag == 4
            state = 'keep'
            goal = goal_last
        end
        ku = k;
    %     set(hgoal, 'XData', goal(1), 'YData', goal(2));
        save paraU flag goal wk wn range_goal ku '-append'
    end
    
    % switch to different control logic based on op
    % pause(1);
    if op == 1      % predetermined
       if i < 25
           u(:, i) = [0; 2; 0];
       else
           u(:, i) = [pi/65 2 0]';
       end
    %            turn = [50 100 145 175];
    %            if ismember(i, turn)
    %                u(:, i) = [pi/2, 2, 0];
    % %            elseif ismember(i-1, turn)
    % %                u(:, i) = [-pi/4, 2, 0];
    %            else
    %                u(:, i) = [0 2 0];
    %            end
    %            load u_350 u
    %            u(:, i) = [pi/6 8 0]';
    %            u(:, i) = [0 1 2]';
       
    %             dtheta = pi/50;
    %             rr = 80;
    %             u(:, i) = [dtheta; rr*(sin(i*dtheta)-sin((i-1)*dtheta)); rr*(cos((i-1)*dtheta)-cos(i*dtheta))];
    elseif op == 2      % random
        u(1, i) = unifrnd(0, 2*pi);
        u(2, i) = unifrnd(-2, 2);
        u(3, i) = unifrnd(-2, 2);
    elseif op == 3      % enumeration
    %             u0 = initU();
        horizon = 1;
        actual_h = horizon;
        if step - i < horizon
            actual_h = step - i;
        end
        Pmin = inf;
        save Pmin Pmin
        [~, ~, u0, ~] = initU3(x, P, actual_h, [0;0;0], [], 1);
        u(:, i) = u0(:, 1);
    elseif op == 4      % greedy + random initial value
        actual_h = 1;
        save SLAM_2D_data k actual_h x P '-append'
    %             u0 = randn(3,1);
    %             [u_opt,fmin] = fmincon('objFun2', u0, [],[],[],[],[-pi/4, 1, -2],[pi/4,2,2], 'obs');
    %             u(:, i) = u_opt;
        u0 = randn(2,1);
        [u_opt,fmin] = fmincon('objFun2', u0, [],[],[],[],[-pi/4, 1],[pi/4,2], 'obs');
    %             u(:, i) = u_opt;
        u(:, i) = [u_opt(1); u_opt(2) * cos(u_opt(1)); u_opt(2) * sin(u_opt(1))];
    elseif op == 5      % greedy + enumeration
        actual_h = 1;
        Pmin = inf;
        save Pmin Pmin
        [~, ~, u0, ~] = initU3(x, P, actual_h, [0;0;0], [], 1);
        
        save SLAM_2D_data actual_h '-append'
        [u_opt,fmin] = fmincon('objFun2', u0, [],[],[],[],[-2*pi, -2, -2],[2*pi,2,2]);
        u(:, i) = u_opt;
    elseif op == 6      % multi-step
        horizon = 3;
        actual_h = horizon;
        if step - i < horizon
            actual_h = step - i;
        end
        save SLAM_2D_data k actual_h x P '-append'
        u0 = randn(2,actual_h);
        [u_opt,fmin] = fmincon('objFun2', u0, [],[],[],[],repmat([-pi/4, 1],actual_h,1),repmat([pi/4,2],actual_h,1), 'obs');
    %             u(:, i) = u_opt(:, 1);
        u(:, i) = [u_opt(1, 1); u_opt(2, 1) * cos(u_opt(1, 1)); u_opt(2, 1) * sin(u_opt(1, 1))];
    elseif op == 7      % multi-step + enum
        horizon = 3;
        actual_h = horizon;
        if step - i < horizon
            actual_h = step - i;
        end
        save SLAM_2D_data actual_h '-append'
        Pmin = inf;
        save Pmin Pmin
        [~, ~, u0, ~] = initU3(x, P, actual_h, [0;0], [], 1);
        [u_opt,fmin] = fmincon('objFun2', u0, [],[],[],[],repmat([-2,-2*pi],actual_h,1),repmat([2,2*pi],actual_h,1));
        u(:, i) = u_opt(:, 1);
    end
end
