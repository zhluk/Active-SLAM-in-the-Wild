function u = getU(op, step, range, Le, k, i, x, P_last, P, u, hgoal)
    load paraU flag goal wk wn c range_goal ku
    flag_last = flag;   % indicate last goal point selecting state
    goal_last = goal;   % last goal point
    Le_last = goal_last;
    k_last = ku;        % last number of detected landmarks
    
    pmin = inf;     % initialize lowest landmark uncertainty
    pmax = 0;       % initialize hight landmark uncertainty
    id_good = 1;    % initialize good landmark index
    id_poor = k;    % initialize poor landmark index
    xr = x(2:3);    % robot xy position
    sensedLandmarks = x(4:end);
    landX = sensedLandmarks(1:2:end);
    landY = sensedLandmarks(2:2:end);
    p = polyfit(landX,landY,3);
    x1 = linspace(x(2)-20,x(2)+ 20);
    y1 = polyval(p,x1);
    

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
        lowerbound = upperbound / 2
    
    %     upperbound = wk * (k + 1) + wn*i
    %     lowerbound = wk * (k + 1) + wn*i - c
    %     if k == k_last && num_r == 0
    %         upperbound = w * k + 100 * i
    %         lowerbound = w * k
    %     end
    
        % Explore state
        % set goal point to the closest exploration point
        if trace(P) <= lowerbound
            flag = 1;
            dmin = inf;
          
            if length(Le) ~= 0 
                for j = 1:2:length(Le)
                    delta = x(2:3) - [Le(j),Le(j+1)]'; % replace with furthest detected landmark
                    %d = norm([x(2*j + 1),x(2*j+2)] - xr');
                    d = real(sqrt(delta' * delta));
                    %d = real(sqrt(delta' * delta));
                    if d < dmin
                        dmin = d;
                        %dmax = d;
                        goal_test  = [Le(j),Le(j+1)]';
                        
                    end
                end

                % use curve fitting to get goal point:
                goal_e = [x1(end);y1(end)];
                if i >= 85 
                    goal_e = [x1(1);y1(1)];
                end
                d_e = dmin;
                %d_e = r;
                %d_e = dmax;
                % if the selected exploration point is too far or length(Le)=0
                % set state to 'map'
                
                if d_e > 200
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
        if i >= 70
            a = 1;
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
    elseif op == 8
        
        pos = x(1:3);
        g = G(pos);
        Q = diag([1,1,1,10,10,10,0,0]);
        f = zeros(8,1);
        [A,b] = buildA(pos,goal,g);
        u_opt = quadprog(Q,f,A,b,[],[],[],[]);
        u(:, i) = u_opt(1:3);
    end
end



%% Helper Functions for CLF-CBF Constraints 

function out = G(pos)
    theta = pos(1);
    out = [1 0 0;0 cos(theta) -sin(theta);0 sin(theta) cos(theta)];
end

function out = hg(pos,goal) 
    out = (pos(2) - goal(1))^2 + (pos(3) - goal(2))^2;

end

function out = hgJacobian(pos,goal)
    out = [1,2*(pos(2) - goal(1)), 2*(pos(3) - goal(2))];

end

function out = hs1(pos)
    
    xc = -70;
    yc = 70;
    a = 50^2;
    b = 90^2;
    theta = pi/3;
    x = ((pos(2)-xc)*cos(theta) - (pos(3)-yc)*sin(theta))^2/a;
    y = ((pos(2)-xc)*sin(theta) + (pos(3)-yc)*cos(theta))^2/b;
    out = 1 - x - y;
end

function J = hs1Jacobian(pos)

    theta = pi/3;
    xc = -70;
    yc = 70;
    a = 50^2;
    b = 90^2;
    x1 = 2*((pos(2)-xc)*cos(theta) - (pos(3)-yc)*sin(theta))/a*cos(theta);
    x2 = 2*((pos(2)-xc)*sin(theta) + (pos(3)-yc)*cos(theta))/b*sin(theta);
    x = x1 + x2;
    
    y1 = -2*((pos(2)-xc)*cos(theta) - (pos(3)-yc)*sin(theta))/a*sin(theta);
    y2 = -2*((pos(2)-xc)*sin(theta) + (pos(3)-yc)*cos(theta))/b*cos(theta);
    y = y1 + y2;

    J = [0,-x,-y];
    

end



function out = hs2(pos)
    
    xc = -70;
    yc = 70;
    r = 200^2;
    x = (pos(2)-xc)^2;
    y = (pos(3)-yc)^2;
    out = x + y - r;

end

function J = hs2Jacobian(pos)
    
    xc = -70;
    yc = 70;
    x = 2*(pos(2)-xc);
    y = 2*(pos(3)-yc);
    J = [0,x,y];
    

end

function [A,b] = agentGoal(pos,goal,g)
    
    gamma1 = 1.2;
    gamma2 = 0.8;
    hgOut = hg(pos,goal);
    hgJac = hgJacobian(pos,goal);
    A = [hgJac*g, hgOut, 0,0,(max(0,hgOut))^gamma1, (max(0,hgOut))^gamma2];
    b = 0;

end

function [A,b] = safeSet(pos,g)

    hs1Out = hs1(pos);
    hs2Out = hs2(pos);
    hs1J = hs1Jacobian(pos);
    hs2J = hs2Jacobian(pos);
    A1 = [hs1J*g,0,hs1Out,zeros(1,3)];
    b1 = 0;
    A2 = [hs2J*g,0,0,hs2Out,zeros(1,2)];
    b2 = 0;
    A = [A1;A2];
    b = [b1;b2];

end

function [A,b] = buildA(pos,goal,g)

    [A1,b1] = agentGoal(pos,goal,g);
    [A2,b2] = safeSet(pos,g);
    A3 = zeros(2,8);
    A3(1,end-1) = -1;
    A3(2,end) = -1;
    b3 = -pi/4*ones(2,1);
    A = [A1;A2;A3];
    b = [b1;b2;b3];

end

function [goal,r] = exploration(x,landX,landY)

    r = randi([1,200]);
    t = linspace(0,2*pi,1000);
    ind = randi([1,1000]);
    phi = t(ind);
    goal = [x(2);x(3)] + [r*cos(phi);r*sin(phi)];
    diffs = [landX';landY'] - goal;
    norms = vecnorm(diffs);
    eps = 15;

    while all(norms) > eps
        r = randi([1,100]);
        t = linspace(0,2*pi,2000);
        ind = randi([1,1000]);
        phi = t(ind);
        goal = [x(2);x(3)] + [r*cos(phi);r*sin(phi)];
        diffs = [landX';landY'] - goal;
        norms = vecnorm(diffs);
    end
    

end

function goal = exploration2(x,landX,landY)
    pos = x(2:3);
    mat = [landX,landY];
    numel = length(landX);
    x = datasample(mat,numel);
    dist = [];
    for i = 1:size(x,1)
        y = norm(pos - x(i,:));
        dist = [dist, y];
    end
    
    [~,inds] = max(dist);
    goal = x(inds,:) + sqrt(randi([10,50]))*randn(1,2);

end

