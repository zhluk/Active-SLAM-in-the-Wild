load('landmarks.mat')

landmarksX = landmarks(1:2:end);
landmarksY = landmarks(2:2:end);

pos = [0;0;0]; % starting pose


hgSet = [40,-90,-170,-130,-170,-90,0;100,-90,40,175,40,-90,0];
epsilon = 1;
%hgSet = [40;100];

Q = diag([1,1,1,10,10,10,0,0]);
f = zeros(8,1);
dt = 2;
t = 0;

while ~(isempty(hgSet))

    g = G(pos(:,end));
    [A,b] = buildA(pos(:,end),hgSet(:,1),g);
    z = quadprog(Q,f,A,b,[],[],[],[]);
    theta = pos(1,end);
    rot = [cos(theta) -sin(theta);sin(theta) cos(theta)];
    xy = rot*z(2:3)*dt;
    motion = [-z(1)*dt,xy'];
    %motion = [0;z(2);z(3)]*dt;
    posNew = pos(:,end) + motion';
    pos = [pos,posNew];
    
    if norm(pos(2:3,end) - hgSet(:,1)) < epsilon
        hgSet(:,1) = [];
    end
    t = t + dt;
    plot(landmarksX,landmarksY,'+black')
    hold on
    plotQuad()
    plot(pos(2,:),pos(3,:),'b-')
end




%% Functions

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


function plotQuad()

    t = linspace(0,2*pi,1000);
    alpha = pi/3;
    p = [-50 100];
    x = -70 - p(1)*sin(alpha) + 2*p(1).*((cos(t + alpha/2))./(1 - sin(t-alpha/2)));
    y = 70 - p(2)*cos(alpha) + 2*p(2).*((sin(t + alpha/2))./(1 - sin(t-alpha/2)));
    plot(x,y)

end




