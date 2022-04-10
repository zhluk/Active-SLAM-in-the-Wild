
mode = 5;
if mode ==1
    % % spars
    L0 = [3 6 3 12 7 8 0 10 15 2 10 15];     % landmarks [x1 y1 x2 y2...]
elseif mode == 2
    % circle
    phi = [0:pi/10:2*pi-pi/10];
    for j = 1:k0
        L0(2*j-1:2*j) = [10*cos(phi(j)) 10*sin(phi(j))+10];
    end
elseif mode == 3
    % % four corners
    L0 = [];
    Lx = [5:5:25, 75:5:95];
    Ly = [5:5:25, 75:5:95];
    % obstacle = [90 80 8 85 15 5]';
    obstacle = [];
    for i = Lx
        for j = Ly
            ob = 0;
            for n = 1:3:length(obstacle)
                delta = [i,j]' - obstacle(n:n+1);
                d = real(sqrt(delta' * delta));
    %             obstacle(n+2)
                if d <= obstacle(n+2)
                    ob = 1;
                end
            end
            if ob == 0
                L0 = [L0 i j];
            end
        end
    end
elseif mode == 4
    % random
    % obstacle = [90 80 8 85 15 5]';
    obstacle = [];
    k = 20;
    L0 = randi([0, 20], 1, 2*k);
elseif mode == 5
    load map1_EKFSLAM
    L0 = State';
%     for j = 1:2:length(L0)
%         L0(j) = L0(j)+200;
%         L0(j+1) = L0(j+1)+120;
%     end
end
%%
obstacle = [];
for j = 1:2:length(L0)
    obstacle = [obstacle' L0(j:j+1) 0]';
end

save map1 L0 obstacle