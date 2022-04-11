% fileID = fopen('rand_IEKF_er_0.txt','r');
% A = fscanf(fileID,'%f');
% error = [];
% for j = 1:1:length(A)
%     error = [error A(j)];
% end
% 
% max(sqrt(error))

fileID = fopen('rand_IEKF_tu_0.txt','r');
B = fscanf(fileID,'%f');
fileID = fopen('rand_IEKF_tp_0.txt','r');
A = fscanf(fileID,'%f');
time = [];
for j = 1:1:length(A)
    time = [time (A(j)+B(j))];
end

sum(time)