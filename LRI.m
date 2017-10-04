clear
clc

load('R_task2.mat')
%figure()
%pause(3.0)
%load('trap_s')
%mdp = gen_grid_mdp(R_task);
trap_s = find(R_task(:) == 10);
%trap_s = [];

MDP = mdp(R_task,[1,2,3,4],0.92,trap_s);
[V,pi] = MDP.ValueIteration(0.00001);
V = reshape(V,size(R_task));
pi = reshape(pi,size(R_task));

S = length(V(:));
N = 400;
D = 1:(392-14);
T = 60;
disc_vec = zeros(1,T);
for k=1:T
    disc_vec(k) = MDP.gamma^(k-1);
end

[M,init_tracker,ss] = rollout(MDP,S,pi,N,D,T,-1,true); 

cnstrs = zeros(S,N);
c_opt_set = zeros(N,S);
for i=1:N
    cnstrs(ss(i),i) = 1;
    M_temp = rollout(MDP,S,pi,N,D,T,cnstrs(:,i),false);
    c_opt_set(i,:) = disc_vec*(M_temp');
end 

trans_vec = (1/N)*disc_vec*(M');
A = trans_vec;

sopol = {randi(4,size(pi))};
M_sub = rollout(MDP,S,sopol{1},N,D,T,init_tracker,true); % Do we want same initial states or not?
c_set = {};
c_sub_set = zeros(N,S);
for i=1:N
    M_temp = rollout(MDP,S,sopol{end},N,D,T,cnstrs(:,i),false);
    c_sub_set(i,:) = disc_vec*(M_temp');
end
c_set{end+1} = c_sub_set;

s_tv = {(1/N)*disc_vec*(M_sub')};
B = s_tv{1};

n = length(trans_vec);
p = @(x) max(x,0) + max(-2*x,0); 

for k=1:150
    
    %opt_vals = zeros(length(s_tv),1);
    %opt_xs = zeros(n,length(s_tv));
    reps = length(s_tv);
%     c_sum = 0;
%     for j=1:reps
%         c_sum = c_sum + s_tv{j};
%     end
    %for j=1:reps
    echo off
    cvx_begin quiet
       variable x(n);
       variable g(1)
       maximize( sum((A-B)*x) + g - 0.1*norm(x,1));
       %maximize( sum((A-B)*x) );%- 0.1*norm(x,1));
       subject to
            for j=1:reps
                %(trans_vec - s_tv{j})*x >= g;
                (c_opt_set - c_set{j})*x >= g;
            end
            %g >= 0.001
            %(c_opt_set - c_set{j})*x >= 0;
            norm(x,2) <= 10;
    cvx_end
    display(cvx_optval/reps)
    %opt_vals(j) = cvx_optval;
    %opt_xs(:,j) = x;
    %echo off
    %end
    
    %[~,find_min] = min(opt_vals);
    %R = opt_xs(:,find_min);
    R = x;
    R = reshape(R,size(R_task));
    
    MDP_n = mdp(R,[1,2,3,4],0.92,trap_s);
    [~,pi_n] = MDP_n.ValueIteration(0.0001);
    %V_n = reshape(V_n,size(R));
    pi_n = reshape(pi_n,size(R));
    
    sopol{end + 1} = pi_n;
    M_n = rollout(MDP_n,S,pi_n,N,D,T,init_tracker,false);
    s_tv{end+1} = (1/N)*disc_vec*(M_n');
    B = [B;s_tv{end}];
    A = [A;trans_vec];
    for i=1:N
        M_temp = rollout(MDP,S,sopol{end},N,D,T,cnstrs(:,i),false);
        c_sub_set(i,:) = disc_vec*(M_temp');
    end
    c_set{end+1} = c_sub_set;

    
    display(k);
    imagesc(reshape(R,size(R_task)))
    pause(0.05)
end
