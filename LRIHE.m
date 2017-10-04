clear
clc

%load('roadmap2_50_offroad.mat')
%load('data3.mat')
load('roadmap3_50.mat')
data = double(traj_hist.data);
load('R_task3.mat')
figure()
pause(3.0)

%Trapping states is that fair?
trap_s = cat(1,find(R_task(:) == 10),find(R_task(:) == -10));
%trap_s = find(R_task(:) == 10);
MDP = mdp(R_task,[1,2,3,4],0.92,trap_s);

S = 392;
N = length(data(1,1,:));
D = 1:(14*28);
T = 75;
disc_vec = zeros(1,T);

for k=1:T
    disc_vec(k) = MDP.gamma^(k-1);
end

M = sum(data(:,1:T,1:N),3); %just up to T=50!
for i=1:T
   %imagesc(reshape(M(:,i),28,14));
   %pause(0.15)
end
init_tracker = M(:,1);
c_opt_set = zeros(N,S);
cnstrs = zeros(S,N);
for k=1:N
    cnstrs(:,k) = data(:,1,k);
    c_opt_set(k,:) = disc_vec*(data(:,1:T,k)');
end

trans_vec = (1/N)*disc_vec*(M');
A = trans_vec;

sopol = {randi(4,28,14)};
M_sub = rollout(MDP,S,sopol{1},N,D,T,init_tracker,false); % Do we want same initial states or not?
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

for k=1:200
    
    %opt_vals = zeros(length(s_tv),1);
    %opt_xs = zeros(n,length(s_tv));
    reps = length(c_set);
%     c_sum = 0;
%     for j=1:reps
%         c_sum = c_sum + s_tv{j};
%     end
    %for j=1:reps
    echo off
    cvx_begin quiet
       variable x(n)
       %variable e(1) %**
       variable g(1)
       maximize( sum((A-B)*x) + g );%- 0.9*norm(x,1));
       %minimize( x'*x + e );
       subject to
            for j=1:reps
                %(trans_vec - s_tv{j})*x >= 1 - e;
                (c_opt_set - c_set{j})*x >= g + 0.1;
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
    
%     if (length(s_tv) > 10)
%         c_set(1) = [];
%     end
    
    %display(k);
    %imagesc(R)
    %pause(0.01)
end
