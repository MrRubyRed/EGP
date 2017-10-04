function [ M,init_tracker,ss ] = rollout(MDP,S,pi,N,D,T,tra,onoff)
%FOLLOUT Summary of this function goes here
%   Detailed explanation goes here
if(tra == -1)
    M = zeros(S,T);
    tracker = zeros(S,1);
    ss = randi(length(D),[1,N]);
    for i=1:N
        tracker(ss(i)) = tracker(ss(i)) + 1; 
    end
else
    tracker = tra;
    ss = 0;
end
init_tracker = tracker;
for i=1:T
    M(:,i) = tracker;
    tracker = Trans(tracker,MDP,pi);
    if(onoff)
        imagesc(reshape(tracker,size(pi)))
        pause(0.15)
    end
end



end

