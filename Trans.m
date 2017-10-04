function [ tracker ] = Trans( tracker,MDP,pi )
%TRANS Summary of this function goes here
%   Detailed explanation goes here
tracker = reshape(tracker,size(pi));
temp = zeros(size(tracker));

for i=1:size(pi,1)
    for j=1:size(pi,2)
        if(tracker(i,j) ~= 0)
            act = pi(i,j);
            if(act == 1 && i-1 >= 1)
                if(MDP.T(i+(j-1)*size(pi,1),(i-1)+(j-1)*size(pi,1),act) == 1)
                    temp(i-1,j) = temp(i-1,j) + tracker(i,j);
                else
                    temp(i,j) = temp(i,j) + tracker(i,j);
                end
            elseif(act == 2 && j+1 <= size(pi,2))
                if(MDP.T(i+(j-1)*size(pi,1),i+(j+1-1)*size(pi,1),act) == 1)
                    temp(i,j+1) = temp(i,j+1) + tracker(i,j);
                else
                    temp(i,j) = temp(i,j) + tracker(i,j);
                end
            elseif(act == 3 && i+1 <= size(pi,1))
                if(MDP.T(i+(j-1)*size(pi,1),(i+1)+(j-1)*size(pi,1),act) == 1)
                    temp(i+1,j) = temp(i+1,j) + tracker(i,j);
                else
                    temp(i,j) = temp(i,j) + tracker(i,j);
                end
            elseif(act == 4 && j-1 >= 1)
                if(MDP.T(i+(j-1)*size(pi,1),i+(j-1-1)*size(pi,1),act) == 1)
                    temp(i,j-1) = temp(i,j-1) + tracker(i,j);
                else
                    temp(i,j) = temp(i,j) + tracker(i,j);
                end
            else
                temp(i,j) = temp(i,j) + tracker(i,j);
            end
        end
    end
end

tracker = temp(:);

end

