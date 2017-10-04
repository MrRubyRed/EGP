classdef mdp
    %MDP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        R
        T
        gamma
    end
    
    methods
        function self = mdp(Reward,Actions,gamma,trap_list)
            if nargin > 0
                self.R = Reward;
                self.gamma = gamma;
                num_s = length(Reward(:));
                axis_1 = size(Reward,1);
                num_a = length(Actions(:));
                axis_2 = size(Reward,2);
                
                %Actions - [1 "UP",2 "LEFT",3 "DOWN",4 "RIGHT"]
                self.T = zeros(num_s,num_s,num_a);
                for a=1:num_a
                    for i=1:size(Reward,1)
                        for j=1:size(Reward,2)
                            
                            %b = 1.0;
                            if(any(i + (j-1)*axis_1 == trap_list))
                                %b = 0.0;
                                self.T(i + (j-1)*axis_1,i + (j-1)*axis_1,1) = 1.0;
                                self.T(i + (j-1)*axis_1,i + (j-1)*axis_1,2) = 1.0;
                                self.T(i + (j-1)*axis_1,i + (j-1)*axis_1,3) = 1.0;
                                self.T(i + (j-1)*axis_1,i + (j-1)*axis_1,4) = 1.0;
                            else
                                if(a == 1)
                                    if(i-1 >= 1)
                                        self.T(i + (j-1)*axis_1,(i-1) + (j-1)*axis_1,a) = 1.0;
                                    else
                                        self.T(i + (j-1)*axis_1,i + (j-1)*axis_1,a) = 1.0;
                                    end
                                end
                                if(a == 2)
                                    if(j+1 <= axis_2)
                                        self.T(i + (j-1)*axis_1,i + j*axis_1,a) = 1.0;
                                    else
                                        self.T(i + (j-1)*axis_1,i + (j-1)*axis_1,a) = 1.0;
                                    end
                                end
                                if(a == 3)
                                    if(i+1 <= axis_1)
                                        self.T(i + (j-1)*axis_1,(i+1) + (j-1)*axis_1,a) = 1.0;
                                    else
                                        self.T(i + (j-1)*axis_1,i + (j-1)*axis_1,a) = 1.0;
                                    end
                                end
                                if(a == 4)
                                    if(j-1 >= 1)
                                        self.T(i + (j-1)*axis_1,i + (j-1-1)*axis_1,a) = 1.0;
                                    else
                                        self.T(i + (j-1)*axis_1,i + (j-1)*axis_1,a) = 1.0;
                                    end
                                end
                            end
                        end
                    end
                end
                
            else
                error('Need Reward Matrix. Quitting')
            end
        end
        
        function [old_V,pi] = ValueIteration(self,eps)
            old_V=zeros(size(self.R(:)));
            new_V=old_V;
            R = self.R(:);
            pi = zeros(size(R));
            stopper = ones(size(self.R(:)));
            
            num_a = size(self.T,3);
            temp = zeros(length(R(:)),num_a);
            
            while(max(stopper) >= (eps / (2*self.gamma/(1-self.gamma))))
                %tic
                for i=1:num_a
                    temp(:,i) = R + self.gamma*(self.T(:,:,i)*old_V);
                end
                [new_V,pi] = max(temp,[],2);
                stopper = abs(new_V-old_V);
                old_V = new_V;
                %toc
            end
            
        end
    end
    
end

