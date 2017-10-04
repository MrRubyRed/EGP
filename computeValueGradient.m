
[height, width] = size(V);
quivX = zeros(height, width);
quivY = zeros(height, width);

Vgrad = zeros(height, width, 2);
for i = 1:height
    for j = 1:width
        gradSumX = 0;
        gradSumY = 0;
   
        % Look up
        if i ~= 1
            gradSumY = gradSumY + (V(i-1, j) - V(i, j));
        end
        % Look left
        if j < width
            gradSumX = gradSumX + (V(i, j+1) - V(i, j));
        end
        % Look down
        if i ~= height
            gradSumY = gradSumY - (V(i+1, j) - V(i, j));
        end
        % Look right
        if j ~= 1
            gradSumX = gradSumX - (V(i, j-1) - V(i, j));
        end
        
        Vgrad(i, j, 1) = gradSumX / 2;
        Vgrad(i, j, 2) = gradSumY / 2;
        
        quivX(i, j) = j-0.5;
        quivY(i, j) = height - i + 0.5;
    end
end

valueGradient = reshape(Vgrad, height*width, 2);