tic

for i = 10000
    a = rand()
    if a == 0
        a = 0.1;
    end
    A = [0,1;0,1];
    pinv(A);
end

toc