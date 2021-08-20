tic
A = zeros(1,20);
for i = 1:1000
    A = circshift(A,1,2);
    A(1) = rand;
end
toc

tic
A = zeros(1,20);
for i = 1:1000
    temp = A(:,2:end);
    A(:,1:end-1) = temp;
    A(1) = rand;
end
toc

