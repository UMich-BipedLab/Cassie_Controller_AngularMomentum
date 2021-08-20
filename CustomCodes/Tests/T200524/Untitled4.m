tic
for i = 1:1000
    A = rand(5,5);
    if rank(A) == 5
    A_inv = A^(-1);
    end
end
toc

tic
for i = 1:1000
    A = rand(5,5);
    if rank(A) == 5
%     A_inv = YToolkits.QR_inverse(A);
% %     A_inv = inv(A);
    A_inv = A\eye(5);
    end
end

toc

