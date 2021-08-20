figure;
plot(Data_perfect.q); 
hold on
plot(Data.q);
plot(Data.stanceLeg,'g-.')
hold off
legend('q true 1','q true 2','q true 3','q true 4','q true 5','q true 6','q true 7','q true 8','q true 9','q true 10','q true 11','q true 12','q true 13','q true 14','q true 15','q true 16','q true 17','q true 18','q true 19','q true 20',...
    'q obs 1','q obs 2','q obs 3','q obs 4','q obs 5','q obs 6','q obs 7','q obs 8','q obs 9','q obs 10','q obs 11','q obs 12','q obs 13','q obs 14','q obs 15','q obs 16','q obs 17','q obs 18','q obs 19','q obs 20')




figure;
plot(Data_perfect.dq); 
hold on
plot(Data.dq);
plot(Data.stanceLeg,'g-.')
hold off
legend('dq true 1','dq true 2','dq true 3','dq true 4','dq true 5','dq true 6','dq true 7','dq true 8','dq true 9','dq true 10','dq true 11','dq true 12','dq true 13','dq true 14','dq true 15','dq true 16','dq true 17','dq true 18','dq true 19','dq true 20',...
    'dq obs 1','dq obs 2','dq obs 3','dq obs 4','dq obs 5','dq obs 6','dq obs 7','dq obs 8','dq obs 9','dq obs 10','dq obs 11','dq obs 12','dq obs 13','dq obs 14','dq obs 15','dq obs 16','dq obs 17','dq obs 18','dq obs 19','dq obs 20')


