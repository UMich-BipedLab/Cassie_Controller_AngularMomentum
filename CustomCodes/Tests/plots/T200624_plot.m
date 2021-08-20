figure;
plot(left_toe_back_force.Friction_Force + left_toe_front_force.Friction_Force)
hold on
plot(right_toe_back_force.Friction_Force + right_toe_front_force.Friction_Force)
plot(100*Data.stanceLeg,'g-.')
hold off
ax1 = gca;

figure;
plot(right_toe_front_force.Friction_Force)
ax2 = gca;

figure;
plot(right_toe_back_force.Friction_Force)
ax3 = gca;

figure;
plot(right_toe_front_force.Friction_Force)
ax4 = gca;
