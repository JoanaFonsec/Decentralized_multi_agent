x = linspace(0,500,500);
x2 = linspace(0,499,499);

figure(1)
subplot(4,2,1)
plot(x,t.x)
xlabel('t');
ylabel('x');
set(gca, 'FontSize', 14)
%title('Target centre x')
hold on
plot(x,t_est.x(:,1))
hold off
axis([0 500 40 80])
legend('Ellipse target','Circle estimate')

subplot(4,2,2)
plot(x,t.y)
xlabel('t');
ylabel('y');
set(gca, 'FontSize', 14)
%title('Target centre y')
hold on
plot(x,t_est.y(:,1))
hold off
axis([0 500 10 80])
legend('Ellipse target','Circle estimate')

subplot(4,2,3:4)
plot(x,abs(the_cosine(:,1)))
xlabel('t');
ylabel('cos(\alpha)');
set(gca, 'FontSize', 14)
axis([0 500 0 1])

subplot(4,2,5)
plot(x,Distance(1:500,1))
xlabel('t');
ylabel('D_1');
set(gca, 'FontSize', 14)
axis([0 500 0 20])

subplot(4,2,6)
plot(x,p{1}(:,5))
xlabel('t');
ylabel('D_{1,2}/D_{1,n}');
set(gca, 'FontSize', 14)
%title('Angle to the closest agent, \beta_1')
hold on
plot(x,ones(1,500))
hold off
axis([0 500 0 2])

subplot(4,2,7)
plot(x,p{1}(:,6))
xlabel('t');
ylabel('u_1.x');
set(gca, 'FontSize', 14)
%title('Control input in x, u_1.x')
axis([0 500 -3 3])

subplot(4,2,8)
plot(x,p{1}(:,7))
xlabel('t');
ylabel('u_1.y');
set(gca, 'FontSize', 14)
%title('Control input in y, u_i.y')
axis([0 500 -3 3])
