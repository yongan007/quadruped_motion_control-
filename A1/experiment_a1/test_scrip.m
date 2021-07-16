% Generate data and set up the axes
x = linspace(0, 2*pi, 500);
axis([0 2*pi -8 8]);
hold on
% Plot 8 sine curves, naming each n*sin(n*x) for legend purposes
for n = 1:8
  plot(x, n.*sin(n*x), 'DisplayName', sprintf('%d*sin(%d*x)', n, n));
end
% Turn on the legend
L = legend('show');
% Make it 2-by-4 instead of 8-by-1
L.NumColumns = 4;

h(1) = plot(1:10,sin(1:10),'r');
hold on
h(2) = plot(1:10,cos(1:10),'r');
h(3) = plot(1:10,cos(1:10) + sin(1:10),'b');
legend(h([1,3]),{'data1','data2'})