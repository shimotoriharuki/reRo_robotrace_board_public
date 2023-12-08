straight = 1000;
adjust_x = 25; % 始まりがちゃんとminになるように調整する定数
adjust_y = -0; % 始まりがちゃんとminになるように調整する定数
gain = 0.45; % 傾きのかわり具合が変わる

facter = straight/adjust_x;
radius = 0:0.1:straight;
max = 6.5;
min = 2.5;
velo = (1 ./ (1 + exp(-(gain/facter)*radius+(adjust_x / 2)*gain))) * (max-min) + min + adjust_y ;

plot(radius, velo)

linear = linspace(min, max, length(radius));
quadratic = 1e-3 .* radius .* radius * ((max - min) / straight) + min;
hold on
plot(radius, linear)
plot(radius, quadratic)
hold off

legend('シグモイド', '1次関数', '2次関数')