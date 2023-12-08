clear 

r_max = 1000;
radius = 0:0.1:r_max;
v_max = 7;
v_min = 2.5;

r1 = 400;
r2 = 800;

ratio1 = 0.4;
ratio2 = 0.9;

a = (v_max - v_min) / r_max;

v1 = vr(a, r1, 0, v_min);
vv1 = (v1-v_min) * ratio1 + v_min;
a1 = (vv1 - v_min) / r1;

v2 = vr(a, r2, 0, v_min);
vv2 = (v2-v_min) * ratio2 + v_min;
a2 = (vv2 - vv1) / (r2 - r1);

v3 = v_max;
vv3 = v_max;
a3 = (vv3 - vv2) / (r_max - r2);

velo = zeros(1, length(radius));
velo1 = zeros(1, length(radius));
velo2 = zeros(1, length(radius));

%1次関数と2次関数と条件別1次関数
for i = 1 : length(radius)
    if radius(i) < r1
        velo(i) = vr(a1, radius(i), 0, v_min);   
    elseif radius(i) < r2
        velo(i) = vr(a2, radius(i), r1, vv1); 
    else
        velo(i) = vr(a3, radius(i), r2, vv2);
    end

    velo1(i) = vr(a, radius(i), 0, v_min);
    velo2(i) = 1e-3 * radius(i) * radius(i) * ((v_max - v_min) / r_max) + v_min;
end

% シグモイド
adjust_x = 25; % 始まりがちゃんとminになるように調整する定数
adjust_y = 0; % 始まりがちゃんとminになるように調整する定数
gain = 0.30; % 傾きのかわり具合が変わる

facter = r_max/adjust_x;
max = 6.5;
min = 2.5;
velo_sigmoid = (1 ./ (1 + exp(-(gain/facter)*radius+(adjust_x / 2)*gain))) * (max-min) + min + adjust_y ;

%多項式
n = 4;

x0 = [0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000];
y0 = [2.5, 2.5, 2.8, 3.0, 3.3, 4.3, 4.5, 5.0, 5.5, 6.3, 7];

p = polyfit(x0, y0, n);
velo_poly = polyval(p, radius);

%プロット
plot(radius, velo, radius, velo1, radius, velo2, radius, velo_sigmoid, radius, velo_poly);
legend("条件別1次関数", "1次関数", "2次関数", "シグモイド関数", "多項式")

hold on
plot(x0, y0, 'o');
hold off

% 1次関数の関数
function v = vr(a, r, r_shift, b)
    v = a * (r - r_shift) + b;
end
