clear all;

v_in = [0.00
0.00
0.00
0.00
0.00
2.22
4.57
4.58
2.55
0.00
0.00
0.00
0.00
0.00
4.58
0.00
0.00
0.00
1.53
0.27
0.00
0.00
0.00
4.51
0.00
0.00
0.00
4.58
0.00
0.00
0.00
4.57
0.00
0.00
0.00
1.53
0.26
0.00
0.00
0.00
4.51
0.00
0.00
0.00
4.58
0.00
0.00
0.00
4.57
0.00
4.58
4.58
0.00
0.00
4.57
1.73
4.58
4.58
4.58
4.58
1.62
4.58
4.58
4.51
0.00
0.00
4.58
4.58
0.00
0.00
2.55
4.58
0.00
0.00
0.01
4.58
0.26
0.00
0.00
4.58
4.51
0.00
0.00
4.58
4.58
0.00
0.00
2.55
4.57
0.00
0.00
0.02
0.00
0.26
0.00
0.00
4.58
4.51
0.00
0.00
4.58
4.58
4.58
4.58
2.22
4.58
0.00
4.58
4.58
3.60
4.58
4.58
0.00
4.58
4.57
0.00
0.00
4.57
4.58
3.88
0.00
4.58
4.58
4.58
0.00
1.34
4.57
4.57
0.00
0.00
4.58
4.58
0.00
0.00
4.57
4.57
3.89
0.00
4.58
4.57
4.58
0.00
1.34
4.58
4.57
0.00
0.00
4.57
4.57
0.00
0.00
4.59
2.70
0.76
4.58
4.58
4.58
4.58
4.58
4.58
2.55
4.58
4.58
0.40
4.58
4.58
4.58
0.00
4.58
4.58
4.58
0.00
4.58
4.57
4.58
2.69
3.60
4.58
4.58
4.57
0.40
4.58
4.58
4.58
0.00
4.58
4.58
4.58
0.00
4.58
4.58
4.58
2.69
3.60
4.57
4.58
4.57
0.40
4.58
4.57
4.58
4.58
4.59
4.58
4.58
4.58
4.59
4.58
4.58
4.58
4.58
3.86
4.58
4.58
3.15
4.58
4.58
4.58
4.58
4.58
4.58
4.58
4.58
4.58
4.57
4.58
4.58
4.58
4.58
4.57
4.58
3.16
4.58
4.58
4.58
4.57
4.58
4.57
4.58
4.58
4.58
4.57
4.58
4.58
4.58
4.58
4.58
4.58
3.16
4.57
4.58
4.58
4.58
4.58
4.58
4.58
4.58
4.58
4.59
4.58
0.00
4.58
4.58
4.57
4.58
0.00
4.58
4.58
4.58
3.18
2.95
4.57
4.58
4.58
0.09
4.58
4.57
4.58
0.00
4.57
4.58
4.58
0.00
4.58
4.57
4.58
3.18
2.95
4.57
4.57
4.57
0.09
4.58
4.57
4.58
0.00
4.58
4.57
4.58
0.00
0.00
4.58
4.56
4.57
4.58
4.58
4.58
4.58
4.58
3.19
4.58
0.00
0.00
0.96
4.58
4.57
0.00
0.00
4.57
4.58
0.00
0.00
4.58
4.58
4.32
0.00
4.58
4.57
4.58
0.00
0.95
4.57
4.58
0.00
0.00
4.58
4.58
0.00
0.00
4.58
4.57
4.31
0.00
4.58
4.57
4.58
0.00
0.96
4.58
4.57
0.00
4.58
1.95
4.57
0.00
0.00
2.22
4.58
0.00
0.23
2.70
0.00
4.57
4.57
0.00
4.58
4.58
0.00
0.00
1.94
4.57
0.00
0.00
0.00
4.57
4.58
0.00
0.00
4.58
4.56
0.00
0.00
4.58
4.57
0.00
0.00
1.93
4.57
0.00
0.00
0.00
4.58
1.95
0.00
0.00
0.00
4.57
0.00
0.00
4.57
4.57
0.00
2.95
0.01
0.00
4.58
0.00
0.00
0.00
0.00
0.00
0.09
0.03
3.43
0.00
0.00
0.00
4.58
0.00
0.00
0.00
4.57
0.00
0.00
0.00
2.75
0.00
0.00
0.00
0.00
3.43
0.00
0.00
0.00
4.57
0.00
0.00
0.00
4.58
0.00
0.00
0.00
2.75
0.00
0.00
0.00
0.04
3.43
0.00
0.00
];


v = [0.88
0.88
0.89
0.95
0.94
0.96
0.98
1.01
1.05
1.12
1.11
1.14
1.25
1.61
1.88
2.11
2.18
2.26
2.31
2.35
2.40
2.39
2.41
2.42
2.48
2.45
2.45
2.46
2.50
2.47
2.48
2.48
2.52
2.49
2.49
2.49
2.50
2.52
2.50
2.50
2.50
2.54
2.51
2.51
2.51
2.55
2.51
2.51
2.51
2.55
0.97
0.99
0.97
1.00
1.09
1.13
1.14
1.23
1.24
1.29
1.35
1.41
1.59
2.14
2.51
2.69
2.76
2.82
2.89
2.92
2.90
2.92
2.93
2.98
2.95
2.95
2.96
3.01
3.01
2.97
2.98
3.02
3.02
2.98
2.98
3.03
3.03
2.99
3.00
3.00
3.04
3.00
3.00
2.99
3.04
3.04
3.00
3.00
3.04
3.04
0.93
0.98
1.02
1.02
1.12
1.18
1.25
1.28
1.40
1.48
1.55
1.63
1.83
2.49
2.83
3.03
3.07
3.12
3.20
3.22
3.24
3.21
3.26
3.27
3.27
3.23
3.24
3.28
3.28
3.25
3.26
3.29
3.29
3.26
3.26
3.30
3.30
3.30
3.27
3.31
3.31
3.30
3.27
3.27
3.31
3.31
3.31
3.27
3.31
3.31
0.97
0.99
1.04
1.10
1.17
1.25
1.34
1.43
1.53
1.62
1.72
1.80
2.09
2.80
3.08
3.26
3.33
3.37
3.40
3.42
3.43
3.44
3.41
3.46
3.46
3.46
3.46
3.43
3.47
3.47
3.47
3.44
3.48
3.48
3.48
3.44
3.48
3.48
3.48
3.45
3.49
3.48
3.48
3.48
3.45
3.49
3.49
3.48
3.45
3.49
0.97
1.01
1.05
1.13
1.21
1.32
1.42
1.53
1.64
1.74
1.86
1.96
2.28
3.01
3.30
3.43
3.48
3.52
3.54
3.55
3.57
3.58
3.58
3.59
3.59
3.60
3.60
3.60
3.60
3.60
3.60
3.61
3.61
3.61
3.61
3.61
3.61
3.61
3.61
3.61
3.61
3.61
3.61
3.61
3.61
3.62
3.62
3.61
3.62
3.61
0.98
0.97
1.05
1.11
1.17
1.26
1.34
1.44
1.53
1.62
1.68
1.81
2.10
2.76
3.12
3.27
3.34
3.33
3.40
3.42
3.44
3.44
3.41
3.46
3.46
3.46
3.46
3.47
3.47
3.48
3.44
3.47
3.48
3.48
3.45
3.48
3.49
3.48
3.48
3.45
3.49
3.49
3.49
3.45
3.49
3.49
3.49
3.45
3.49
3.49
0.98
0.96
1.03
1.08
1.09
1.19
1.26
1.34
1.41
1.49
1.53
1.64
1.88
2.49
2.84
3.04
3.13
3.14
3.16
3.23
3.25
3.24
3.23
3.27
3.28
3.28
3.25
3.26
3.29
3.30
3.26
3.27
3.30
3.31
3.27
3.27
3.31
3.31
3.29
3.27
3.31
3.31
3.31
3.27
3.29
3.31
3.32
3.27
3.28
3.32
0.98
0.95
1.01
1.05
1.04
1.09
1.18
1.19
1.25
1.30
1.35
1.41
1.64
2.18
2.48
2.66
2.81
2.87
2.87
2.90
2.92
2.97
2.94
2.96
2.96
3.01
3.01
2.98
2.99
3.03
3.03
2.99
2.99
3.04
3.04
3.00
3.00
3.00
3.05
3.00
3.01
3.01
3.05
3.05
3.01
3.01
3.05
3.05
3.01
3.01
0.94
0.94
0.96
0.97
0.99
1.06
1.04
1.08
1.10
1.13
1.16
1.20
1.31
1.67
1.97
2.11
2.22
2.30
2.39
2.38
2.41
2.43
2.49
2.46
2.47
2.48
2.49
2.49
2.50
2.50
2.50
2.54
2.50
2.51
2.51
2.56
2.51
2.52
2.52
2.56
2.52
2.52
2.52
2.53
2.53
2.53
2.53
2.53
2.57
2.53
];

v1 = v(1:50);
v2 = v(51:100);
v3 = v(101:150);
v4 = v(151:200);
v5 = v(201:250);
v6 = v(251:300);
v7 = v(301:350);
v8 = v(351:400);
v9 = v(401:450);

stab1 = mean (v1(30:50));%50
stab2 = mean (v2(30:50));%100
stab3 = mean (v3(30:50));%150
stab4 = mean (v4(30:50));%200
stab5 = mean (v5(30:50));%250
stab6 = mean (v6(30:50));%200
stab7 = mean (v7(30:50));%150
stab8 = mean (v8(30:50));%100
stab9 = mean (v9(30:50));%50

tau(1) = ( stab1-v1(1) )*0.63; %50
tau(2) = ( stab2-v2(1) )*0.63; %100
tau(3) = ( stab3-v3(1) )*0.63; %150
tau(4) = ( stab4-v4(1) )*0.63; %200
tau(5) = ( stab5-v5(1) )*0.63; %250
tau(6) = ( stab6-v6(1) )*0.63; %200
tau(7) = ( stab7-v7(1) )*0.63; %150
tau(8) = ( stab8-v8(1) )*0.63; %100
tau(9) = ( stab9-v9(1) )*0.63; %50

disp(tau);

for i=1:1:450
   x(i) = i; 
end
%plot(x,v_in);
hold on;
plot(x,v);