clear all;

v_in =[0.00
0.00
0.00
0.00
4.61
0.00
4.61
0.00
0.00
0.00
4.61
0.00
0.00
0.00
0.00
0.00
4.62
4.61
0.00
0.00
0.00
0.00
0.00
0.00
0.00
0.00
4.62
4.62
4.61
0.00
0.00
0.00
0.00
0.00
0.00
0.00
0.00
4.62
4.61
0.00
0.00
0.00
0.00
0.00
0.00
0.00
0.00
0.00
4.62
4.61
4.62
4.61
0.00
4.61
0.00
4.61
0.00
0.00
0.00
4.61
0.00
4.61
4.62
0.00
0.00
0.00
0.00
0.00
0.00
0.00
4.61
4.62
4.61
4.61
0.00
0.00
0.00
0.00
0.00
0.00
4.61
4.61
4.61
4.61
4.61
0.00
0.00
0.00
0.00
0.00
0.00
4.61
4.61
4.61
4.62
0.00
0.00
0.00
0.00
0.00
0.00
0.00
4.62
0.00
4.61
0.00
4.61
0.00
4.61
0.00
4.61
0.00
0.00
0.00
4.61
4.62
4.61
4.61
4.61
4.61
0.00
0.00
0.00
0.00
4.62
4.61
4.62
4.61
4.61
4.62
4.61
0.00
0.00
0.00
0.00
4.61
4.62
4.61
4.62
4.61
4.61
0.00
0.00
0.00
0.00
0.00
4.62
4.61
4.61
4.62
4.61
4.61
4.61
4.61
4.62
4.61
4.61
4.61
4.61
0.00
4.62
0.00
4.61
4.61
4.61
0.00
0.00
0.00
4.61
4.61
4.61
4.62
4.61
4.62
4.61
4.61
0.00
0.00
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
0.00
0.00
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
0.00
0.00
0.00
0.00
4.61
4.61
4.62
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.62
4.62
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.62
4.61
4.61
0.00
4.62
0.00
4.61
0.00
4.61
0.00
4.61
0.00
4.61
0.00
0.00
4.61
4.62
4.61
4.61
4.61
4.61
4.61
4.61
4.61
0.00
0.00
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.62
4.61
0.00
0.00
4.61
4.61
4.61
4.61
4.61
4.61
4.61
4.61
0.00
0.00
4.61
4.61
4.61
0.00
4.61
0.00
4.61
0.00
4.61
0.00
4.61
0.00
4.61
0.00
4.61
4.62
4.62
4.61
0.00
0.00
0.00
0.00
0.00
4.61
4.61
4.62
4.61
4.61
4.61
0.00
0.00
0.00
0.00
4.61
4.61
4.61
4.61
4.61
4.61
4.61
0.00
0.00
0.00
0.00
4.61
4.61
4.61
4.61
4.61
4.61
0.00
0.00
0.00
0.00
0.00
0.00
0.00
4.61
0.00
4.61
0.00
0.00
0.00
4.61
0.00
0.00
0.00
4.61
4.61
4.62
4.61
0.00
0.00
0.00
0.00
0.00
0.00
4.61
4.61
4.61
4.61
0.00
0.00
0.00
0.00
0.00
0.00
0.00
4.61
4.61
4.62
4.61
0.00
0.00
0.00
0.00
0.00
0.00
0.00
4.61
4.61
4.61
4.61
4.61
0.00
4.61
0.00
4.61
0.00
4.62
0.00
4.62
0.00
4.62
0.00
0.00
0.00
0.00
0.00
0.00
0.00
4.61
4.61
0.00
0.00
0.00
0.00
0.00
0.00
0.00
0.00
4.62
4.61
0.00
0.00
0.00
0.00
0.00
0.00
0.00
0.00
0.00
4.61
4.61
0.00
0.00
0.00
0.00
0.00
0.00
0.00
0.00
0.00
];



v = [0.38
0.38
0.54
0.39
0.41
0.42
0.44
0.47
0.49
0.52
0.55
0.58
0.68
1.07
1.40
1.66
1.83
2.11
2.20
2.11
2.15
2.18
2.21
2.23
2.25
2.26
2.28
2.44
2.46
2.46
2.31
2.32
2.32
2.33
2.33
2.34
2.34
2.34
2.50
2.50
2.35
2.35
2.35
2.36
2.36
2.36
2.37
2.36
2.36
2.53
0.68
0.71
0.59
0.78
0.68
0.88
0.77
0.82
0.87
0.93
0.98
1.05
1.39
1.98
2.22
2.44
2.57
2.65
2.70
2.73
2.76
2.94
2.95
2.96
2.97
2.83
2.83
2.83
2.84
2.84
2.85
3.01
3.01
3.02
3.02
3.02
2.87
2.87
2.87
2.87
2.87
2.87
3.03
3.03
3.03
3.03
2.88
2.88
2.88
2.88
0.54
0.55
0.71
0.58
0.78
0.67
0.89
0.95
1.03
1.09
1.17
1.25
1.35
2.11
2.71
2.94
3.06
3.13
3.17
3.20
3.06
3.07
3.08
3.09
3.10
3.27
3.27
3.27
3.28
3.28
3.28
3.13
3.13
3.13
3.13
3.29
3.30
3.30
3.30
3.30
3.30
3.30
3.14
3.14
3.14
3.15
3.30
3.31
3.31
3.31
0.70
0.72
0.76
0.81
0.88
0.96
1.04
1.13
1.22
1.32
1.41
1.52
1.81
2.62
3.02
3.20
3.13
3.18
3.37
3.40
3.41
3.42
3.43
3.44
3.45
3.45
3.45
3.29
3.30
3.46
3.46
3.47
3.47
3.47
3.47
3.47
3.47
3.31
3.31
3.32
3.47
3.48
3.48
3.48
3.48
3.48
3.48
3.47
3.32
3.32
0.70
0.72
0.76
0.81
0.88
0.96
1.06
1.16
1.27
1.38
1.49
1.60
1.94
2.82
3.20
3.36
3.44
3.48
3.51
3.53
3.54
3.55
3.56
3.56
3.57
3.57
3.58
3.58
3.58
3.58
3.58
3.59
3.59
3.59
3.59
3.59
3.59
3.59
3.59
3.60
3.59
3.59
3.59
3.44
3.60
3.60
3.60
3.60
3.60
3.60
0.70
0.73
0.77
0.69
0.91
0.83
1.06
1.01
1.25
1.19
1.44
1.38
1.85
2.49
2.87
3.05
3.30
3.35
3.38
3.40
3.42
3.43
3.44
3.44
3.28
3.29
3.46
3.46
3.46
3.47
3.46
3.47
3.47
3.47
3.47
3.31
3.32
3.48
3.48
3.48
3.48
3.48
3.48
3.48
3.48
3.32
3.32
3.32
3.48
3.49
0.70
0.73
0.61
0.81
0.71
0.93
0.84
1.06
1.13
1.22
1.29
1.22
1.62
2.35
2.76
2.82
2.92
2.98
3.03
3.06
3.23
3.25
3.26
3.27
3.27
3.27
3.12
3.12
3.13
3.13
3.29
3.29
3.30
3.30
3.30
3.30
3.30
3.15
3.15
3.14
3.15
3.31
3.31
3.31
3.31
3.31
3.31
3.16
3.15
3.15
0.55
0.54
0.57
0.74
0.62
0.81
0.71
0.90
0.80
1.00
0.90
1.11
1.15
1.92
2.34
2.58
2.73
2.66
2.71
2.75
2.78
2.80
2.81
2.82
2.99
3.00
3.01
3.01
2.86
2.86
2.86
2.87
2.87
2.87
2.88
3.04
3.04
3.04
3.04
2.88
2.88
2.88
2.88
2.89
2.88
3.05
3.05
3.05
3.05
2.89
0.55
0.55
0.70
0.56
0.73
0.59
0.62
0.65
0.67
0.70
0.73
0.76
0.87
1.26
1.58
1.81
1.96
2.22
2.30
2.20
2.23
2.26
2.29
2.30
2.31
2.32
2.33
2.34
2.50
2.51
2.37
2.37
2.37
2.37
2.37
2.38
2.38
2.38
2.38
2.54
2.55
2.39
2.39
2.39
2.40
2.40
2.40
2.40
2.40
2.55
];


for i=1:1:450
   x(i) = i; 
end
plot(x,v_in);
hold on;
plot(x,v);