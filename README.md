# Sdnd-Trem2-P1-ExtendedKalmanFilter

This repository is implementation of project from https://github.com/udacity/CarND-Extended-Kalman-Filter-Project

output are stored in src/output-data-1.txt and src/output-data-2.txt

```
$ ./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output-data-1.txt
EKF:
Accuracy - RMSE:
0.0666257
0.0608575
0.555884
0.545138
```

```
$ ./ExtendedKF ../data/sample-laser-radar-measurement-data-2.txt output-data-2.txt
EKF:
CalculateJacobian () - Error - Division by Zero
Hj=
-2.31584e+77            0            0            0
-2.31584e+77            0            0         1000
7.90505e-323            1  1.47701e+12            0
Accuracy - RMSE:
0.186505
0.190536
0.499525
0.800904
```
