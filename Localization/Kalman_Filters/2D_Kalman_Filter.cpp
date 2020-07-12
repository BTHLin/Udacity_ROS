/*
State Prediction:
x' = Fx
P' = FPF^T + Q

Measurement Update:
y = z - Hx' 
S = HP'H^T + R

Calculation of Kalman Gain:
K = P'H^TS^-1

Calculation of Posterior State and Covariance:
x = x' + Ky
P = (I - KH)P'
*/

#include <iostream>
#include <math.h>
#include <tuple>
#include "Core" // Eigen Library
#include "LU"   // Eigen Library

using namespace std;
using namespace Eigen;

float measurements[3] = { 1, 2, 3 };

tuple<MatrixXf, MatrixXf> kalman_filter(MatrixXf x, MatrixXf P, MatrixXf u, MatrixXf F, MatrixXf H, MatrixXf R, MatrixXf I)
{
    for (int n = 0; n < sizeof(measurements) / sizeof(measurements[0]); n++) {
        //****** TODO: Kalman-filter function********//
        
        // Get current measurement
        MatrixXf Z(1,1); Z << measurements[n];         
        
        // Calculate measurement residual
        MatrixXf y(1,1); y << Z - H*x;                 
        
        // Map current state and noise into measurement space
        MatrixXf S(1,1); S << H*P*H.transpose() + R;
        
        // Kalman Gain
        MatrixXf K(2,1); K << P*H.transpose()*S.inverse();
        
        // Calculate Posterior State and Covariance for current time step
        x << x + K*y;
        P << (I-K*H)*P;
        
        // Prediction State and Covariance for next time step
        x << F*x;
        P << F*P*F.transpose();
        
    }

    return make_tuple(x, P);
}

int main()
{

    MatrixXf x(2, 1);// Initial state (location and velocity) 
    x << 0,
    	 0; 
    MatrixXf P(2, 2);//Initial Uncertainty
    P << 100, 0, 
    	 0, 100; 
    MatrixXf u(2, 1);// External Motion
    u << 0,
    	 0; 
    MatrixXf F(2, 2);//Next State Function
    F << 1, 1,
    	 0, 1; 
    MatrixXf H(1, 2);//Measurement Function
    H << 1,
    	 0; 
    MatrixXf R(1, 1); //Measurement Uncertainty
    R << 1;
    MatrixXf I(2, 2);// Identity Matrix
    I << 1, 0,
    	 0, 1; 

    tie(x, P) = kalman_filter(x, P, u, F, H, R, I);
    cout << "x= " << x << endl;
    cout << "P= " << P << endl;

    return 0;
}
