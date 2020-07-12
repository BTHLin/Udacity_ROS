/*
Every execution of a motion will result in a new state distribution.
Combine it with current best estimated state results in the next time step state estimation. 
simply add the means and variances of two distribution for new mean and variance. 
N*(x: mu1+mu2, s1+s2)
*/

#include <iostream>
#include <math.h>
#include <tuple>

using namespace std;

double new_mean, new_var;

tuple<double, double> state_prediction(double mean1, double var1, double mean2, double var2)
{
    new_mean = mean1+mean2;
    new_var =  var1+var2;
    return make_tuple(new_mean, new_var);
}

int main()
{

    tie(new_mean, new_var) = state_prediction(10, 4, 12, 4);
    printf("[%f, %f]", new_mean, new_var);
    return 0;
}
