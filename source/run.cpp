#include <iostream>
#include <map>
#include <Eigen/Dense>
#include <vector>
#include "A_star.h"


using namespace std;

int main() {

    Eigen::Matrix<float,3,1> obj;
    obj(0,0) = 10.2;
    obj(1,0) = 9.6;
    obj(2,0) = 10.2;

    Eigen::Matrix<float,3,1> end;
    end(0) = 9;
    end(1) = 9;
    end(2) = 9;

    Eigen::Matrix<float,3,1> start;
    start(0) = 12;
    start(1) = 10;
    start(2) = 12;

    float step = 0.3;

    star::A_star A(start, end, obj, step);

    return 0;
}