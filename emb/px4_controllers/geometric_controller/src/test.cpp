#include <iostream>
#include <StdVector>
#include <eigen3/Eigen/
using namespace std;

int main() {
    int A[] = {1, 2, 4, 6, 7};
    for(auto a: A)
    cout<<a<<endl;
    // cout<<"Hello World!" << a <<endl;
    vector<Eigen::Vector3d> a;
    Eigen::Vector3d b;
    b(0) = 1;
    b(1) = 2;
    b(2) = 3;
    
    return 0;
}