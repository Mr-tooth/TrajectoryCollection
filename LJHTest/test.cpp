

#include <GTest/gtest.h>
#include <Eigen/Core>
#include <TrajColl/CubicSpline.h>
#include <iostream>
#include <chrono>

int main()
{
    
    // test for one dimension and one piece
    using Vector1d = Eigen::Matrix<double, 1, 1>;
    std::map<double, Vector1d> points = {{2.,Vector1d(3.)},
                                         {4.,Vector1d(9.)}};
    
    Vector1d start_vel(0.);
    Vector1d end_vel(0.);

    //  setup spline
    TrajColl::CubicSpline<Vector1d> spline1(
        1,TrajColl::BoundaryConstraint<Vector1d>(TrajColl::BoundaryConstraintType::Velocity, start_vel),
        TrajColl::BoundaryConstraint<Vector1d>(TrajColl::BoundaryConstraintType::Velocity, end_vel), points);

    auto tic = std::chrono::high_resolution_clock::now();
    spline1.calcCoeff();
    auto toc = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(toc-tic);

    std::cout<<"Elapsed time: "<<duration.count()<<" microseconds"<<std::endl;

    std::cout<<"The Value of 3.0s in Spline is: "<<
        spline1(3.0) << std::endl;

    std::cout<<"The Value and dValue of 4.0s in Spline is: "<<
        spline1(4.0) << ", "<<spline1.derivative(4.0) << std::endl;
    
    std::cout<<"The Value of 2.0s in Spline is: "<<
        spline1(2.0) << ", "<<spline1.derivative(2.0)<< std::endl;

    std::cout<<"The Value of 2.5s in Spline is: "<<
        spline1(2.50) << ", "<<spline1.derivative(2.50)<<std::endl;

    std::cout<<"The Coeff of the Spline is: ";
    for(int i=0;i<4;i++)
        std::cout<<spline1.getCubicPolyFuncs().lower_bound(3.0)->second->getCoeff()._Elems[i]<<", ";
    // std::cout<<"The Value of 2.0s in Spline is: "<<
    //         spline1(2.0) - points.at(0) << std::endl;
    

    // test for two dimension and two pieces

    std::map<double, Eigen::Vector2d> points2 = {{1.,Eigen::Vector2d( 2.,3.)},
                                                 {3.,Eigen::Vector2d(-1.,9.)},
                                                 {7.,Eigen::Vector2d( 3.,25.)},};
    
    TrajColl::BoundaryConstraint<Eigen::Vector2d> start_acc2(TrajColl::BoundaryConstraintType::Acceleration, Eigen::Vector2d(0.0,0.0));
    TrajColl::BoundaryConstraint<Eigen::Vector2d> end_vel2(TrajColl::BoundaryConstraintType::Velocity, Eigen::Vector2d(1.0,4.0));

    TrajColl::CubicSpline<Eigen::Vector2d> spline2(2, start_acc2, end_vel2, points2);

    tic = std::chrono::high_resolution_clock::now();
    spline2.calcCoeff();
    toc = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(toc-tic);
    std::cout<<"Spline2 Elapsed time: "<<duration.count()<<" microseconds"<<std::endl;

    std::cout<<"The Value of 3.0s in Spline2 is: "<<
        spline2(3.0).transpose() << std::endl;

    std::cout<<"The Value and dValue of 4.0s in Spline2 is: "<<
        spline2(4.0).transpose() << ", "<<spline2.derivative(4.0).transpose() << std::endl;
    
    std::cout<<"The Value of 2.0s in Spline2 is: "<<
        spline2(2.0).transpose() << ", "<<spline2.derivative(2.0).transpose()<< std::endl;

    std::cout<<"The Value of 2.5s in Spline2 is: "<<
        spline2(2.50).transpose() << ", "<<spline2.derivative(2.50).transpose()<<std::endl;

    std::cout<<"The Coeff of the Spline2 is: ";
     for(int i=0;i<4;i++)
         std::cout<<spline2.getCubicPolyFuncs().lower_bound(3.0)->second->getCoeff()._Elems[i].transpose()<<", ";
    return 0;
}