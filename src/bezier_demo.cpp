#include "bezier_header.hpp"

int demo_circle()
{
    /* 测试圆上的最优参数搜索 */
    Matrix point_0(2, 1);
    point_0.set_vec2(100, -200);
    double theata_0 = 30 * RAD;
    
    Matrix target_point(2, 1);
    target_point.set_vec2(1000, 500);
    double r_target = 300;

    cout << "point_0:\t" << point_0[0] << "\t" << point_0[1] << endl;
    cout << "target_point:\t" << target_point[0] << "\t" << target_point[1] << endl;

    double target_length = 1200;
    double r_min = 150;
    
    std::tuple<Matrix, Matrix, Matrix> optimal_parameters = findOptimalParameters_Circle(
        point_0, target_point, r_target, theata_0, target_length, r_min
    );

    Matrix p1 = std::get<0>(optimal_parameters);
    Matrix p2 = std::get<1>(optimal_parameters);
    Matrix p3 = std::get<2>(optimal_parameters);

    // 输出到文件
    outputBezierCurvePoints(point_0, p1, p2, p3, target_point, r_target, "../output/bezier_curve.txt", 1000);

    return 0;
}

int main()
{
    // demo0();
    // demo1();
    // testMaxCurvature();
    demo_circle();
    return 0;
}
