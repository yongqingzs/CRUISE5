#include "bezier_header.hpp"

struct initData{
    // only for demo, not API
    initData(
        double point_0_0,
        double point_0_1,
        double target_point_0,
        double target_point_1,
        double r_target,
        double theata_0,
        double target_length,
        double r_min
    ) : point_0(2, 1), target_point(2, 1) {
        point_0 = Matrix(2, 1);
        target_point = Matrix(2, 1);
        point_0.set_vec2(point_0_0, point_0_1);
        target_point.set_vec2(target_point_0, target_point_1);
        this->r_target = r_target;
        this->theata_0 = theata_0;
        this->target_length = target_length;
        this->r_min = r_min;
    };

    Matrix point_0;
    Matrix target_point;
    double r_target;
    double theata_0;
    double target_length;
    double r_min;
};

int demo_circle(
    const Matrix& point_0,
    const Matrix& target_point,
    double r_target,
    double theata_0,
    double target_length,
    double r_min,
    string path
)
{
    /* 测试圆上的最优参数搜索 */
    std::tuple<Matrix, Matrix, Matrix> optimal_parameters = findOptimalParameters_Circle(
        point_0, target_point, r_target, theata_0, target_length, r_min
    );

    Matrix p1 = std::get<0>(optimal_parameters);
    Matrix p2 = std::get<1>(optimal_parameters);
    Matrix p3 = std::get<2>(optimal_parameters);

    // 输出到文件
    outputBezierCurvePoints(point_0, p1, p2, p3, target_point, r_target, path, 1000);

    return 0;
}

int demo_nlopt_circle(
    const Matrix& point_0,
    const Matrix& target_point,
    double r_target,
    double theata_0,
    double target_length,
    double r_min,
    string path
)
{
    /* 测试圆上的最优参数搜索 */
    std::tuple<Matrix, Matrix, Matrix> optimal_parameters = findNLoptParameters_Circle(
        point_0, target_point, r_target, theata_0, target_length, r_min
    );

    Matrix p1 = std::get<0>(optimal_parameters);
    Matrix p2 = std::get<1>(optimal_parameters);
    Matrix p3 = std::get<2>(optimal_parameters);

    // 输出到文件
    outputBezierCurvePoints(point_0, p1, p2, p3, target_point, r_target, path, 1000);

    return 0;

}

int main()
{
    /* 
            x               y               heading(rad)
    c0:     -14922.974681   -13310.281983   0.640792
    c1:     -14920.476178   -9972.019593    0.641107
    c2:     -14919.097693   -6631.636326    0.641872
    tar:    0               0
    RADIUS:8000.000000
    LENGTH:13996.469183
    RAD_MIN:2000.000000
     */
    initData c0(-14922.974681, -13310.281983, 0, 0, 8000, 0.640792, 13996.469183, 2000);
    initData c1(-14920.476178, -9972.019593, 0, 0, 8000, 0.641107, 13996.469183, 2000);
    initData c2(-14919.097693, -6631.636326, 0, 0, 8000, 0.641872, 13996.469183, 2000);

    string point_0 = "../output/bezier_curve_c0.txt";
    string point_1 = "../output/bezier_curve_c1.txt";
    string point_2 = "../output/bezier_curve_c2.txt";

    // "../output/bezier_curve.txt"
    cout << "==========================Search=========================" << endl;
    demo_circle(c0.point_0, c0.target_point, c0.r_target, c0.theata_0, c0.target_length, c0.r_min, point_0);
    demo_circle(c1.point_0, c1.target_point, c1.r_target, c1.theata_0, c1.target_length, c1.r_min, point_1);
    demo_circle(c2.point_0, c2.target_point, c2.r_target, c2.theata_0, c2.target_length, c2.r_min, point_2);
    cout << "==========================NLopt=========================" << endl;
    demo_nlopt_circle(c0.point_0, c0.target_point, c0.r_target, c0.theata_0, c0.target_length, c0.r_min, point_0);
    demo_nlopt_circle(c1.point_0, c1.target_point, c1.r_target, c1.theata_0, c1.target_length, c1.r_min, point_1);
    demo_nlopt_circle(c2.point_0, c2.target_point, c2.r_target, c2.theata_0, c2.target_length, c2.r_min, point_2);
    return 0;
}
