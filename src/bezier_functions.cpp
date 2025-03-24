#include "bezier_header.hpp"

/** 基于控制点计算贝塞尔曲线长度
 * @note 初步对照MATLAB测试
 */
double calculateBezierLength(const Matrix& p0, const Matrix& p1, 
    const Matrix& p2, const Matrix& p3, int num_samples = 100) 
{
    double length = 0.0;
    double dt = 1.0 / (num_samples - 1);
    double t = 0.0;
    double x_prev = p0[0];
    double y_prev = p0[1];

    // 对曲线进行采样并计算长度
    for (int i = 1; i < num_samples; i++) {
        t += dt;

        // 三次贝塞尔曲线公式
        double B0 = pow(1-t, 3);
        double B1 = 3 * t * pow(1-t, 2);
        double B2 = 3 * t * t * (1-t);
        double B3 = pow(t, 3);

        double x_curr = B0 * p0[0] + B1 * p1[0] + B2 * p2[0] + B3 * p3[0];
        double y_curr = B0 * p0[1] + B1 * p1[1] + B2 * p2[1] + B3 * p3[1];
        double dx = x_curr - x_prev;
        double dy = y_curr - y_prev;
        length += sqrt(dx*dx + dy*dy);
        x_prev = x_curr;
        y_prev = y_curr;
    }

    return length;
}

/** 评估函数 - 计算曲线长度与目标长度的差的绝对值
 * @note 无需对照MATLAB测试
 */
double evaluateError(double d0, double d1, 
    const Matrix& p0, const Matrix& p3, 
    double theta0, double theta3, 
    double targetLength)
{
    Matrix p1(2, 1);
    Matrix p2(2, 1);
    p1.set_vec2(p0[0] + d0 * cos(theta0), p0[1] + d0 * sin(theta0));
    p2.set_vec2(p3[0] - d1 * cos(theta3), p3[1] - d1 * sin(theta3));

    double curveLength = calculateBezierLength(p0, p1, p2, p3, 500); // 使用更高精度

    return fabs(curveLength - targetLength);
}

/** 网格搜索函数 - 在参数空间中进行系统搜索
 * @brief 约束为起点、终点
 */
std::pair<double, double> findOptimalParameters(
    const Matrix& p0, const Matrix& p3,
    double theta0, double theta3, 
    double targetLength,
    double d0_min, double d0_max, double d0_step,
    double d1_min, double d1_max, double d1_step)
{
    double best_d0 = d0_min;
    double best_d1 = d1_min;
    double min_error = std::numeric_limits<double>::max();

    // 粗略
    for (double d0 = d0_min; d0 <= d0_max; d0 += d0_step) {
        for (double d1 = d1_min; d1 <= d1_max; d1 += d1_step) {
            double error = evaluateError(d0, d1, p0, p3, theta0, theta3, targetLength);

            if (error < min_error) {
                min_error = error;
                best_d0 = d0;
                best_d1 = d1;
            }
        }
    }

    // 细化
    double refined_d0_min = std::max(d0_min, best_d0 - d0_step);
    double refined_d0_max = std::min(d0_max, best_d0 + d0_step);
    double refined_d1_min = std::max(d1_min, best_d1 - d1_step);
    double refined_d1_max = std::min(d1_max, best_d1 + d1_step);

    for (double d0 = refined_d0_min; d0 <= refined_d0_max; d0 += d0_step/5) {
        for (double d1 = refined_d1_min; d1 <= refined_d1_max; d1 += d1_step/5) {
            double error = evaluateError(d0, d1, p0, p3, theta0, theta3, targetLength);

            if (error < min_error) {
                min_error = error;
                best_d0 = d0;
                best_d1 = d1;
            }
        }
    }

    std::cout << "最佳参数: d_opt_0 = " << best_d0 
    << ", d_opt_1 = " << best_d1 
    << "，误差 = " << min_error << std::endl;

    return std::make_pair(best_d0, best_d1);
}

/** 计算贝塞尔曲线上某点的曲率（根据t找点）
 * @note 初步对照MATLAB测试
 */
double calculateCurvatureAtPoint(double t, 
    const Matrix& p0, const Matrix& p1, 
    const Matrix& p2, const Matrix& p3)
{
    // 控制点坐标
    double x0 = p0[0], y0 = p0[1];
    double x1 = p1[0], y1 = p1[1];
    double x2 = p2[0], y2 = p2[1];
    double x3 = p3[0], y3 = p3[1];

    // 一阶导数系数
    double dx0 = 3 * (x1 - x0);
    double dx1 = 3 * (x2 - x1);
    double dx2 = 3 * (x3 - x2);
    double dy0 = 3 * (y1 - y0);
    double dy1 = 3 * (y2 - y1);
    double dy2 = 3 * (y3 - y2);

    // 二阶导数系数
    double ddx0 = 6 * (x2 - 2*x1 + x0);
    double ddx1 = 6 * (x3 - 2*x2 + x1);
    double ddy0 = 6 * (y2 - 2*y1 + y0);
    double ddy1 = 6 * (y3 - 2*y2 + y1);

    // 计算一阶导数
    double x_prime = dx0 * pow(1-t, 2) + dx1 * 2*t*(1-t) + dx2 * pow(t, 2);
    double y_prime = dy0 * pow(1-t, 2) + dy1 * 2*t*(1-t) + dy2 * pow(t, 2);

    // 计算二阶导数
    double x_double_prime = ddx0 * (1-t) + ddx1 * t;
    double y_double_prime = ddy0 * (1-t) + ddy1 * t;

    // 计算曲率
    double numerator = fabs(x_prime * y_double_prime - y_prime * x_double_prime);
    double denominator = pow(x_prime * x_prime + y_prime * y_prime, 1.5);

    if (denominator < 1e-10) {
        return 0.0;
    }

    return numerator / denominator;
}

/** 寻找贝塞尔曲线的最大曲率
 * @note 初步对照MATLAB测试
 */
double findMaxCurvature(const Matrix& p0, const Matrix& p1, 
    const Matrix& p2, const Matrix& p3, 
    double dt)
{
    double max_curvature = 0.0;
    double t_at_max = 0.0;
    int num_samples = int(1 / dt);
    
    for (int i = 0; i < num_samples; i++) {
        double t = i * dt;
        double curvature = calculateCurvatureAtPoint(t, p0, p1, p2, p3);

        if (curvature > max_curvature) {
            max_curvature = curvature;
            t_at_max = t;
        }
    }

    // 输出最大曲率点的参数和曲率值
    // std::cout << "最大曲率出现在 t = " << t_at_max 
    // << "，值为 " << max_curvature << std::endl;

    return max_curvature;
}

/**
 * 在目标点固定距离圆上寻找最优终点和控制点参数（带自适应步长）
 * 
 * @param p0 起始点
 * @param target_point 目标点
 * @param radius 目标点所在圆半径
 * @param theta0 起始航向角
 * @param target_length 目标曲线长度
 * @param r_min 最小转弯半径
 * @return std::tuple<Matrix, double, double, double> 最优终点和控制点参数
 */
std::tuple<Matrix, Matrix, Matrix> findOptimalParameters_Circle(
    const Matrix& p0,
    const Matrix& target_point,
    double radius,
    double theta0,
    double target_length,
    double r_min)
{
    // 算法参数
    double angle_min = PI / 2;  // 圆上搜索角度范围最小值 (弧度)
    double angle_max = PI * 3 / 2;  // 圆上搜索角度范围最大值 (弧度)
    double d0_min = 0;  // 第一控制点距离范围最小值
    double d0_max = target_length;  // 第一控制点距离范围最大值
    double d3_min = 0;  // 最后控制点距离范围最小值
    double d3_max = target_length;  // 最后控制点距离范围最大值
    double dt = 0.01;  // 采样步长

    // 中间参数
    double max_curvature = 0;
    double min_error = std::numeric_limits<double>::max();
    Matrix best_p3(2, 1);
    double best_d0 = d0_min;
    double best_d3 = d3_min;
    double best_angle = angle_min;
    
    // 获取目标点坐标
    double target_x = target_point[0];
    double target_y = target_point[1];
    
    // 粗搜索
    double coarse_angle_step = (angle_max - angle_min) / 24;  // 约15度
    double coarse_d0_step = (d0_max - d0_min) / 10;
    double coarse_d3_step = (d3_max - d3_min) / 10;
    for (double angle = angle_min; angle <= angle_max; angle += coarse_angle_step) {
        // 计算当前角度对应的终点位置
        Matrix p3(2, 1);
        p3.set_vec2(target_x + radius * cos(angle), target_y + radius * sin(angle));

        // 计算终点的角度（用于控制点方向）
        double theta3 = angle;
        
        // 对每个终点位置，寻找最优控制点参数
        for (double d0 = d0_min; d0 <= d0_max; d0 += coarse_d0_step) {
            for (double d3 = d3_min; d3 <= d3_max; d3 += coarse_d3_step) {
                // 创建控制点
                Matrix p1(2, 1);
                Matrix p2(2, 1);
                p1.set_vec2(p0[0] + d0 * cos(theta0), p0[1] + d0 * sin(theta0));
                p2.set_vec2(p3[0] + d3 * cos(theta3), p3[1] + d3 * sin(theta3));
                
                max_curvature = findMaxCurvature(p0, p1, p2, p3, dt);  // 计算最大曲率
                if (max_curvature > 1 / r_min) {
                    // cout << "more than r_min, continue" << endl;
                    continue;  // 超过最小转弯半径，跳过
                }

                // 计算曲线长度
                double curve_length = calculateBezierLength(p0, p1, p2, p3, 200); // 粗搜索用较少的采样点
                double error = fabs(curve_length - target_length);
                
                // 如果找到更好的解，更新参数
                if (error < min_error) {
                    min_error = error;
                    best_p3 = p3;
                    best_d0 = d0;
                    best_d3 = d3;
                    best_angle = angle;
                }
            }
        }
    }

    // 细搜索
    double fine_angle_min = std::max(angle_min, best_angle - coarse_angle_step);
    double fine_angle_max = std::min(angle_max, best_angle + coarse_angle_step);
    double fine_d0_min = std::max(d0_min, best_d0 - coarse_d0_step);
    double fine_d0_max = std::min(d0_max, best_d0 + coarse_d0_step);
    double fine_d3_min = std::max(d3_min, best_d3 - coarse_d3_step);
    double fine_d3_max = std::min(d3_max, best_d3 + coarse_d3_step);
    double fine_angle_step = coarse_angle_step / 5;  // 定义细搜索步长 约3度
    double fine_d0_step = coarse_d0_step / 5;
    double fine_d3_step = coarse_d3_step / 5;
    min_error = std::numeric_limits<double>::max(); // 重置最小误差
    
    // 细搜索过程
    for (double angle = fine_angle_min; angle <= fine_angle_max; angle += fine_angle_step) {
        Matrix p3(2, 1);
        p3.set_vec2(target_x + radius * cos(angle), target_y + radius * sin(angle));
        
        double theta3 = angle;
        
        for (double d0 = fine_d0_min; d0 <= fine_d0_max; d0 += fine_d0_step) {
            for (double d3 = fine_d3_min; d3 <= fine_d3_max; d3 += fine_d3_step) {
                Matrix p1(2, 1);
                Matrix p2(2, 1);
                p1.set_vec2(p0[0] + d0 * cos(theta0), p0[1] + d0 * sin(theta0));
                p2.set_vec2(p3[0] + d3 * cos(theta3), p3[1] + d3 * sin(theta3));
                
                max_curvature = findMaxCurvature(p0, p1, p2, p3, dt);  // 计算最大曲率
                if (max_curvature > 1 / r_min) {
                    // cout << "more than r_min, continue" << endl;
                    continue;  // 超过最小转弯半径，跳过
                }

                double curve_length = calculateBezierLength(p0, p1, p2, p3, 500); // 细搜索用更多采样点
                double error = fabs(curve_length - target_length);
                
                if (error < min_error) {
                    min_error = error;
                    best_p3 = p3;
                    best_d0 = d0;
                    best_d3 = d3;
                    best_angle = angle;
                }
            }
        }
    }
    
    Matrix p1(2, 1);
    Matrix p2(2, 1);
    p1.set_vec2(p0[0] + best_d0 * cos(theta0), p0[1] + best_d0 * sin(theta0));
    p2.set_vec2(best_p3[0] + best_d3 * cos(best_angle), best_p3[1] + best_d3 * sin(best_angle));
    

    std::cout << "最终结果: 终点角度 = " << best_angle * 180 / PI << "度, "
              << "终点坐标 = (" << best_p3[0] << ", " << best_p3[1] << "), "
              << "d0 = " << best_d0 << ", d3 = " << best_d3 
              << ", 误差 = " << min_error << std::endl;
    
    return std::make_tuple(p1, p2, best_p3);
}


/**
 * @brief 计算贝塞尔曲线采样点并输出到文件
 * 
 * @param p0 起始控制点
 * @param p1 第一控制点
 * @param p2 第二控制点
 * @param p3 终止控制点
 * @param target_point 目标点
 * @param r 半径
 * @param filename 输出文件名
 * @param num_samples 采样点数量
 * @return bool 是否成功输出
 */
bool outputBezierCurvePoints(
    const Matrix& p0, const Matrix& p1, 
    const Matrix& p2, const Matrix& p3,
    const Matrix& target_point, const double& r,
    const std::string& filename,
    int num_samples)
{
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "错误：无法打开输出文件 " << filename << std::endl;
        return false;
    }
    
    // 写入文件头
    outfile << "# Bezier Curve Points\n";
    outfile << "# t\tx\ty\n";
    
    double dt = 1.0 / (num_samples - 1);
    
    // 计算并输出采样点
    for (int i = 0; i < num_samples; i++) {
        double t = i * dt;
        
        // 贝塞尔公式系数
        double B0 = pow(1-t, 3);
        double B1 = 3 * t * pow(1-t, 2);
        double B2 = 3 * t * t * (1-t);
        double B3 = pow(t, 3);
        
        // 计算点坐标
        double x = B0 * p0[0] + B1 * p1[0] + B2 * p2[0] + B3 * p3[0];
        double y = B0 * p0[1] + B1 * p1[1] + B2 * p2[1] + B3 * p3[1];
        
        double curvature = calculateCurvatureAtPoint(t, p0, p1, p2, p3);
        
        // 输出到文件，使用固定精度
        outfile << std::fixed << std::setprecision(3) << t << "\t"
                << std::setprecision(6) << x << "\t" << y << "\t"
                << curvature << std::endl;
    }
    
    outfile.close();
    std::cout << "贝塞尔曲线点已输出到文件: " << filename << std::endl;
    
    // 单独输出控制点到文件
    std::string control_points_file = filename.substr(0, filename.find_last_of('.')) + "_control_points.txt";
    std::ofstream cp_outfile(control_points_file);
    if (cp_outfile.is_open()) {
        cp_outfile << "# Control Points\n";
        cp_outfile << "# x\ty\tcurvature\n";
        cp_outfile << std::fixed << std::setprecision(6) 
                  << p0[0] << "\t" << p0[1] << std::endl
                  << p1[0] << "\t" << p1[1] << std::endl
                  << p2[0] << "\t" << p2[1] << std::endl
                  << p3[0] << "\t" << p3[1] << std::endl;

        cp_outfile << "# Target Point\n";
        cp_outfile << "# x\ty\n";
        cp_outfile << target_point[0] << "\t" << target_point[1] << std::endl;
        cp_outfile << "# r\n";
        cp_outfile << r << "\t" << 0 << std::endl;
        cp_outfile.close();
        std::cout << "控制点已输出到文件: " << control_points_file << std::endl;
    }

    return true;
}