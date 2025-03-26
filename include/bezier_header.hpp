#ifndef BEZIER_HEADER
#define BEZIER_HEADER

#include "class_hierarchy.hpp"
#include <iomanip>
#include <limits>
#include <nlopt.hpp>

typedef struct {
    const Matrix* p0;
    const Matrix* target_point;
    double radius;
    double theta0;
    double target_length;
    double r_min;
} OptData;

double calculateBezierLength(const Matrix& p0, const Matrix& p1, 
    const Matrix& p2, const Matrix& p3, int num_samples);

double evaluateError(double d0, double d1, 
    const Matrix& p0, const Matrix& p3, 
    double theta0, double theta3, 
    double targetLength);

std::pair<double, double> findOptimalParameters(
    const Matrix& p0, const Matrix& p3,
    double theta0, double theta3, 
    double targetLength,
    double d0_min = 10.0, double d0_max = 100.0, double d0_step = 5.0,
    double d1_min = 10.0, double d1_max = 100.0, double d1_step = 5.0);

double calculateCurvatureAtPoint(double t, 
    const Matrix& p0, const Matrix& p1, 
    const Matrix& p2, const Matrix& p3);

double findMaxCurvature(const Matrix& p0, const Matrix& p1, 
    const Matrix& p2, const Matrix& p3, 
    double dt = 0.01);

std::tuple<Matrix, Matrix, Matrix> findOptimalParameters_Circle(
    const Matrix& p0,
    const Matrix& target_point,
    double radius, double theta0, double target_length, double min_radius);

bool outputBezierCurvePoints(
    const Matrix& p0, const Matrix& p1, 
    const Matrix& p2, const Matrix& p3,
    const Matrix& target_point, const double& r,
    const std::string& filename,
    int num_samples = 100);

double objective_function(const std::vector<double> &x, std::vector<double> &grad, void *data);

double constraint_function(const std::vector<double> &x, std::vector<double> &grad, void *data);

void printNLoptResult(nlopt::result result);

std::tuple<Matrix, Matrix, Matrix> findNLoptParameters_Circle(
    const Matrix& p0,
    const Matrix& target_point,
    double radius,
    double theta0,
    double target_length,
    double r_min,
    nlopt::algorithm algo = nlopt::LN_COBYLA);
    
#endif