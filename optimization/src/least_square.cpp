#include <vector>
#include <random>
#include <iostream>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;

Vector least_square(Matrix X, Vector y)
{
    // X is a matrix of size (n, m)
    // y is a vector of size (n, 1)
    // (X.T * X)^-1 * X.T * y
    auto ans = (X.transpose() * X).inverse() * X.transpose() * y;
    return ans;
}

int main(int argc, char** argv)
{
    float a, b, c;
    a = 1.0;
    b = 2.0;
    c = 3.0;
    for(int i = 0; i < argc; i++)
    {
        if (std::string(argv[i]) == "-a")
            a = std::stof(argv[i+1]);
        if (std::string(argv[i]) == "-b")
            b = std::stof(argv[i+1]);
        if (std::string(argv[i]) == "-c")
            c = std::stof(argv[i+1]);
    }
    std::random_device gen;
    std::default_random_engine engine(gen());
    auto gaussian_noise = std::normal_distribution<float>(0, 0.2);

    std::vector<float> x_vec, y_vec;
    for (int i = 0; i < 1000; i++)
    {
        float x = 0.01 * i;
        float noise = gaussian_noise(engine);
        float y = a*x*x + b*x + c + noise;
        x_vec.push_back(x);
        y_vec.push_back(y); 
    }

    Matrix X(x_vec.size(), 3);
    Vector y(y_vec.size());
    for (int i = 0; i < x_vec.size(); i++)
    {
        X(i, 0) = x_vec[i]*x_vec[i];
        X(i, 1) = x_vec[i];
        X(i, 2) = 1.0;
        y(i) = y_vec[i];
    }

    auto ans = least_square(X, y);
    std::cout << "y = " << ans(0) << "x^2 + " << ans(1) << "x + " << ans(2) << "\n";

    FILE *gp;
    //gp = popen("gnuplot -persist", "w");
    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "unset key\n");
    fprintf(gp, "set grid\n");
    fprintf(gp, "plot '-' with points pt 7 ps 0.3, ");
    fprintf(gp, "%f*x*x+%f*x+%f \n", ans(0), ans(1), ans(2));
    std::ostream ss;
    ss << ans(0) << "x*x + " << ans(1) << "x + " << ans(2);
    
    for(int i = 0; i < x_vec.size(); i++)
        fprintf(gp, "%f %f\n", x_vec[i], y_vec[i]);
    fprintf(gp, "e\n");
    fflush(gp);

    std::cout << "Press enter to continue...";
    std::cin.get();
    pclose(gp);

    return 0;
}