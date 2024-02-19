#include <vector>
#include <mutex>
#include <eigen3/Eigen/Dense>

namespace PolynomialSolver
{
    enum Exception { InitialVecSizeError, PredictionNotAvailable };

    /*
    PolynomialSolver use least squares method to find n-degrees polynomial coefficients, and predict a result from given value (1d only)
    */
    class PolynomialSolver
    {
    private:
        unsigned int deg_;// Degree
        Eigen::VectorXd coeff_;// Coefficients
        Eigen::VectorXd var_;// Preserve variable vector to prevent frequently allocating when predict() called
        bool predF_;

        std::mutex predictLock_;

    public:
        /*
        xVec and yVec represents input and observed data sets. The two vector size must be the same.
        degree: approximate polynomial degree. Ex: degree=3, polynomial function: ax^3+bx^2+cx+d=y
        */
        PolynomialSolver(std::vector<double> xVec, std::vector<double> yVec, unsigned int degree) : predF_(false)
        {
            if (xVec.size() != yVec.size() || xVec.size() <= 0)
                throw Exception::InitialVecSizeError;
            size_t n = xVec.size();

            this->deg_ = degree;
            this->coeff_ = Eigen::VectorXd(degree + 1);
            this->var_ = Eigen::VectorXd(degree + 1);

            Eigen::Map<Eigen::VectorXd> col(xVec.data(), n);
            Eigen::Map<Eigen::VectorXd> b(yVec.data(), n);
            Eigen::MatrixXd A(n, degree + 1);
            for (size_t i = 0; i < degree + 1; i++)
                A.block(0, i, n, 1) = col.array().pow(degree - i);
            auto ATA = A.transpose() * A;
            this->coeff_ = ATA.inverse() * A.transpose() * b;
            this->predF_ = true;
        }

        /*
        Predict a result from given value (1d only)
        val: input value
        return: predicted value from polynomial
        */
        double predict(double val)
        {
            if (!this->predF_)
                throw Exception::PredictionNotAvailable;
            
            std::lock_guard<std::mutex> locker(this->predictLock_);// For thread safe

            for (int i = 0; i < this->deg_ - 1; i++)
                this->var_(i) = std::pow(val, this->deg_ - i);
            this->var_(this->deg_ - 1) = val;
            this->var_(this->deg_) = 1;
            return this->coeff_.dot(this->var_);
        }
    };
}