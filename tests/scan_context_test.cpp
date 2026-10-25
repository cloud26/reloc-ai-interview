#include "scan_context.h"
#include <iostream>
#include <random>
#include <stdexcept>
std::pair<double,int> reference(const Eigen::MatrixXf& q, const Eigen::MatrixXf& m) {
    double best = 2; int best_shift = 0;
    for (int s=0; s<q.cols(); ++s) {
        double sum=0; int valid=0, occupied=0;
        for (int c=0; c<q.cols(); ++c) {
            auto a=q.col(c); auto b=m.col((c-s+q.cols())%q.cols());
            double an=a.norm(), bn=b.norm();
            occupied += an>0 || bn>0;
            if (!an || !bn) continue;
            sum += a.dot(b)/(an*bn); ++valid;
        }
        double d=valid && occupied ? 1-(sum/valid)*(double(valid)/occupied) : 1;
        if(d<best) { best=d; best_shift=s; }
    }
    return {best,best_shift};
}
int main() {
    ScanContext sc; sc.rings=8; sc.sectors=16;
    std::mt19937 generator(42);
    for (int trial=0; trial<100; ++trial) {
        Eigen::MatrixXf q=Eigen::MatrixXf::Zero(sc.rings,sc.sectors),m=q;
        for (int c=0;c<sc.sectors;++c) for(int r=0;r<sc.rings;++r) {
            if(generator()%3) q(r,c)=float(generator()%100)/100;
            if(generator()%3) m(r,c)=float(generator()%100)/100;
        }
        if(trial%5==0) q.col(trial%sc.sectors).setZero();
        if(trial%7==0) m.col(trial%sc.sectors).setZero();
        auto a=reference(q,m),b=sc.compare(q,m);
        if(std::abs(a.first-b.first)>1e-6 || a.second!=b.second)
            throw std::runtime_error("Cyclic cosine distance or yaw shift changed");
    }
    auto empty=Eigen::MatrixXf::Zero(sc.rings,sc.sectors).eval();
    if(sc.compare(empty,empty)!=std::make_pair(1.0,0))
        throw std::runtime_error("Empty descriptor must have distance one");
    if(std::abs(sc.yawForShift(4)+M_PI/2)>1e-12)
        throw std::runtime_error("Positive shift must produce negative yaw");
    Eigen::MatrixXf asymmetric=Eigen::MatrixXf::Zero(sc.rings,sc.sectors);
    for(int c=0;c<sc.sectors;++c) asymmetric(c%sc.rings,c)=1+float(c)/sc.sectors;
    Eigen::MatrixXf shifted=asymmetric;
    for(int c=0;c<sc.sectors;++c) shifted.col(c)=asymmetric.col((c-3+sc.sectors)%sc.sectors);
    if(sc.compare(shifted,asymmetric).second!=3)
        throw std::runtime_error("Rotation recovery sign changed");
    std::cout << "Scan Context equivalence and yaw sign passed\n";
}
