#include <iostream>
#if __has_include(<coroutine>)
#include <coroutine>
#else
#include <experimental/coroutine>
#define USE_EXPERIMENTAL_COROUTINE
#endif
#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>
#include <vector>
#include <deque>
#include <Eigen/Core>   // Eigen 3.4>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include "ptgl/Core/QuickGraphicsView.h"
#ifdef EMSCRIPTEN
#include <emscripten.h>
#include "ptgl/Driver/EMGLUTGraphicsDriver.h"
#else   // EMSCRIPTEN
#include "ptgl/Driver/GLFWGraphicsDriver.h"
#endif  // EMSCRIPTEN
#include "QuadProg++Eigen.hh"

// Implemented up to [1]-(16).
// [1] A. Herdt, H. Diedam, P.-B. Wieber, D. Dimitrov, K. Mombaur and M. Diehl: “Online walking motion generation with automatic foot step placement,” Advanced Robotics, vol.24, no.5–6, pp.719–737, 2010.
// [2] Herdt,A.(2012).Modelpredictivecontrolofahumanoid robot.Ph.D.thesis,EcoleNationaleSup ́erieuredes MinesdeParis.

// https://cpprefjp.github.io/lang/cpp20/coroutines.html
#ifdef USE_EXPERIMENTAL_COROUTINE
namespace std_coro = std::experimental;
#else   // USE_EXPERIMENTAL_COROUTINE
namespace std_coro = std;
#endif  // USE_EXPERIMENTAL_COROUTINE
template <typename T>
struct generator {
  struct promise_type;
//  using handle = std::coroutine_handle<promise_type>;
  using handle = std_coro::coroutine_handle<promise_type>;
  struct promise_type {
    T value_;
    static auto get_return_object_on_allocation_failure() { return generator{nullptr}; }
    auto get_return_object() { return generator{handle::from_promise(*this)}; }
    auto initial_suspend() { return std_coro::suspend_always{}; }
    auto final_suspend() noexcept { return std_coro::suspend_always{}; }
    void unhandled_exception() { std::terminate(); }
    void return_void() {}
    auto yield_value(T value) {
      value_ = value;
      return std_coro::suspend_always{};
    }
  };
  bool move_next() { return coro ? (coro.resume(), !coro.done()) : false; }
  T current_value() { return coro.promise().value_; }
  generator(generator const&) = delete;
  generator(generator && rhs) : coro(rhs.coro) { rhs.coro = nullptr; }
  ~generator() { if (coro) coro.destroy(); }
private:
  generator(handle h) : coro(h) {}
  handle coro;
};

struct FootStep {
    enum SupportState {
        DoubleSupport,
        LeftSupport,
        RightSupport,
    };
    uint64_t time;
    SupportState supportState;

    Eigen::Vector3d leftPos;
    Eigen::Vector3d rightPos;

    Eigen::Matrix3d leftRot;
    Eigen::Matrix3d rightRot;

    Eigen::Vector3d zmpRef;
};

struct FootStepInstruction
{
    bool stop;
    Eigen::Vector3d stride;
    Eigen::Vector3d direction;
    uint64_t Tssp;
    uint64_t Tdsp;
    double swingFootHeight;
};

struct PredictiveState {
    uint64_t time;  //    ms

    Eigen::Vector3d zmpRef;

    // a * x + b * y >= c
    std::vector<double> a;
    std::vector<double> b;
    std::vector<double> c;

    std::shared_ptr<FootStep> footStep;
};

bool compare(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    return (a.x() < b.x()) ? a.x() < b.x() : (a.x() == b.x() ? a.y() < b.y() : false);
}

double cross(const Eigen::Vector3d& p_O, const Eigen::Vector3d& p_A, const Eigen::Vector3d& p_B)
{
    return ((p_A.x() - p_O.x()) * (p_B.y() - p_O.y()) - (p_A.y() - p_O.y()) * (p_B.x() - p_O.x()));
}

// https://github.com/patrikpihlstrom/Monotone-Chain-Convex-Hull (The Unlicense)
std::vector<Eigen::Vector3d> calcConvexHull(std::vector<Eigen::Vector3d> points)
{
    int n = points.size();
    int k = 0;

    std::vector<Eigen::Vector3d> H(2*n);
    std::sort(points.begin(), points.end(), compare);

    // lower
    for (int i = 0; i < n; ++i) {
        while(k >= 2 && cross(H[k-2], H[k-1], points[i]) <= 0) {
            k--;
        }
        H[k++] = points[i];
    }

    // upper
    for (int i = n-2, t = k+1; i >= 0; i--) {
        while (k >= t && cross(H[k-2], H[k-1], points[i]) <= 0) {
            k--;
        }
        H[k++] = points[i];
    }

    H.resize(k);
    return H;
}

// a * x + b * y >= c
size_t calcConvexHullHalfSpace(const std::vector<Eigen::Vector3d>& ps,
        std::vector<double>& a, std::vector<double>& b, std::vector<double>& c)
{
    size_t size = ps.size();
    a.resize(size);
    b.resize(size);
    c.resize(size);

    for (size_t i = 0; i < size; ++i) {
        const Eigen::Vector3d& curr = ps[i];
        const Eigen::Vector3d& next = ps[(i+1)%size];
        double x1 = curr(0);
        double x2 = next(0);
        double y1 = curr(1);
        double y2 = next(1);
        a[i] = -(y2 - y1);
        b[i] = x2 - x1;
        c[i] = a[i]*x1 + b[i]*y1;
    }
    return size;
}

Eigen::Vector3d calcCentorOfPolygon(const std::vector<Eigen::Vector3d>& points)
{
    if (points.empty()) {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d c(Eigen::Vector3d::Zero());
    for (const auto& p: points) {
        c += p;
    }
    return c / points.size();
}

class LegBody {
public:
    std::vector<Eigen::Vector3d> localLeftFootPolygon;
    std::vector<Eigen::Vector3d> localRightFootPolygon;

    Eigen::Vector3d com;
    Eigen::Vector3d zmp;

    std::vector<Eigen::Vector3d> leftFootPolygon;
    std::vector<Eigen::Vector3d> rightFootPolygon;
    std::vector<Eigen::Vector3d> supportFootPolygon;

    void updateFootPolygon(const FootStep& footStep) {
        leftFootPolygon.clear();
        rightFootPolygon.clear();

        for (auto&& fp : localLeftFootPolygon) {
            Eigen::Vector3d p =  footStep.leftPos + footStep.leftRot * fp;
            leftFootPolygon.push_back(p);
        }

        for (auto&& fp : localRightFootPolygon) {
            Eigen::Vector3d p =  footStep.rightPos + footStep.rightRot * fp;
            rightFootPolygon.push_back(p);
        }

        if (footStep.supportState == FootStep::LeftSupport) {
            supportFootPolygon = leftFootPolygon;
        } else if (footStep.supportState == FootStep::RightSupport) {
            supportFootPolygon = rightFootPolygon;
        } else {
            supportFootPolygon = leftFootPolygon;
            supportFootPolygon.insert(supportFootPolygon.end(), rightFootPolygon.begin(), rightFootPolygon.end());
        }

        supportFootPolygon = calcConvexHull(supportFootPolygon);
    }
};

generator<FootStep> generateFootStep(std::shared_ptr<FootStepInstruction> instruction, const Eigen::Vector3d& localLeftFootPos, const Eigen::Vector3d& localRightFootPos, uint64_t timeStepMs, uint64_t initialTdsp)
{
    auto swingFoot = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b, double swingFootHeight, double t)
    {
        const double theta = 2*M_PI*t;
        const double r = 0.5;
        const double cycroid_x = r * (theta - sin(theta)) / M_PI; // [0, 1]
        const double cycroid_z = r * (1.0 - cos(theta)); // [0, 1]
        Eigen::Vector3d x;
        x(0) = (b(0) - a(0)) * cycroid_x + a(0);
        x(1) = (b(1) - a(1)) * cycroid_x + a(1);
        x(2) = swingFootHeight * cycroid_z;
        return x;
    };

    FootStep::SupportState prevSingleSupportState = FootStep::SupportState::RightSupport;
    Eigen::Vector3d bodyDirection(1, 0, 0);
    Eigen::Vector3d bodyPosition(Eigen::Vector3d::Zero());

    FootStep footStep;

    // Initial state
    footStep.time = 0;
    footStep.supportState = FootStep::DoubleSupport;
    footStep.leftPos = localLeftFootPos;
    footStep.rightPos = localRightFootPos;
    footStep.leftRot.setIdentity();
    footStep.rightRot.setIdentity();
    footStep.zmpRef = (footStep.leftPos + footStep.rightPos)/2.0;

    uint64_t nextTime = footStep.time + initialTdsp;
    while (footStep.time < nextTime) {
        co_yield footStep;
        footStep.time += timeStepMs;
    }

    while (true) {
        if (footStep.supportState == FootStep::DoubleSupport) {
            // DoubleSupport -> LeftSupport or RightSupport

            while (instruction->stop) {
                // Stop state
                footStep.zmpRef = (footStep.leftPos + footStep.rightPos)/2.0;

                co_yield footStep;
                footStep.time += timeStepMs;
            }

            if (instruction->direction.squaredNorm() > 0.001) {
                Eigen::Vector3d c = bodyDirection.cross(instruction->direction);
                if (prevSingleSupportState == FootStep::LeftSupport) {
                    if (c(2) >= 0) {  // turn left
                        bodyDirection = instruction->direction;
                    }
                } else {
                    if (c(2) < 0) {  // turn right
                        bodyDirection = instruction->direction;
                    }
                }
            }

            const Eigen::Quaterniond rot(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), bodyDirection));

            bodyPosition += rot * instruction->stride;

            const Eigen::Vector3d beginLeftPos = footStep.leftPos;
            const Eigen::Vector3d beginRightPos = footStep.rightPos;
            const Eigen::Quaterniond beginLeftRot(footStep.leftRot);
            const Eigen::Quaterniond beginRightRot(footStep.rightRot);

            Eigen::Vector3d endLeftPos = footStep.leftPos;
            Eigen::Vector3d endRightPos = footStep.rightPos;
            Eigen::Quaterniond endLeftRot(footStep.leftRot);
            Eigen::Quaterniond endRightRot(footStep.rightRot);

            if (prevSingleSupportState == FootStep::LeftSupport) {
                // DoubleSupport -> RightSupport
                footStep.supportState = FootStep::RightSupport;
                endLeftPos = bodyPosition + rot * localLeftFootPos;
                endLeftRot = rot;
            } else {
                // DoubleSupport -> LeftSupport
                footStep.supportState = FootStep::LeftSupport;
                endRightPos = bodyPosition + rot * localRightFootPos;
                endRightRot = rot;
            }

            // RightSupport/LeftSupport phase
            const double swingFootHeight = instruction->swingFootHeight;
            const uint64_t Tssp = instruction->Tssp;
            const uint64_t nextTime = footStep.time + Tssp;
            while (footStep.time < nextTime) {
                const double t = 1.0 - double(nextTime - footStep.time)/Tssp;
                if (footStep.supportState == FootStep::LeftSupport) {
                    // Swing Right
                    footStep.rightPos = swingFoot(beginRightPos, endRightPos, swingFootHeight, t);
                    footStep.rightRot = beginRightRot.slerp(t, endRightRot).toRotationMatrix();

                    footStep.zmpRef = footStep.leftPos;
                } else {
                    // Swing Left
                    footStep.leftPos = swingFoot(beginLeftPos, endLeftPos, swingFootHeight, t);
                    footStep.leftRot = beginLeftRot.slerp(t, endLeftRot).toRotationMatrix();

                    footStep.zmpRef = footStep.rightPos;
                }

                co_yield footStep;
                footStep.time += timeStepMs;
            }

            footStep.leftPos = endLeftPos;
            footStep.rightPos = endRightPos;

            footStep.leftRot = endLeftRot.toRotationMatrix();
            footStep.rightRot = endRightRot.toRotationMatrix();

        } else if ((footStep.supportState == FootStep::LeftSupport)
                || (footStep.supportState == FootStep::RightSupport)) {
            // LeftSupport or RightSupport -> DoubleSupport
            prevSingleSupportState = footStep.supportState;

            footStep.supportState = FootStep::DoubleSupport;

            Eigen::Vector3d beginZmp;
            Eigen::Vector3d endZmp;
            if (prevSingleSupportState == FootStep::LeftSupport) {
                beginZmp = footStep.leftPos;
                endZmp = footStep.rightPos;
            } else {
                beginZmp = footStep.rightPos;
                endZmp = footStep.leftPos;
            }

            // DoubleSupport phase
            const uint64_t Tdsp = instruction->Tdsp;
            const uint64_t nextTime = footStep.time + Tdsp;
            while (footStep.time < nextTime) {
                const double t = 1.0 - double(nextTime - footStep.time)/Tdsp;
                footStep.zmpRef = t * endZmp + (1.0 - t) * beginZmp;   // lerp

                co_yield footStep;
                footStep.time += timeStepMs;
            }
        }
    }
}

void calcConstraint(Eigen::MatrixXd& CI, Eigen::VectorXd& ci0,
        const std::deque<std::shared_ptr<PredictiveState>>& predictiveStateFIFO, int N,
        const Eigen::Vector3d& x, const Eigen::Vector3d& y,
        const Eigen::MatrixXd& Pzu, const Eigen::MatrixXd& Pzs)
{
    size_t sum = 0;
    for (int i = 0; i < N; i++) {
        sum += predictiveStateFIFO[i]->a.size();    // numPolygonSides
    }

    ci0.resize(sum);

    Eigen::SparseMatrix<double> Fx(sum, N);
    Eigen::SparseMatrix<double> Fy(sum, N);

    Fx.reserve(sum);
    Fy.reserve(sum);

    int ns = 0;
    for (int j = 0; j < N; j++) {
        const auto& predictiveState = predictiveStateFIFO[j];
        const int numPolygonSides = predictiveState->a.size();
        for (int i = 0; i < numPolygonSides; i++) {
            Fx.insert(ns+i, j) = predictiveState->a[i];
            Fy.insert(ns+i, j) = predictiveState->b[i];

            ci0(ns+i) = -predictiveState->c[i];
        }
        ns += numPolygonSides;
    }

    Fx.finalize();
    Fy.finalize();

    CI.resize(sum, N*2);

    CI.block(0, 0, sum, N) = Fx * Pzu;
    CI.block(0, N, sum, N) = Fy * Pzu;

    ci0 += Fx * (Pzs * x) + Fy * (Pzs * y);
}

void addFootStepToPredictiveFIFO(std::deque<std::shared_ptr<PredictiveState>>& predictiveStateFIFO, std::shared_ptr<LegBody>& predictiveLegBody, const std::shared_ptr<FootStep>& footStep) {
    predictiveLegBody->updateFootPolygon(*footStep);

    std::shared_ptr<PredictiveState> predictiveState = std::make_shared<PredictiveState>();
    predictiveState->time = footStep->time;
    predictiveState->footStep = footStep;
    calcConvexHullHalfSpace(predictiveLegBody->supportFootPolygon, predictiveState->a, predictiveState->b, predictiveState->c);
//    predictiveState->zmp_ref = calcCentorOfPolygon(predictiveLegBody->supportFootPolygon);
    predictiveState->zmpRef = footStep->zmpRef;

    predictiveStateFIFO.push_back(predictiveState);
};

int main(int argc, char* argv[])
{
    const int predictionTimeMs = 1600;
    const int timeStepMs = 20;
    const int N = predictionTimeMs / timeStepMs;

    const double g = 9.8;
    const double Zc = 0.8;

    const double alpha = 1.0;
    const double gamma = 100.0;

    const double dT = 0.001 * timeStepMs;

    Eigen::Vector3d x(Eigen::Vector3d::Zero());
    Eigen::Vector3d y(Eigen::Vector3d::Zero());

    const Eigen::Matrix3d A{
        {1, dT, dT*dT/2},
        {0,  1,      dT},
        {0,  0,       1}
    };

    const Eigen::Vector3d B{
        dT*dT*dT/6,
        dT*dT/2,
        dT
    };

    const Eigen::RowVector3d C{1, 0, -Zc/g};

    Eigen::MatrixXd Pps(Eigen::MatrixXd::Zero(N, 3));
    Eigen::MatrixXd Pvs(Eigen::MatrixXd::Zero(N, 3));
    Eigen::MatrixXd Pzs(Eigen::MatrixXd::Zero(N, 3));
    for (int i = 0; i < N; i++) {
        int n = i + 1;
        Pps(i, 0) = 1;
        Pps(i, 1) = n*dT;
        Pps(i, 2) = n*n*dT*dT/2;

        Pvs(i, 0) = 0;
        Pvs(i, 1) = 1;
        Pvs(i, 2) = n*dT;

        Pzs(i, 0) = 1;
        Pzs(i, 1) = n*dT;
        Pzs(i, 2) = n*n*dT*dT/2 - Zc/g;
    }

    Eigen::MatrixXd Ppu(Eigen::MatrixXd::Zero(N, N));
    Eigen::MatrixXd Pvu(Eigen::MatrixXd::Zero(N, N));
    Eigen::MatrixXd Pzu(Eigen::MatrixXd::Zero(N, N));
    for (int i = 0; i < N; i++) {
        for (int j = 0; j <= i; j++) {
            int n = (i - j);
            Ppu(i, j) = (1 + 3*n + 3*n*n) * dT*dT*dT/6;

            Pvu(i, j) = (1 + 2*n) * dT*dT/2;

            Pzu(i, j) = (1 + 3*n + 3*n*n) * dT*dT*dT/6 - dT*Zc/g;
        }
    }

    Eigen::VectorXd U(Eigen::VectorXd::Zero(2*N));

    Eigen::VectorXd Zx_ref(Eigen::VectorXd::Zero(N));
    Eigen::VectorXd Zy_ref(Eigen::VectorXd::Zero(N));

    const Eigen::MatrixXd Qxy = alpha * Eigen::MatrixXd::Identity(N, N) + gamma * Pzu.transpose() * Pzu;
    Eigen::MatrixXd Q(Eigen::MatrixXd::Zero(2*N, 2*N));
    Q.block(0, 0, N, N) = Qxy;
    Q.block(N, N, N, N) = Qxy;

    Eigen::VectorXd g0(2*N);
    Eigen::MatrixXd CE(2*N, 0);
    Eigen::VectorXd ce0(0);
    Eigen::MatrixXd CI(0, 2*N);
    Eigen::VectorXd ci0(0);

    // ---------------------------
    // LegBody
    // ---------------------------
    const Eigen::Vector3d localLeftFootPos(0, 0.1, 0);
    const Eigen::Vector3d localRightFootPos(0, -0.1, 0);

    const std::vector<Eigen::Vector3d> localLeftFootPolygon{
            { 0.15,  0.05, 0},
            {-0.05,  0.05, 0},
            {-0.05, -0.05, 0},
            { 0.15, -0.05, 0}
    };

    const std::vector<Eigen::Vector3d> localRightFootPolygon{
            { 0.15,  0.05, 0},
            {-0.05,  0.05, 0},
            {-0.05, -0.05, 0},
            { 0.15, -0.05, 0}
    };

    auto presentLegBody = std::make_shared<LegBody>();
    presentLegBody->localLeftFootPolygon = localLeftFootPolygon;
    presentLegBody->localRightFootPolygon = localRightFootPolygon;

    auto predictiveLegBody = std::make_shared<LegBody>();
    predictiveLegBody->localLeftFootPolygon = localLeftFootPolygon;
    predictiveLegBody->localRightFootPolygon = localRightFootPolygon;

    // ---------------------------
    // PredictiveState
    // ---------------------------
    std::deque<std::shared_ptr<PredictiveState>> predictiveStateFIFO;

    auto stepInstruction = std::make_shared<FootStepInstruction>();
    stepInstruction->stop = false;
    stepInstruction->stride = {0.1, 0, 0};
    stepInstruction->direction = {1, 0, 0};
    stepInstruction->Tdsp = 200;
    stepInstruction->Tssp = 1000;
    stepInstruction->swingFootHeight = 0.1;

    // FootStep generator
    auto footStepGenerator = generateFootStep(stepInstruction, localLeftFootPos, localRightFootPos, timeStepMs, 2*1000);

    // Fill initial state
    while (predictiveStateFIFO.size() < N) {
        footStepGenerator.move_next();
        std::shared_ptr<FootStep> predictiveFootStep = std::make_shared<FootStep>(footStepGenerator.current_value());
        addFootStepToPredictiveFIFO(predictiveStateFIFO, predictiveLegBody, predictiveFootStep);
    }

    const size_t drawPointsBuffSize = 4000 / timeStepMs;
    std::deque<Eigen::Vector3d> leftFootPosBuff;
    std::deque<Eigen::Vector3d> rightFootPosBuff;
    std::deque<Eigen::Vector3d> zmpBuff;
    std::deque<Eigen::Vector3d> comBuff;

    std::shared_ptr<PredictiveState> presentPredictiveState;
    std::shared_ptr<FootStep> presentFootStep;

    // Set View
#ifdef EMSCRIPTEN
    ptgl::QuickGraphicsView view(std::make_unique<ptgl::EMGLUTGraphicsDriver>(argc, argv));
#else   // EMSCRIPTEN
    ptgl::QuickGraphicsView view(std::make_unique<ptgl::GLFWGraphicsDriver>());
#endif  // EMSCRIPTEN
    view.setWindowTitle("MPC Demo");
    view.setWindowSize(1280, 800);

    bool fixedOriginMode = false;

    view.setPrevProcessFunction([&](){
        // ---------------------------
        // Update predictive FootStep
        // ---------------------------
        while (predictiveStateFIFO.size() >= N) {
            predictiveStateFIFO.pop_front();
        }

        footStepGenerator.move_next();
        std::shared_ptr<FootStep> predictiveFootStep = std::make_shared<FootStep>(footStepGenerator.current_value());
        addFootStepToPredictiveFIFO(predictiveStateFIFO, predictiveLegBody, predictiveFootStep);

        presentPredictiveState = predictiveStateFIFO.front();
        presentFootStep = presentPredictiveState->footStep;

        // ---------------------------
        // Update constraint
        // ---------------------------
        calcConstraint(CI, ci0, predictiveStateFIFO, N, x, y, Pzu, Pzs);

        Zx_ref.setZero();
        Zy_ref.setZero();
        for (int i = 0; i < N; i++) {
            const auto& predictiveState = predictiveStateFIFO[i];
            Zx_ref(i) = predictiveState->zmpRef(0);
            Zy_ref(i) = predictiveState->zmpRef(1);
        }

        // ---------------------------
        // Update MPC
        // ---------------------------
        g0.segment(0, N) = Pzu.transpose().triangularView<Eigen::Upper>() * (Pzs * x - Zx_ref) * gamma;
        g0.segment(N, N) = Pzu.transpose().triangularView<Eigen::Upper>() * (Pzs * y - Zy_ref) * gamma;

        // solve QP
        quadprogpp::solve_quadprog(Q, g0, CE, ce0, CI.transpose(), ci0, U);

        // ---------------------------
        // Update state
        // ---------------------------
        const auto& Ux = U.segment(0, N);
        const auto& Uy = U.segment(N, N);

#if 0
        Eigen::VectorXd X  = Pps * x + Ppu.triangularView<Eigen::Lower>() * Ux;
        Eigen::VectorXd Y  = Pps * y + Ppu.triangularView<Eigen::Lower>() * Uy;

        Eigen::VectorXd Zx = Pzs * x + Pzu.triangularView<Eigen::Lower>() * Ux;
        Eigen::VectorXd Zy = Pzs * y + Pzu.triangularView<Eigen::Lower>() * Uy;
#endif

        Eigen::Vector3d xn = A * x + B * Ux(0);
        Eigen::Vector3d yn = A * y + B * Uy(0);

        double zmp_x = C * x;
        double zmp_y = C * y;

        x = xn;
        y = yn;

        // update LegBody
        presentLegBody->updateFootPolygon(*presentFootStep);
        presentLegBody->com = {x(0), y(0), Zc};
        presentLegBody->zmp = {zmp_x, zmp_y, 0};
    });

    view.setRenderSceneFunction([&](ptgl::Renderer3D* r){
        // Update previous buffer
        leftFootPosBuff.push_back(presentFootStep->leftPos);
        rightFootPosBuff.push_back(presentFootStep->rightPos);
        zmpBuff.push_back(presentLegBody->zmp);
        comBuff.push_back(presentLegBody->com);

        while (leftFootPosBuff.size() > drawPointsBuffSize) {
            leftFootPosBuff.pop_front();
            rightFootPosBuff.pop_front();
            zmpBuff.pop_front();
            comBuff.pop_front();
        }

        if (fixedOriginMode) {
            r->pushMatrix();
            r->translate(-presentLegBody->com(0), -presentLegBody->com(1), 0);
        }

        r->setPointSize(12);

        // Draw support foot polygon
        r->setColor(0.4, 0.4, 1);
        r->drawLineLoop(presentLegBody->supportFootPolygon.front().data(), presentLegBody->supportFootPolygon.size());

        // Draw left leg
        r->setColor(0, 1, 0);
        r->drawPoint(presentFootStep->leftPos);
        r->drawLineLoop(presentLegBody->leftFootPolygon.front().data(), presentLegBody->leftFootPolygon.size());

        // Draw right leg
        r->setColor(1, 0, 0);
        r->drawPoint(presentFootStep->rightPos);
        r->drawLineLoop(presentLegBody->rightFootPolygon.front().data(), presentLegBody->rightFootPolygon.size());

        // Draw LegBody
        r->setColor(0, 0, 0);
        r->drawLine(presentLegBody->com, presentFootStep->leftPos);
        r->drawLine(presentLegBody->com, presentFootStep->rightPos);

        r->setColor(1, 1, 1);
        r->drawPoint(presentLegBody->com);  // com
        r->drawPoint(presentLegBody->com(0), presentLegBody->com(1), 0);

        r->setColor(1, 1, 0);
        r->drawPoint(presentLegBody->zmp);  // zmp

        r->setColor(1, 0.5, 0);
        r->drawPoint(presentPredictiveState->zmpRef); // zmp ref

        // Draw previous
        r->setPointSize(3);
        r->setColor(0.4, 1, 0.4);
        r->drawPoints(leftFootPosBuff);

        r->setColor(1, 0.4, 0.4);
        r->drawPoints(rightFootPosBuff);

        // Previous zmp
        r->setColor(1, 1, 0);
        r->drawPoints(zmpBuff);

        // Previous com
        r->setColor(1, 1, 1);
        r->drawPoints(comBuff);

        // Previous com (Projected on the floor)
        r->beginDrawPoints();
        for (const auto& p : comBuff) {
            r->addDrawPoints(p(0), p(1), 0);
        }
        r->endDrawPoinhts();


        if (fixedOriginMode) {
            r->popMatrix();
        }
    });

    view.setRenderTextSceneFunction([&](ptgl::TextRenderer* r){
        int x = 10;
        int y = 20;
        int dy = 20;
        r->setTextColor(1,1,1);
        r->drawText(x, y, "time : " + std::to_string(0.001 * presentFootStep->time)); y += dy;
        r->drawText(x, y, "Tdsp : " + std::to_string(0.001 *stepInstruction->Tdsp) + ", Tssp : " + std::to_string(0.001 *stepInstruction->Tssp)); y += dy;
        r->drawText(x, y, "Stride : " + std::to_string(stepInstruction->stride(0)) + ", " + std::to_string(stepInstruction->stride(1)) + ", " + std::to_string(stepInstruction->stride(2))); y += dy;
        r->drawText(x, y, "Direction : " + std::to_string(stepInstruction->direction(0)) + ", " + std::to_string(stepInstruction->direction(1)) + ", " + std::to_string(stepInstruction->direction(2))); y += dy;

        y += dy;
        r->drawText(x, y, "Key input:"); y += dy;
        r->drawText(x, y, "w : Move forward"); y += dy;
        r->drawText(x, y, "s : Move backward"); y += dy;
        r->drawText(x, y, "a : Turn left"); y += dy;
        r->drawText(x, y, "d : Turn right"); y += dy;
        r->drawText(x, y, "o : - Tdsp"); y += dy;
        r->drawText(x, y, "p : + Tdsp"); y += dy;
        r->drawText(x, y, "k : - Tssp"); y += dy;
        r->drawText(x, y, "l : + Tssp"); y += dy;
        r->drawText(x, y, "c : Fixed to origin"); y += dy;
    });

    view.setKeyPressEventFunction([&](ptgl::KeyEvent* e){
        switch (e->key()) {
        case ptgl::Key::Key_Up:
        case ptgl::Key::Key_W:
            stepInstruction->stride += Eigen::Vector3d(0.1, 0, 0);
            break;
        case ptgl::Key::Key_Down:
        case ptgl::Key::Key_S:
            stepInstruction->stride -= Eigen::Vector3d(0.1, 0, 0);
            break;
        case ptgl::Key::Key_Right:
        case ptgl::Key::Key_D:
            stepInstruction->direction = Eigen::AngleAxisd(-M_PI/8, Eigen::Vector3d::UnitZ()) * stepInstruction->direction;
            break;
        case ptgl::Key::Key_Left:
        case ptgl::Key::Key_A:
            stepInstruction->direction = Eigen::AngleAxisd(M_PI/8, Eigen::Vector3d::UnitZ()) * stepInstruction->direction;
            break;
        case ptgl::Key::Key_P:
            stepInstruction->Tdsp += 100;
            break;
        case ptgl::Key::Key_O:
            stepInstruction->Tdsp = (stepInstruction->Tdsp > 100) ? (stepInstruction->Tdsp - 100) : 100;
            break;
        case ptgl::Key::Key_L:
            stepInstruction->Tssp += 100;
            break;
        case ptgl::Key::Key_K:
            stepInstruction->Tssp = (stepInstruction->Tssp > 100) ? (stepInstruction->Tssp - 100) : 100;
            break;
        case ptgl::Key::Key_C:
            fixedOriginMode = !fixedOriginMode;
            break;
        default:
            break;
        }
    });

    view.initialize();
    view.execute();

#ifdef EMSCRIPTEN
#else   // EMSCRIPTEN
    while (!view.terminated()) {

    }
#endif  // EMSCRIPTEN

    return 0;
}
