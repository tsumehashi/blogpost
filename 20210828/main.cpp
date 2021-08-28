#include <iostream>
#include <algorithm>
#include <ucnoid/BodyLoader>
#include <ucnoid/DyBody>
#include <ucnoid/DyWorld>
#include <ucnoid/EigenUtil>
#include <ucnoid/JointPath>
#include "ptgl/Core/QuickGraphicsView.h"
#ifdef EMSCRIPTEN
#include <emscripten.h>
#include "ptgl/Driver/EMGLUTGraphicsDriver.h"
#else   // EMSCRIPTEN
#include "ptgl/Driver/GLFWGraphicsDriver.h"
#endif  // EMSCRIPTEN
#include "ptgl/Handle/TransformHandle.h"
#include "ptgl/Core/QuickGraphicsView.h"
#include "ptgl/Handle/TransformHandle.h"
#include "ptgl/GUI/ButtonPanel.h"
#include "SceneBodyItem.h"
#include "ConstraintForceSolverMod.h"

int main(int argc, char* argv[])
{
    std::string modelfile = "SR1-2D_ext.body";
    cnoid::BodyLoader loader;
    cnoid::DyBodyPtr body(new cnoid::DyBody(*loader.load(modelfile)));

    body->calcForwardKinematics();

    for (int i = 0; i < body->numLinks(); ++i) {
        cnoid::Link* link = body->link(i);
        std::cout << link->name() << ":" << std::endl;
        std::cout << "p = " << link->p().transpose() << std::endl;
    }

    cnoid::Link* footL = body->link("LLEG_ANKLE_R");
    cnoid::Link* footR = body->link("RLEG_ANKLE_R");
    cnoid::Link* waist = body->link("WAIST");
    cnoid::Link* chest = body->link("CHEST");
    cnoid::Link* handL = body->link("LARM_WRIST_R");
    cnoid::Link* handR = body->link("RARM_WRIST_R");

    cnoid::Link* fixedLeftFoot = body->link("FIXED_LFOOT");
    cnoid::Link* fixedRightFoot = body->link("FIXED_RFOOT");

    cnoid::JointPath jointPathFootL(body->rootLink(), footL);
    cnoid::JointPath jointPathFootR(body->rootLink(), footR);
    cnoid::JointPath jointPathWaist(body->rootLink(), waist);
    cnoid::JointPath jointPathChest(body->rootLink(), chest);
    cnoid::JointPath jointPathHandL(body->rootLink(), handL);
    cnoid::JointPath jointPathHandR(body->rootLink(), handR);

    // joint damping gain
    std::vector<double> Kdqs(body->numJoints(), 500);
    Kdqs[body->link("LARM_ELBOW")->jointId()] = 100;
    Kdqs[body->link("RARM_ELBOW")->jointId()] = 100;
    Kdqs[body->link("LLEG_KNEE")->jointId()]  = 100;
    Kdqs[body->link("RLEG_KNEE")->jointId()]  = 100;

    cnoid::World<cnoid::ucnoid::ConstraintForceSolverMod> world;
    world.setTimeStep(0.001);
    world.addBody(body);
//    world.constraintForceSolver.setSelfCollisionDetectionEnabled(0, false);   // choreonoid 1.8

    world.initialize();

#ifdef EMSCRIPTEN
    ptgl::GraphicsDriverPtr driver = std::make_unique<ptgl::EMGLUTGraphicsDriver>(argc, argv);
#else   // EMSCRIPTEN
    ptgl::GraphicsDriverPtr driver = std::make_unique<ptgl::GLFWGraphicsDriver>();
#endif  // EMSCRIPTEN
    ptgl::QuickGraphicsView view(std::move(driver));
    view.setWindowTitle("EmscriptenDemo");
    view.setWindowSize(1280, 800);

    ptgl::handle::TransformHandlePtr handleFootL = std::make_shared<ptgl::handle::TransformHandle>();
    ptgl::handle::TransformHandlePtr handleFootR = std::make_shared<ptgl::handle::TransformHandle>();
    ptgl::handle::TransformHandlePtr handleWaist = std::make_shared<ptgl::handle::TransformHandle>();
    ptgl::handle::TransformHandlePtr handleChest = std::make_shared<ptgl::handle::TransformHandle>();
    ptgl::handle::TransformHandlePtr handleHandL = std::make_shared<ptgl::handle::TransformHandle>();
    ptgl::handle::TransformHandlePtr handleHandR = std::make_shared<ptgl::handle::TransformHandle>();

    view.addGraphicsItem(handleFootL);
    view.addGraphicsItem(handleFootR);
    view.addGraphicsItem(handleWaist);
    view.addGraphicsItem(handleChest);
    view.addGraphicsItem(handleHandL);
    view.addGraphicsItem(handleHandR);

    handleFootL->transform()->setPosition(footL->p());
    handleFootR->transform()->setPosition(footR->p());
    handleWaist->transform()->setPosition(waist->p());
    handleChest->transform()->setPosition(chest->p());
    handleHandL->transform()->setPosition(handL->p());
    handleHandR->transform()->setPosition(handR->p());

    handleFootL->setVisible(true);
    handleFootR->setVisible(false);
    handleWaist->setVisible(false);
    handleChest->setVisible(false);
    handleHandL->setVisible(false);
    handleHandR->setVisible(false);

    auto panel = std::make_shared<ptgl::gui::ButtonPanel>("Handle");
    panel->addButton("FootL")->setOnToggledFunction([&](bool on){ handleFootL->setVisible(on); });
    panel->addButton("FootR")->setOnToggledFunction([&](bool on){ handleFootR->setVisible(on); });
    panel->addButton("Waist")->setOnToggledFunction([&](bool on){ handleWaist->setVisible(on); });
    panel->addButton("Chest")->setOnToggledFunction([&](bool on){ handleChest->setVisible(on); });
    panel->addButton("HandL")->setOnToggledFunction([&](bool on){ handleHandL->setVisible(on); });
    panel->addButton("HandR")->setOnToggledFunction([&](bool on){ handleHandR->setVisible(on); });
    panel->button("FootL")->setChecked(true);
    panel->setPos(10, 40);
    view.addGraphicsItem(panel);

    auto sceneBody = std::make_shared<SceneBodyItem>(body);
    view.addGraphicsItem(sceneBody);
    sceneBody->setShowLink(true);
    sceneBody->setStartShowLink("WAIST");

    view.setPrevProcessFunction([&](){
        auto updateWorld = [&](){

            auto calcConstraint = [](Eigen::VectorXd& out_U, const cnoid::JointPath& jointPath, cnoid::Link* targetLink,
                                     const Eigen::Vector3d& target_p, const Eigen::Matrix3d& target_R,
                                     const std::array<bool, 6>& constraintIndex){
                const double Kp  = 200000;
                const double rKp = 50000;
                const std::array<double, 6> gains{Kp, Kp, Kp, rKp, rKp, rKp};

                Eigen::Vector<double, 6> ev;
                ev << target_p - targetLink->p(),
                      cnoid::omegaFromRot(Eigen::Matrix3d(target_R * targetLink->R().transpose()));

                const int nC = std::count(constraintIndex.begin(), constraintIndex.end(), true);
                const int nJ = jointPath.numJoints();
                Eigen::VectorXd v(nC);
                Eigen::MatrixXd out_J(6, nJ);
                Eigen::MatrixXd J(nC, nJ);

                cnoid::setJacobian<0b111111, 0, 0>(jointPath, targetLink, out_J);

                int index = 0;
                for (int i = 0; i < 6; i++) {
                    if (constraintIndex[i]) {
                        v(index) = gains[i] * ev(i);
                        J.row(index) = out_J.row(i);
                        index++;
                    }
                }

                Eigen::VectorXd u = J.transpose() * v;
                for (int i = 0; i < nJ; i++) {
                    int jointId = jointPath.joint(i)->jointId();
                    out_U(jointId) += u(i);
                }
            };

            Eigen::VectorXd U(Eigen::VectorXd::Zero(body->numJoints()));

            // Calc constraint
            calcConstraint(U, jointPathFootL, footL, handleFootL->transform()->position(), handleFootL->transform()->rotation(), {0, 0, 0, 1, 1, 1});
            calcConstraint(U, jointPathFootR, footR, handleFootR->transform()->position(), handleFootR->transform()->rotation(), {0, 0, 0, 1, 1, 1});
            calcConstraint(U, jointPathWaist, waist, handleWaist->transform()->position(), handleWaist->transform()->rotation(), {1, 1, 1, 1, 1, 1});
            calcConstraint(U, jointPathChest, chest, handleChest->transform()->position(), handleChest->transform()->rotation(), {0, 0, 0, 1, 1, 1});
            calcConstraint(U, jointPathHandL, handL, handleHandL->transform()->position(), handleHandL->transform()->rotation(), {1, 1, 1, 1, 1, 1});
            calcConstraint(U, jointPathHandR, handR, handleHandR->transform()->position(), handleHandR->transform()->rotation(), {1, 1, 1, 1, 1, 1});

            // Set fixed constraint
            fixedLeftFoot->setOffsetTranslation(handleFootL->transform()->position());
            fixedRightFoot->setOffsetTranslation(handleFootR->transform()->position());

            // Calc joint limit
            for (auto&& joint : body->joints()) {
                const double Kj = 100000;
                int id = joint->jointId();

                if (joint->q() < joint->q_lower()) {
                    const double u = Kj * (joint->q_lower() - joint->q());
                    if (U(id) > 0) {
                        U(id) = std::max(U(id), u);
                    } else {
                        U(id) = u;
                    }
                } else if (joint->q_upper() < joint->q()) {
                    const double u = Kj * (joint->q_upper() - joint->q());
                    joint->u() = u;
                    if (U(id) < 0) {
                        U(id) = std::min(U(id), u);
                    } else {
                        U(id) = u;
                    }
                }
            }

            // Set joint damping
            for (auto&& joint : body->joints()) {
                U(joint->jointId()) += -Kdqs[joint->jointId()] * joint->dq();
            }

            for (auto&& joint : body->joints()) {
                joint->u() = U(joint->jointId());
            }

            // Update World
            world.calcNextState();
            world.constraintForceSolver.clearExternalForces();
        };

        const int iteration = 1000 / 60;
        for (int i = 0; i < iteration; ++i) {
            updateWorld();
        }
    });

    view.setRenderTextSceneFunction([&](ptgl::TextRenderer* r){
        r->setTextColor(1,1,1);
        r->drawText(10, 20, "time: " + std::to_string(world.currentTime()));
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
