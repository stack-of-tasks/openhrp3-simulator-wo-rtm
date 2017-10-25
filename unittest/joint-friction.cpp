#include <hrpGepModel/Link.h>
#include <hrpGepModel/LinkTraverse.h>
#include <hrpGepModel/Body.h>
#include <hrpGepModel/ForwardDynamicsABM.h>
#include <hrpGepModel/ForwardDynamicsCBM.h>

#include <hrpModel/Link.h>
#include <hrpModel/LinkTraverse.h>
#include <hrpModel/Body.h>
#include <hrpModel/ForwardDynamicsABM.h>
#include <hrpModel/ForwardDynamicsCBM.h>

#include <boost/test/unit_test.hpp>


BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE (test_joint_no_friction_ABM)
{
  double timestep = 0.001;
  const Eigen::Vector3d gravity(0,0,-9.8);

  /////////////////////////////////////////////////////////////
  hrp::BodyPtr bodyorgPtr(new hrp::Body());
  hrpGep::BodyPtr bodyPtr(new hrpGep::Body());
  bodyorgPtr->rootLink()->jointType = hrp::Link::FREE_JOINT;
  bodyorgPtr->rootLink()->name = "root_link";
  bodyorgPtr->rootLink()->m = 10.0;
  bodyorgPtr->rootLink()->I = Eigen::Matrix3d::Random();
  bodyorgPtr->rootLink()->jointId = -1;
  //Create Rotational First Link
  bodyorgPtr->rootLink()->b = Eigen::Vector3d::Random();
  bodyorgPtr->rootLink()->Rs = Eigen::Matrix3d::Random();
  bodyorgPtr->rootLink()->c = Eigen::Vector3d::Random();
  bodyorgPtr->rootLink()->dvo.setZero();
  bodyorgPtr->rootLink()->dw.setZero();

  hrp::Link first_link_org;
  first_link_org.jointType = hrp::Link::ROTATIONAL_JOINT;
  first_link_org.a << 0,0,1;
  first_link_org.name = "first_link_org";
  first_link_org.m = 10.0;
  first_link_org.jointId = 0;
  first_link_org.I = Eigen::Matrix3d::Random();
  first_link_org.b = Eigen::Vector3d::Random();
  first_link_org.Rs = Eigen::Matrix3d::Random();
  first_link_org.c = Eigen::Vector3d::Random();
  bodyorgPtr->rootLink()->addChild(&first_link_org);
  bodyorgPtr->updateLinkTree();
  bodyorgPtr->initializeConfiguration();

  //////////////////////////////////////////////////////////////
  // Create Body
  hrpGep::Body body;
  bodyPtr->rootLink()->jointType = hrpGep::Link::FREE_JOINT;
  bodyPtr->rootLink()->name = bodyorgPtr->rootLink()->name;
  bodyPtr->rootLink()->m = bodyorgPtr->rootLink()->m;
  bodyPtr->rootLink()->I = bodyorgPtr->rootLink()->I;
  bodyPtr->rootLink()->jointId = bodyorgPtr->rootLink()->jointId;
  //Create Rotational First Link
  bodyPtr->rootLink()->b = bodyorgPtr->rootLink()->b;
  bodyPtr->rootLink()->Rs = bodyorgPtr->rootLink()->Rs;
  bodyPtr->rootLink()->c = bodyorgPtr->rootLink()->c;
  bodyPtr->rootLink()->dvo = bodyorgPtr->rootLink()->dvo;
  bodyPtr->rootLink()->dw = bodyorgPtr->rootLink()->dw;
  hrpGep::Link first_link;
  first_link.jointType = hrpGep::Link::ROTATIONAL_JOINT;
  first_link.a << 0,0,1;
  first_link.name = "first_link";
  first_link.m = first_link_org.m;
  first_link.jointId = first_link_org.jointId;
  first_link.I = first_link_org.I;
  first_link.b = first_link_org.b;
  first_link.Rs = first_link_org.Rs;
  first_link.c = first_link_org.c;
  bodyPtr->rootLink()->addChild(&first_link);
  bodyPtr->updateLinkTree();
  bodyPtr->initializeConfiguration();
  /////////////////////////////////////////////////////////////

  const Eigen::VectorXd q = Eigen::VectorXd::Random(bodyPtr->numJoints());
  const Eigen::VectorXd dq = Eigen::VectorXd::Random(bodyPtr->numJoints());
  const Eigen::VectorXd tau = 10.0*Eigen::VectorXd::Zero(6+bodyPtr->numJoints());

  for(int i=0;i<bodyPtr->numJoints();i++){
    bodyPtr->linkTraverse()[i]->q = q[i];
    bodyPtr->linkTraverse()[i]->dq = dq[i];
    bodyPtr->linkTraverse()[i]->ddq = 0.0;
    bodyorgPtr->linkTraverse()[i]->q = q[i];
    bodyorgPtr->linkTraverse()[i]->dq = dq[i];
    bodyorgPtr->linkTraverse()[i]->ddq = 0.0;
  }

  hrp::ForwardDynamicsABM fdABMorg (bodyorgPtr);
  //fdABM.setRungeKuttaMethod();
  fdABMorg.setEulerMethod();
  fdABMorg.setTimeStep(timestep);
  fdABMorg.setGravityAcceleration(gravity);
  fdABMorg.initialize();
  fdABMorg.calcNextState();

  hrpGep::ForwardDynamicsABM fdABM (bodyPtr);
  //fdABM.setRungeKuttaMethod();
  fdABM.setEulerMethod();
  fdABM.setTimeStep(timestep);
  fdABM.setGravityAcceleration(gravity);
  fdABM.initialize();
  fdABM.calcNextState();
  ///////////////////////////////////////////////////////////
  bodyPtr->linkTraverse()[1]->Kv_p = 10.0;
  bodyPtr->linkTraverse()[1]->Kv_n = 8.0;
  bodyPtr->linkTraverse()[1]->Kf_p = 6.0;
  bodyPtr->linkTraverse()[1]->Kf_n = 4.0;
  for(int i=0;i<bodyPtr->numJoints();i++){
    bodyPtr->linkTraverse()[i+1]->u = tau[i+6] + fdABM.jointFriction(bodyPtr->linkTraverse()[i+1]);
  }
  bodyPtr->rootLink()->fext = tau.head<3>();
  bodyPtr->rootLink()->tauext = tau.segment<3>(3);

  for(int i=0;i<bodyorgPtr->numJoints();i++){
    bodyorgPtr->linkTraverse()[i+1]->u = tau[i+6];
  }
  bodyorgPtr->rootLink()->fext = tau.head<3>();
  bodyorgPtr->rootLink()->tauext = tau.segment<3>(3);

  fdABMorg.calcNextState();
  fdABM.calcNextState();
  ///////////////////////////////////////////////////////////
  Eigen::VectorXd ddq(bodyPtr->numJoints()+6);
  ddq.head<3>() = bodyPtr->rootLink()->dvo;
  ddq.segment<3>(3) = bodyPtr->rootLink()->dw;
  for(int i=0;i<bodyPtr->numJoints();i++){  
    ddq[i+6] = bodyPtr->linkTraverse()[i]->ddq;
  }

  Eigen::VectorXd ddqorg(bodyorgPtr->numJoints()+6);
  ddqorg.head<3>() = bodyorgPtr->rootLink()->dvo;
  ddqorg.segment<3>(3) = bodyorgPtr->rootLink()->dw;
  for(int i=0;i<bodyorgPtr->numJoints();i++){  
    ddqorg[i+6] = bodyorgPtr->linkTraverse()[i]->ddq;
  }
  /////////////////////////////////////////////////////////////
  BOOST_CHECK(ddq.isApprox(ddqorg,1e-12));

  //Need to detach to facilitate destructors
  bodyPtr->rootLink()->detachChild(&first_link);
  bodyorgPtr->rootLink()->detachChild(&first_link_org);
}
BOOST_AUTO_TEST_SUITE_END ()
