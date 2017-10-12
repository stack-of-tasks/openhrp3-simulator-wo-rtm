/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 */

/** 
    \author Shin'ichiro Nakaoka
*/

#ifndef HRPGEPMODEL_INVERSE_KINEMATICS_H_INCLUDED
#define HRPGEPMODEL_INVERSE_KINEMATICS_H_INCLUDED

#include <boost/shared_ptr.hpp>
#include <hrpGepUtil/Eigen3d.h>

namespace hrpGep {

    class InverseKinematics
    {
      public:
        virtual ~InverseKinematics() { }
        virtual bool calcInverseKinematics(const Vector3& end_p, const Matrix33& end_R) = 0;
    };

    typedef boost::shared_ptr<InverseKinematics> IInverseKinematicsPtr;
}

#endif
