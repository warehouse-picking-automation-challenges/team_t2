/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Toshiba Corporation, 
 *                     Toshiba Infrastructure Systems & Solutions Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Toshiba Corporation, nor the Toshiba
 *       Infrastructure Systems & Solutions Corporation, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef T2_MOTION_PLANNER_T2_ROBOT_MODEL_H
#define T2_MOTION_PLANNER_T2_ROBOT_MODEL_H

#include <moveit/robot_model/robot_model.h>

namespace t2_motion_planner
{
class T2RobotModel : public moveit::core::RobotModel
{
public:
  T2RobotModel(const boost::shared_ptr<const urdf::ModelInterface>& urdf_model,
               const boost::shared_ptr<const srdf::Model>& srdf_model)
    : moveit::core::RobotModel(urdf_model, srdf_model)
  {
  }

  double weightingDistance(const double* state1, const double* state2, const std::vector<std::string>& joint_names,
                           const std::vector<double>& weight) const
  {
    double d = 0.0;

    for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
    {
      auto ite = std::find(joint_names.begin(), joint_names.end(), active_joint_model_vector_[i]->getName());
      double w = 1.0;

      if (ite != joint_names.end())
      {
        w = weight[std::distance(joint_names.begin(), ite)];
      }

      d += w * active_joint_model_vector_[i]->getDistanceFactor() *
           active_joint_model_vector_[i]->distance(state1 + active_joint_model_start_index_[i],
                                                   state2 + active_joint_model_start_index_[i]);
    }

    return d;
  }
};  // class

}  // namespace t2_motion_planner

#endif  // T2_MOTION_PLANNER_T2_ROBOT_MODEL_H
