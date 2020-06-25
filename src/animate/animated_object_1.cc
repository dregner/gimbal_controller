/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo {
    class Animated_object : public ModelPlugin {


    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
            // Store the pointer to the model
            this->_model = _parent;
            this->_link = _parent->GetLinks()[0];
            this->world_ = _parent->GetWorld();
            this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
                    std::bind(&Animated_object::OnUpdate, this, std::placeholders::_1)));

            int time_coef = 4;
            // create the animation
            gazebo::common::PoseAnimationPtr anim(
                    // name the animation "test",
                    // make it last 450 seconds (50 seconds for each riser),
                    // and set it on a repeat loop
                    new gazebo::common::PoseAnimation("airplane", 14*time_coef, true));

            gazebo::common::PoseKeyFrame *key;

            key = anim->CreateKeyFrame(0);
            key->Translation(ignition::math::Vector3d(0, 0, 10));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));

            key = anim->CreateKeyFrame(3 * time_coef);
            key->Translation(ignition::math::Vector3d(0, 10, 5));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));

            key = anim->CreateKeyFrame(6 * time_coef);
            key->Translation(ignition::math::Vector3d(0, 10, 15));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));

            key = anim->CreateKeyFrame(9 * time_coef);
            key->Translation(ignition::math::Vector3d(0, -10, 5));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));

            key = anim->CreateKeyFrame(10 * time_coef);
            key->Translation(ignition::math::Vector3d(0, -10, 15));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));

            key = anim->CreateKeyFrame(11 * time_coef);
            key->Translation(ignition::math::Vector3d(0, 0, 15));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));

            key = anim->CreateKeyFrame(12 * time_coef);
            key->Translation(ignition::math::Vector3d(0, 0, 10));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));








            // set the animation
            _parent->SetAnimation(anim);
        }

    public:
        void
        OnUpdate(const common::UpdateInfo &_info) {
            // Time
//            std::cerr << "\nO A posicao da camera  [" <<
//                      _model->GetWorldPose() << "]\n";

        }

        // Pointer to the model
    private:
        physics::ModelPtr _model;
        // Pointer to the model
    private:
        physics::LinkPtr _link;
    private:
        // World
        physics::WorldPtr world_;
    private:
        std::vector<event::ConnectionPtr> connections;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

// Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(Animated_object)
}
