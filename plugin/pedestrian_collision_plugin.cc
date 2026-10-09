// Gives a Gazebo Classic actor a collision box.
//
// Actors are animated meshes without physics, so other models drive straight
// through them. This plugin is loaded by a small model holding a kinematic box
// and moves that model to the actor's pose on every physics step.
//
//   <model name="NAME_collision">
//     <link name="link"><kinematic>true</kinematic> ... box collision ... </link>
//     <plugin name="NAME_collision_plugin" filename="libpedestrian_collision_plugin.so">
//       <actor_name>NAME</actor_name>
//       <z_offset>0.9</z_offset>   <!-- box centre above the actor origin -->
//     </plugin>
//   </model>

#include <functional>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
class PedestrianCollisionPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    this->model = _model;
    this->world = _model->GetWorld();
    if (!_sdf->HasElement("actor_name")) {
      gzerr << "[pedestrian_collision] <actor_name> missing in " << _model->GetName() << "\n";
      return;
    }
    this->actorName = _sdf->Get<std::string>("actor_name");
    if (_sdf->HasElement("z_offset")) {
      this->zOffset = _sdf->Get<double>("z_offset");
    }
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&PedestrianCollisionPlugin::OnUpdate, this, std::placeholders::_1));
  }

private:
  void OnUpdate(const common::UpdateInfo & _info)
  {
    if (!this->actor) {
      this->actor = this->world->ModelByName(this->actorName);
      if (!this->actor) {
        if (!this->warned) {
          gzwarn << "[pedestrian_collision] actor '" << this->actorName << "' not found\n";
          this->warned = true;
        }
        return;
      }
    }

    const ignition::math::Pose3d actorPose = this->actor->WorldPose();
    const ignition::math::Pose3d target(
      actorPose.Pos().X(), actorPose.Pos().Y(), actorPose.Pos().Z() + this->zOffset,
      0.0, 0.0, actorPose.Rot().Yaw());

    // Give the box the actor's velocity so contacts see a moving body, not a
    // teleporting one.
    const double dt = (_info.simTime - this->lastTime).Double();
    if (this->hasLast && dt > 0.0) {
      this->model->SetLinearVel((target.Pos() - this->lastPos) / dt);
    }
    this->model->SetWorldPose(target);

    this->lastPos = target.Pos();
    this->lastTime = _info.simTime;
    this->hasLast = true;
  }

  physics::ModelPtr model;
  physics::WorldPtr world;
  physics::ModelPtr actor;
  std::string actorName;
  double zOffset{0.9};
  bool warned{false};
  bool hasLast{false};
  ignition::math::Vector3d lastPos;
  common::Time lastTime;
  event::ConnectionPtr updateConnection;
};

GZ_REGISTER_MODEL_PLUGIN(PedestrianCollisionPlugin)
}  // namespace gazebo
