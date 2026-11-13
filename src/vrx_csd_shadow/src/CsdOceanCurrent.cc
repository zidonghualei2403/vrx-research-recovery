#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <sdf/Element.hh>

namespace vrx_csd_shadow
{
struct CurrentObject
{
  std::string linkName;
  gz::sim::Link link;
};

class CsdOceanCurrent final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public: void Configure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    this->currentWorld = _sdf->Get<gz::math::Vector3d>(
      "current_velocity", gz::math::Vector3d::Zero).first;
    this->xU = _sdf->Get<double>("xU", 100.0).first;
    this->xUU = _sdf->Get<double>("xUU", 150.0).first;
    this->yV = _sdf->Get<double>("yV", 100.0).first;
    this->yVV = _sdf->Get<double>("yVV", 100.0).first;

    auto sdf = _sdf->Clone();
    if (!sdf->HasElement("current_obj"))
    {
      gzerr << "CsdOceanCurrent requires at least one <current_obj>\n";
      return;
    }

    for (auto elem = sdf->GetElement("current_obj"); elem;
         elem = elem->GetNextElement("current_obj"))
    {
      CurrentObject object;
      object.linkName = elem->Get<std::string>("link_name");
      this->objects.push_back(object);
    }

    gzmsg << "CSD ocean current configured: velocity=" << this->currentWorld
          << " m/s, objects=" << this->objects.size() << "\n";
  }

  public: void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    for (auto &object : this->objects)
    {
      if (!object.link.Valid(_ecm))
      {
        const auto entities = gz::sim::entitiesFromScopedName(
          object.linkName, _ecm);
        if (entities.empty())
          continue;

        object.link = gz::sim::Link(*entities.begin());
        object.link.EnableVelocityChecks(_ecm);
      }

      const auto worldVelocity = object.link.WorldLinearVelocity(_ecm);
      const auto worldPose = object.link.WorldPose(_ecm);
      if (!worldVelocity || !worldPose)
        continue;

      const auto bodyVelocity =
        worldPose->Rot().Inverse().RotateVector(*worldVelocity);
      const auto currentBody =
        worldPose->Rot().Inverse().RotateVector(this->currentWorld);
      const auto relativeWater = currentBody - bodyVelocity;

      // SimpleHydrodynamics already applies drag for still water. Add only
      // the correction needed to obtain drag based on velocity relative to
      // the moving water, so a zero current produces exactly zero correction.
      const double desiredX =
        (this->xU + this->xUU * std::abs(relativeWater.X())) *
        relativeWater.X();
      const double stillX =
        -(this->xU + this->xUU * std::abs(bodyVelocity.X())) *
        bodyVelocity.X();
      const double desiredY =
        (this->yV + this->yVV * std::abs(relativeWater.Y())) *
        relativeWater.Y();
      const double stillY =
        -(this->yV + this->yVV * std::abs(bodyVelocity.Y())) *
        bodyVelocity.Y();

      const gz::math::Vector3d correctionBody(
        desiredX - stillX, desiredY - stillY, 0.0);
      const auto correctionWorld =
        worldPose->Rot().RotateVector(correctionBody);
      object.link.AddWorldForce(_ecm, correctionWorld);
    }
  }

  private: std::vector<CurrentObject> objects;
  private: gz::math::Vector3d currentWorld{0.0, 0.0, 0.0};
  private: double xU{100.0};
  private: double xUU{150.0};
  private: double yV{100.0};
  private: double yVV{100.0};
};
}

GZ_ADD_PLUGIN(
  vrx_csd_shadow::CsdOceanCurrent,
  gz::sim::System,
  vrx_csd_shadow::CsdOceanCurrent::ISystemConfigure,
  vrx_csd_shadow::CsdOceanCurrent::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  vrx_csd_shadow::CsdOceanCurrent,
  "vrx_csd_shadow::CsdOceanCurrent")
