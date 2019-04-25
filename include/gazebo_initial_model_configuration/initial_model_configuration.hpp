#ifndef GAZEBO_INITIAL_MODEL_CONFIGURATION
#define GAZEBO_INITIAL_MODEL_CONFIGURATION

#include <iostream>
#include <map>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

class InitialModelConfiguration : public WorldPlugin {
public:
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override {
    const std::string plugin_name(_sdf->GetAttribute("name")->GetAsString());

    std::cout << "[" << plugin_name << "]:"
              << " Start loading plugin" << std::endl;

    // find the model from the [model] element
    GZ_ASSERT(_sdf->HasElement("model"), "No [model] element");
    const physics::ModelPtr model(
        _world->ModelByName(_sdf->GetElement("model")->Get< std::string >()));
    GZ_ASSERT(model, "Cannot find a model with the value of [model] element");
    std::cout << "[" << plugin_name << "]:"
              << " Found the target model \"" << model->GetScopedName() << "\"" << std::endl;

    // load joint map (name -> position) from the [joint] elements
    std::map< std::string, double > joint_positions;
    GZ_ASSERT(_sdf->HasElement("joint"), "No [joint] element");
    for (sdf::ElementPtr e = _sdf->GetElement("joint"); e; e = e->GetNextElement("joint")) {
      GZ_ASSERT(e->HasElement("name"), "No [name] element under [joint] element");
      GZ_ASSERT(e->HasElement("position"), "No [position] element under [joint] element");
      const std::string name(e->GetElement("name")->Get< std::string >());
      const double position(e->GetElement("position")->Get< double >());
      joint_positions[name] = position;
      std::cout << "[" << plugin_name << "]:"
                << " Will set the position of joint \"" << name << "\" to " << position
                << std::endl;
    }

    // set joint positions
    model->SetJointPositions(joint_positions);

    // done!!
    std::cout << "[" << plugin_name << "]:"
              << " Loaded plugin" << std::endl;
  }
};

} // namespace gazebo

#endif