#ifndef GAZEBO_INITIAL_MODEL_CONFIGURATION
#define GAZEBO_INITIAL_MODEL_CONFIGURATION

#include <iostream>
#include <map>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <ros/package.h>

namespace gazebo {

//
// Note:
//   This should be a world plugin, not a model plugin
//   because actual joint names are unknown at the layer of model description.
//
//   ex.
//   [robot.sdf]
//      <model name="robot">
//          ...
//          <plugin name="a_model_plugin">
//              <joint>
//                  <name>a_joint</name>
//                  <position>3.14</position>
//              </joint>
//          </plugin>
//      </model>
//
//   [super_robot.sdf]
//      <model name="super_robot">
//          <!-- the model plugin cannot find a_joint               -->
//          <!-- because its name becomes embedded_robot::a_joint ! -->
//          <include>
//              <name>embedded_robot</name>
//              <uri>model://robot</uri>
//          </include>
//          ...
//      </model>
//

class InitialModelConfiguration : public WorldPlugin {
public:
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override {
    // assert the given sdf matches the plugin format.
    const sdf::ElementPtr formatted_sdf(FormatAsPluginSDF(_sdf));

    const std::string plugin_name(formatted_sdf->GetAttribute("name")->GetAsString());
    std::cout << "[" << plugin_name << "]:"
              << " Start loading plugin" << std::endl;

    // find the model from the [model] element
#if GAZEBO_MAJOR_VERSION >= 8
    const physics::ModelPtr model(
        _world->ModelByName(formatted_sdf->GetElement("model")->Get< std::string >()));
#else
    const physics::ModelPtr model(
        _world->GetModel(formatted_sdf->GetElement("model")->Get< std::string >()));
#endif
    GZ_ASSERT(model, "Cannot find a model with the value of [model] element");
    std::cout << "[" << plugin_name << "]:"
              << " Found the target model \"" << model->GetScopedName() << "\"" << std::endl;

    // load joint map (name -> position) from the [joint] elements
    std::map< std::string, double > joint_positions;
    for (sdf::ElementPtr joint_elem = formatted_sdf->GetElement("joint"); joint_elem;
         joint_elem = joint_elem->GetNextElement("joint")) {
      joint_positions.insert(std::make_pair(joint_elem->GetElement("name")->Get< std::string >(),
                                            joint_elem->GetElement("position")->Get< double >()));
    }

    // assert each given joint name points a unique joint in the model
    const std::map< std::string, physics::JointPtr > existing_joints(
        model->GetJointController()->GetJoints());
    for (const std::map< std::string, double >::value_type &given_joint : joint_positions) {
      std::size_t n_found(0);
      for (const std::map< std::string, physics::JointPtr >::value_type &existing_joint :
           existing_joints) {
        if (given_joint.first == existing_joint.second->GetName()) {
          ++n_found;
        }
      }
      GZ_ASSERT(n_found >= 1 , "A given joint does not exist");
      GZ_ASSERT(n_found <= 1, "A given joint name is ambiguous");
      std::cout << "[" << plugin_name << "]:"
                << " Will set the position of joint \"" << given_joint.first << "\" to "
                << given_joint.second << std::endl;
    }

    // set joint positions
    model->SetJointPositions(joint_positions);

    // done!!
    std::cout << "[" << plugin_name << "]:"
              << " Loaded plugin" << std::endl;
  }

private:
  // get a sdf element which has been initialized by the plugin format file.
  // the initialied sdf may look empty but have a format information.
  static sdf::ElementPtr InitializedPluginSDF() {
    const sdf::ElementPtr sdf(new sdf::Element());
    GZ_ASSERT(sdf::initFile(ros::package::getPath("gazebo_initial_model_configuration") +
                                "/sdf/initial_model_configuration_plugin.sdf",
                            sdf),
              "Cannot initialize sdf by initial_model_configuration_plugin.sdf");
    return sdf;
  }

  // merge the plugin format sdf and the given sdf.
  // assert if the given sdf does not match the format
  // (ex. no required element, value type mismatch, ...).
  static sdf::ElementPtr FormatAsPluginSDF(const sdf::ElementPtr &_src) {
    static const sdf::ElementPtr fmt(InitializedPluginSDF());
    const sdf::ElementPtr dst(fmt->Clone());
    GZ_ASSERT(
        sdf::readString("<sdf version='" SDF_VERSION "'>" + _src->ToString("") + "</sdf>", dst),
        "The given sdf does not match InitialModelConfiguration plugin format");
    return dst;
  }
};

} // namespace gazebo

#endif