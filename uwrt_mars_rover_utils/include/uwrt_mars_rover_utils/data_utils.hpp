#pragma once

namespace DifferentialTransmissionData {


namespace ActuatorData {

    struct CommandData {
        double velocity{};  // TODO: change this so that it can work with any interface type... add enum class that will hold
                            // the different command types
    };

    struct StateData {
        double velocity{};
        double position{};
        double iq_current{};  // not used for differential transmission
    };  

}  // namespace ActuatorData

// keeping this separate from ActuatorData despite the overlap for extensibility reasons
namespace JointData {

struct CommandData { double velocity{}; };

struct StateData {
  double velocity{};
  double position{};
};

}  // namespace JointData

}  // namespace DifferentialTransmissionData
