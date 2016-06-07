#ifndef OROCOS_ACFORCEGEN_COMPONENT_HPP
#define OROCOS_ACFORCEGEN_COMPONENT_HPP

#include <rtt/RTT.hpp>

class AcForceGen : public RTT::TaskContext{
  public:
    AcForceGen(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
