#include "acForceGen-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

AcForceGen::AcForceGen(std::string const& name) : TaskContext(name){
  std::cout << "AcForceGen constructed !" <<std::endl;
}

bool AcForceGen::configureHook(){
  std::cout << "AcForceGen configured !" <<std::endl;
  return true;
}

bool AcForceGen::startHook(){
  std::cout << "AcForceGen started !" <<std::endl;
  return true;
}

void AcForceGen::updateHook(){
  std::cout << "AcForceGen executes updateHook !" <<std::endl;
}

void AcForceGen::stopHook() {
  std::cout << "AcForceGen executes stopping !" <<std::endl;
}

void AcForceGen::cleanupHook() {
  std::cout << "AcForceGen cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(AcForceGen)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(AcForceGen)
