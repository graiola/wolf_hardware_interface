/// \author: Michele Focchi

#ifndef HARDWARE_INTERFACE_CONTACT_SWITCH_SENSOR_INTERFACE_H
#define HARDWARE_INTERFACE_CONTACT_SWITCH_SENSOR_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>


namespace hardware_interface
{

/// A handle used to read the state of a force-torque sensor.
class ContactSwitchSensorHandle
{
public:
    ContactSwitchSensorHandle() : name_(""), contact_(NULL), force_(NULL), torque_(NULL), normal_(NULL) {}

    /**
     * \param name The name of the sensor
     * \param contact A pointer to the storage of the contact value
     */
    ContactSwitchSensorHandle(const std::string& name,
                              bool* contact,
                              double* force,
                              double* torque,
                              double* normal) :
                                  name_(name),
                                  contact_(contact),
                                  force_(force),
                                  torque_(torque),
                                  normal_(normal){}

    std::string getName() const {return name_;}
    const bool* getContactState() const {return contact_;}
    const double* getForce() const {return force_;}
    const double* getTorque() const {return torque_;}
    const double* getNormal() const {return normal_;}

private:
    std::string name_;
    bool* contact_;
    double* force_;
    double* torque_;
    double* normal_;
};

/** \brief Hardware interface to support reading the state of a force-torque sensor. */
class ContactSwitchSensorInterface : public HardwareResourceManager<ContactSwitchSensorHandle> {};

}

#endif // HARDWARE_INTERFACE_CONTACT_SWITCH_SENSOR_INTERFACE_H
