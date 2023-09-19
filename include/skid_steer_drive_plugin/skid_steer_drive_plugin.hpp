#ifndef SKID_STEER_DRIVE_HPP_ 
#define SKID_STEER_DRIVE_HPP_ 

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo
{
class SkidSteerDrivePrivate;

class SkidSteerDrive : public gazebo::ModelPlugin
{
    public:
    SkidSteerDrive();

    ~SkidSteerDrive();

    protected:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void Reset() override;

    private:
    std::unique_ptr<SkidSteerDrivePrivate> impl_;


};
}


#endif