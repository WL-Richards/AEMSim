#pragma once

#include <IEventReceiver.h>
#include <chrono/core/ChVector3.h>

namespace irr
{
    namespace core {
        template <class T> class vector3d;
        using vector3df = vector3d<float>;
    }
    
    namespace scene
    {
        class ICameraSceneNode;
    }

    struct SEvent;
}

namespace chrono::irrlicht
{
    class ChVisualSystemIrrlicht;
}

namespace chrono::irrlicht { class ChVisualSystemIrrlicht; }

class OrbitFieldCameraController : public irr::IEventReceiver
{
public:
    struct Params {
        // Mouse sensitivities
        double orbit_sens = 0.005;   // radians per pixel
        double pan_sens   = 0.002;   // world units per pixel per distance
        double zoom_sens  = 0.12;    // exponential zoom per wheel notch

        // Keyboard speeds (world units / second or radians / second)
        double move_speed = 3.0;     // WASD (in-plane)
        double lift_speed = 3.0;     // E/Q
        double rot_speed  = 1.5;     // IJKL

        // Limits
        double min_dist = 0.25;
        double max_dist = 200.0;
        double min_pitch = -1.45;   // ~ -83 degrees
        double max_pitch =  1.45;   // ~ +83 degrees
    };


    OrbitFieldCameraController(std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> vis,
                               const chrono::ChVector3d& initial_target = chrono::ChVector3d(0, 0, 0),
                               double initial_distance = 6.0,
                               double initial_yaw = 0.8,
                               double initial_pitch = 0.55,
                               Params params = Params());

    void Update(double dt);

    // Let callers re-center / retarget easily.
    void SetTarget(const chrono::ChVector3d& target);
    const chrono::ChVector3d& GetTarget() const;

    void SetDistance(double d);
    double GetDistance() const;

    void SetYawPitch(double yaw, double pitch);

    bool OnEvent(const irr::SEvent& event) override;
    
private:
    std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> m_vis;
    irr::scene::ICameraSceneNode* m_cam = nullptr;

    chrono::ChVector3d m_target;
    double m_distance = 6.0;
    double m_yaw = 0.0;    // radians
    double m_pitch = 0.5;  // radians

    Params m_params;

    // Input state
    bool m_left_down = false;
    bool m_right_down = false;
    
    int lastMouseX = 0;
    int lastMouseY = 0;

    bool m_key_w=false, m_key_a=false, m_key_s=false, m_key_d=false;
    bool m_key_q=false, m_key_e=false;
    bool m_key_i=false, m_key_j=false, m_key_k=false, m_key_l=false;


    void ensureCameraExists();

    bool handleMouse(const irr::SEvent::SMouseInput& m);

    bool handleKey(const irr::SEvent::SKeyInput& k);

    void clampState();

    chrono::ChVector3d getForwardVector() const;

    chrono::ChVector3d computeCameraPos() const;

    void pan(int dx_pixels, int dy_pixels);

    void applyToCamera();

    static irr::core::vector3df ToIrr(const chrono::ChVector3d& v);
};
