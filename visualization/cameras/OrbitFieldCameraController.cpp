#include "OrbitFieldCameraController.h"
#include <irrlicht.h>
#include <chrono/assets/ChVisualSystem.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

OrbitFieldCameraController::OrbitFieldCameraController(
                                                        std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> vis,
                                                        const Params& params,
                                                        const chrono::ChVector3d& initial_target, double initial_distance, double initial_yaw, double initial_pitch
                                                       ) : m_vis(vis),
                                                                        m_target(initial_target),
                                                                        m_distance(initial_distance),
                                                                        m_yaw(initial_yaw),
                                                                        m_pitch(initial_pitch),
                                                                        m_params(params) {
    clampState();
    ensureCameraExists();
    applyToCamera();
}

void OrbitFieldCameraController::Update(double dt)
{
    if (!m_cam)
        ensureCameraExists();
    if (!m_cam)
        return;

    // Build basis from current yaw/pitch
    chrono::ChVector3d forward = getForwardVector();
    chrono::ChVector3d up(0, 0, 1);
    chrono::ChVector3d right = Vcross(forward, up);
    double rlen = right.Length();
    if (rlen > 1e-12)
        right /= rlen;

    // Optional: constrain WASD to horizontal plane (field-style)
    chrono::ChVector3d forward_flat(forward.x(), forward.y(), 0.0);
    double flen = forward_flat.Length();
    if (flen > 1e-12)
        forward_flat /= flen;

    chrono::ChVector3d delta(0, 0, 0);

    if (m_key_w) delta += forward_flat * (m_params.move_speed * dt);
    if (m_key_s) delta -= forward_flat * (m_params.move_speed * dt);
    if (m_key_d) delta += right        * (m_params.move_speed * dt);
    if (m_key_a) delta -= right        * (m_params.move_speed * dt);
    if (m_key_e) delta += up           * (m_params.lift_speed * dt);
    if (m_key_q) delta -= up           * (m_params.lift_speed * dt);

    m_target += delta;

    if (m_key_i) m_pitch += m_params.rot_speed * dt;
    if (m_key_k) m_pitch -= m_params.rot_speed * dt;
    if (m_key_l) m_yaw   += m_params.rot_speed * dt;
    if (m_key_j) m_yaw   -= m_params.rot_speed * dt;

    clampState();
    applyToCamera();
}

void OrbitFieldCameraController::SetTarget(const chrono::ChVector3d& target)
{
    m_target = target;
    applyToCamera();
}

const chrono::ChVector3d& OrbitFieldCameraController::GetTarget() const
{
    return m_target;
}

void OrbitFieldCameraController::SetDistance(double d)
{
    m_distance = d;
    clampState();
    applyToCamera();
}

double OrbitFieldCameraController::GetDistance() const
{
    return m_distance; 
}

void OrbitFieldCameraController::SetYawPitch(double yaw, double pitch)
{
    m_yaw = yaw; m_pitch = pitch;
    clampState();
    applyToCamera();
}

bool OrbitFieldCameraController::OnEvent(const irr::SEvent& event)
{
    if (event.EventType == irr::EET_MOUSE_INPUT_EVENT) {
        return handleMouse(event.MouseInput);
    }
    if (event.EventType == irr::EET_KEY_INPUT_EVENT) {
        return handleKey(event.KeyInput);
    }
    return false;
}

void OrbitFieldCameraController::ensureCameraExists()
{
    // Ensure Z-up viewing (field mode) using Chrono API.
    // ChVisualSystemIrrlicht exposes SetCameraVertical(CameraVerticalDir) :contentReference[oaicite:3]{index=3}
    m_vis->SetCameraVertical(chrono::CameraVerticalDir::Z);

    m_cam = m_vis->GetActiveCamera();  // available in Chrono 9.0 :contentReference[oaicite:5]{index=5}
    if (m_cam)
        return;

    // If no camera exists yet, create one via AddCamera(pos, targ) :contentReference[oaicite:6]{index=6}
    chrono::ChVector3d pos = computeCameraPos();
    m_vis->AddCamera(pos, m_target);
    m_cam = m_vis->GetActiveCamera(); 
}

bool OrbitFieldCameraController::handleMouse(const irr::SEvent::SMouseInput& m)
{
    switch (m.Event) {
    case irr::EMIE_LMOUSE_PRESSED_DOWN:
        m_left_down = true;
        lastMouseX = m.X;
        lastMouseY = m.Y;
        return true;
    case irr::EMIE_LMOUSE_LEFT_UP:
        m_left_down = false;
        return true;

    case irr::EMIE_RMOUSE_PRESSED_DOWN:
        m_right_down = true;
        lastMouseX = m.X;
        lastMouseY = m.Y;
        return true;
    case irr::EMIE_RMOUSE_LEFT_UP:
        m_right_down = false;
        return true;

    case irr::EMIE_MOUSE_MOVED: {
            const irr::core::position2di cur(m.X, m.Y);
           
            const int mouseXDelta = m.X - lastMouseX;
            const int mouseYDelta = m.Y - lastMouseY;
            lastMouseX = m.X;
            lastMouseY = m.Y;

            if (m_left_down) {
                // Orbit: left-drag rotates (AdvantageScope orbit field) :contentReference[oaicite:9]{index=9}
                m_yaw   -= mouseXDelta * m_params.orbit_sens;
                m_pitch -= mouseYDelta * m_params.orbit_sens;
                clampState();
                applyToCamera();
                return true;
            }

            if (m_right_down) {
                // Pan: right-drag :contentReference[oaicite:10]{index=10}
                pan(mouseXDelta, mouseYDelta);
                applyToCamera();
                return true;
            }

            return false;
    }

    case irr::EMIE_MOUSE_WHEEL: {
            // Zoom: scroll wheel :contentReference[oaicite:11]{index=11}
            // Irrlicht wheel is typically +/-1 per notch (float)
            const double factor = std::exp(-m.Wheel * m_params.zoom_sens);
            m_distance *= factor;
            clampState();
            applyToCamera();
            return true;
    }

    default:
        return false;
    }
}

bool OrbitFieldCameraController::handleKey(const irr::SEvent::SKeyInput& k)
{
    const bool down = k.PressedDown;

    switch (k.Key) {
        case irr::KEY_KEY_W: m_key_w = down; return false;
        case irr::KEY_KEY_A: m_key_a = down; return false;
        case irr::KEY_KEY_S: m_key_s = down; return false;
        case irr::KEY_KEY_D: m_key_d = down; return false;

        case irr::KEY_KEY_Q: m_key_q = down; return false;
        case irr::KEY_KEY_E: m_key_e = down; return false;

        case irr::KEY_KEY_I: m_key_i = down; return false;
        case irr::KEY_KEY_J: m_key_j = down; return false;
        case irr::KEY_KEY_K: m_key_k = down; return false;
        case irr::KEY_KEY_L: m_key_l = down; return false;

        default:
            return false;
    }
}

void OrbitFieldCameraController::clampState()
{
    m_distance = std::clamp(m_distance, m_params.min_dist, m_params.max_dist);
    m_pitch = std::clamp(m_pitch, m_params.min_pitch, m_params.max_pitch);

    // Keep yaw bounded (optional)
    const double two_pi = 2.0 * 3.14159265358979323846;
    if (m_yaw >  two_pi) m_yaw = std::fmod(m_yaw, two_pi);
    if (m_yaw < -two_pi) m_yaw = std::fmod(m_yaw, two_pi);
}

chrono::ChVector3d OrbitFieldCameraController::getForwardVector() const
{
    // Z-up spherical angles:
    // yaw around +Z, pitch about camera-right (handled by spherical formula)
    const double cp = std::cos(m_pitch);
    const double sp = std::sin(m_pitch);
    const double cy = std::cos(m_yaw);
    const double sy = std::sin(m_yaw);

    // forward points from camera toward target
    return {cp * cy, cp * sy, sp};
}

chrono::ChVector3d OrbitFieldCameraController::computeCameraPos() const
{
    chrono::ChVector3d forward = getForwardVector();
    return m_target - forward * m_distance;
}

void OrbitFieldCameraController::pan(int dx_pixels, int dy_pixels)
{
    chrono::ChVector3d forward = getForwardVector();
    chrono::ChVector3d up(0, 0, 1);
    chrono::ChVector3d right = Vcross(forward, up);
    double rlen = right.Length();
    if (rlen > 1e-12) right /= rlen;

    chrono::ChVector3d up2 = Vcross(right, forward);
    double ulen = up2.Length();
    if (ulen > 1e-12) up2 /= ulen;

    // Scale pan with distance so it feels consistent at different zoom levels
    const double scale = m_params.pan_sens * m_distance;

    // Screen space: drag right moves target right; drag up moves target up
    m_target += right * (-dx_pixels * scale);
    m_target += up2   * ( dy_pixels * scale);
}

void OrbitFieldCameraController::applyToCamera()
{
    if (!m_cam)
        ensureCameraExists();
    if (!m_cam)
        return;

    chrono::ChVector3d cam_pos = computeCameraPos();
    const chrono::ChVector3d up(0, 0, 1);

    m_cam->setPosition(ToIrr(cam_pos));
    m_cam->setTarget(ToIrr(m_target));
    m_cam->setUpVector(ToIrr(up));
}

irr::core::vector3df OrbitFieldCameraController::ToIrr(const chrono::ChVector3d& v)
{
    return irr::core::vector3df((irr::f32)v.x(), (irr::f32)v.y(), (irr::f32)v.z());
}
