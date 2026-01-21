#pragma once
#include <chrono>
#include <cstdint>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include <chrono/physics/ChSystem.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#include "../interfaces/Stepable.h"

namespace irr::video { class IVideoDriver; }

class SimTelemetryHUD : public Stepable
{
public:
    struct Style {
        irr::core::position2di pos_px{10, 10};   // top-left corner
        irr::core::dimension2du size_px{150, 100};
        irr::video::SColor background{160, 0, 0, 0}; // semi-transparent black
        irr::video::SColor border{220, 255, 255, 255};
        irr::video::SColor text{255, 255, 255, 255};
        int padding_px = 8;
        bool draw_background = true;
        bool draw_border = true;
    };

    SimTelemetryHUD(std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> vis,
                    std::shared_ptr<chrono::ChSystem> sys,
                    double expected_step = 0.0)
        : m_vis(vis), m_sys(sys), m_expected_step(expected_step) {
        Reset();
    }
    void Reset() {
        m_step_index = 0;
        m_sim_t0 = m_sys->GetChTime();
        m_sim_t_prev = m_sim_t0;

        m_wall_t0 = Clock::now();
        m_wall_prev = m_wall_t0;

        m_rtf_smoothed = 0.0;
        m_last_fps = 0;
    }

    // Optional configuration
    void SetExpectedStep(double step) { m_expected_step = step; }
    void SetStyle(const Style& s) { m_style = s; }
    Style& GetStyle() { return m_style; }
    const Style& GetStyle() const { return m_style; }

    void SetEnabled(bool v) { m_enabled = v; }
    bool IsEnabled() const { return m_enabled; }

    // ---------------- Stepable ----------------
    void DoPhysicsStepBackward() {
        if (!m_enabled) return;
        if (m_step_index > 0) {
            --m_step_index;
        }
        // Update sim time tracking to match restored state
        m_sim_t_prev = m_sys->GetChTime();
        m_wall_prev = Clock::now();
    }

    void DoPhysicsStep() override {
        if (!m_enabled) return;

        ++m_step_index;

        const double sim_now = m_sys->GetChTime();
        const auto wall_now = Clock::now();

        const double sim_dt = std::max(0.0, sim_now - m_sim_t_prev);
        const double wall_dt = Seconds(wall_now - m_wall_prev);

        // Instant RTF over the last step (guard divide-by-zero)
        const double rtf_inst = (wall_dt > 1e-12) ? (sim_dt / wall_dt) : 0.0;

        // Exponential smoothing (nice, stable readout)
        // Alpha chosen to feel responsive without flickering.
        constexpr double alpha = 0.08;
        if (m_step_index == 1) {
            m_rtf_smoothed = rtf_inst;
        } else {
            m_rtf_smoothed = (1.0 - alpha) * m_rtf_smoothed + alpha * rtf_inst;
        }

        m_sim_dt_last = sim_dt;
        m_wall_dt_last = wall_dt;

        m_sim_t_prev = sim_now;
        m_wall_prev = wall_now;
    }

    void DoRenderStep(irr::video::IVideoDriver* drv) override {
        if (!m_enabled || !drv) return;

        // Grab a fixed-size font provided by Chrono's Irrlicht vis system.
        // (Chrono exposes GetMonospaceFont()).
        irr::gui::IGUIFont* font = m_vis->GetMonospaceFont();
        if (!font) {
            // Fallback: Irrlicht built-in font if available
            if (auto* dev = m_vis->GetDevice()) {
                font = dev->getGUIEnvironment()->getBuiltInFont();
            }
        }
        if (!font) return;

        // Update render FPS (Irrlicht reports averaged FPS here).
        m_last_fps = drv->getFPS();

        const double sim_now = m_sys->GetChTime();
        const double sim_elapsed = std::max(0.0, sim_now - m_sim_t0);

        const auto wall_now = Clock::now();
        const double wall_elapsed = std::max(0.0, Seconds(wall_now - m_wall_t0));

        const double rtf_overall = (wall_elapsed > 1e-12) ? (sim_elapsed / wall_elapsed) : 0.0;

        // Build text (wide string for Irrlicht font draw)
        std::wostringstream w;
        w.setf(std::ios::fixed);

        w << L"Chrono Telemetry\n";
        w << L"Render FPS: " << m_last_fps << L"\n";

        w << L"Step #:     " << static_cast<unsigned long long>(m_step_index) << L"\n";

        w << std::setprecision(3);
        w << L"Sim t:      " << sim_now << L" s\n";
        w << L"Wall t:     " << wall_elapsed << L" s\n";

        w << std::setprecision(3);
        w << L"RTF (inst): " << m_rtf_smoothed << L"\n";
        w << L"RTF (avg):  " << rtf_overall << L"\n";

        if (m_expected_step > 0.0) {
            w << std::setprecision(5);
            w << L"dt:         " << m_sim_dt_last
              << L" s (exp " << m_expected_step << L")\n";
        } else {
            w << std::setprecision(5);
            w << L"dt:         " << m_sim_dt_last << L" s\n";
        }

        // Draw panel
        const auto tl = m_style.pos_px;
        const auto sz = m_style.size_px;
        irr::core::rect<irr::s32> r(tl.X, tl.Y, tl.X + (irr::s32)sz.Width, tl.Y + (irr::s32)sz.Height);

        if (m_style.draw_background) {
            drv->draw2DRectangle(m_style.background, r);
        }
        if (m_style.draw_border) {
            // simple border: 4 thin rectangles
            const int t = 1;
            drv->draw2DRectangle(m_style.border, irr::core::rect<irr::s32>(r.UpperLeftCorner.X, r.UpperLeftCorner.Y,
                                                                          r.LowerRightCorner.X, r.UpperLeftCorner.Y + t));
            drv->draw2DRectangle(m_style.border, irr::core::rect<irr::s32>(r.UpperLeftCorner.X, r.LowerRightCorner.Y - t,
                                                                          r.LowerRightCorner.X, r.LowerRightCorner.Y));
            drv->draw2DRectangle(m_style.border, irr::core::rect<irr::s32>(r.UpperLeftCorner.X, r.UpperLeftCorner.Y,
                                                                          r.UpperLeftCorner.X + t, r.LowerRightCorner.Y));
            drv->draw2DRectangle(m_style.border, irr::core::rect<irr::s32>(r.LowerRightCorner.X - t, r.UpperLeftCorner.Y,
                                                                          r.LowerRightCorner.X, r.LowerRightCorner.Y));
        }

        // Text area (padding)
        irr::core::rect<irr::s32> tr(
            r.UpperLeftCorner.X + m_style.padding_px,
            r.UpperLeftCorner.Y + m_style.padding_px,
            r.LowerRightCorner.X - m_style.padding_px,
            r.LowerRightCorner.Y - m_style.padding_px
        );

        font->draw(w.str().c_str(), tr, m_style.text, false /*hcenter*/, false /*vcenter*/, &r /*clip*/);
    }

private:
    using Clock = std::chrono::steady_clock;

    static double Seconds(const Clock::duration& d) {
        return std::chrono::duration<double>(d).count();
    }

private:
    std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> m_vis;
    std::shared_ptr<chrono::ChSystem> m_sys;

    bool m_enabled = true;
    double m_expected_step = 0.0;

    // State
    std::uint64_t m_step_index = 0;

    double m_sim_t0 = 0.0;
    double m_sim_t_prev = 0.0;

    Clock::time_point m_wall_t0{};
    Clock::time_point m_wall_prev{};

    double m_sim_dt_last = 0.0;
    double m_wall_dt_last = 0.0;

    double m_rtf_smoothed = 0.0;
    int m_last_fps = 0;

    Style m_style{};
    
};
