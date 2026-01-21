#include "DebugDrawer.h"

#include <algorithm>
#include <cmath>

#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
#include <irrlicht.h>

#include "AMath.h"

namespace irr::gui
{
    class IGUIFont;
    class IGUIEnvironment;
}

namespace
{
    void DrawArrow3D(irr::video::IVideoDriver* driver,
                     const irr::core::vector3df& start,
                     const irr::core::vector3df& end,
                     const irr::video::SColor& color,
                     float head_length_m,
                     float head_width_m)
    {
        irr::core::vector3df dir = end - start;
        const float length = dir.getLength();
        if (length <= 1e-6f) return;

        dir /= length;

        irr::core::vector3df up(0.0f, 0.0f, 1.0f);
        if (std::abs(dir.dotProduct(up)) > 0.98f) {
            up = irr::core::vector3df(0.0f, 1.0f, 0.0f);
        }

        irr::core::vector3df perp = dir.crossProduct(up);
        perp.normalize();

        const float head_len = std::min(head_length_m, length * 0.5f);
        const irr::core::vector3df head_base = end - dir * head_len;
        const irr::core::vector3df left = head_base + perp * head_width_m;
        const irr::core::vector3df right = head_base - perp * head_width_m;

        driver->draw3DLine(start, end, color);
        driver->draw3DLine(end, left, color);
        driver->draw3DLine(end, right, color);
    }

    void DrawStick3D(irr::video::IVideoDriver* driver,
                     const chrono::ChVector3d& origin,
                     const chrono::ChVector3d& axis_dir,
                     double length_m,
                     double major_step_m,
                     double minor_step_m,
                     double tick_len_m,
                     const irr::video::SColor& color,
                     bool depth_test,
                     bool use_special_tick,
                     double special_tick_m,
                     const irr::video::SColor& special_color)
    {
        if (!driver) return;
        if (length_m <= 0.0 || minor_step_m <= 0.0) return;

        const double dir_len = std::sqrt(axis_dir.x() * axis_dir.x() +
                                         axis_dir.y() * axis_dir.y() +
                                         axis_dir.z() * axis_dir.z());
        if (dir_len <= 1e-9) return;

        const chrono::ChVector3d dir(axis_dir.x() / dir_len,
                                     axis_dir.y() / dir_len,
                                     axis_dir.z() / dir_len);

        irr::video::SMaterial mat;
        mat.Lighting = false;
        mat.ZBuffer = depth_test ? irr::video::ECFN_LESSEQUAL : irr::video::ECFN_ALWAYS;
        mat.ZWriteEnable = true;
        mat.BackfaceCulling = false;
        mat.MaterialType = irr::video::EMT_SOLID;
        driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
        driver->setMaterial(mat);

        const irr::core::vector3df start(
            (irr::f32)origin.x(),
            (irr::f32)origin.y(),
            (irr::f32)origin.z()
        );
        const irr::core::vector3df end(
            (irr::f32)(origin.x() + dir.x() * length_m),
            (irr::f32)(origin.y() + dir.y() * length_m),
            (irr::f32)(origin.z() + dir.z() * length_m)
        );
        driver->draw3DLine(start, end, color);

        irr::core::vector3df up(0.0f, 0.0f, 1.0f);
        const irr::core::vector3df dir_f(
            (irr::f32)dir.x(), (irr::f32)dir.y(), (irr::f32)dir.z());
        if (std::abs(dir_f.dotProduct(up)) > 0.98f) {
            up = irr::core::vector3df(0.0f, 1.0f, 0.0f);
        }
        irr::core::vector3df perp = dir_f.crossProduct(up);
        perp.normalize();

        const int steps = (int)std::floor(length_m / minor_step_m + 1e-6);
        for (int i = 0; i <= steps; ++i) {
            const double t = i * minor_step_m;
            const bool is_major = (major_step_m > 0.0) &&
                                  (std::abs(std::fmod(t, major_step_m)) <= 1e-6);
            const float scale = is_major ? 1.6f : 1.0f;
            const float half_tick = (irr::f32)(tick_len_m * 0.5 * scale);

            const irr::core::vector3df pos(
                (irr::f32)(origin.x() + dir.x() * t),
                (irr::f32)(origin.y() + dir.y() * t),
                (irr::f32)(origin.z() + dir.z() * t)
            );
            const irr::core::vector3df a = pos - perp * half_tick;
            const irr::core::vector3df b = pos + perp * half_tick;
            const bool is_special = use_special_tick &&
                                    (std::abs(t - special_tick_m) <= (minor_step_m * 0.5));
            driver->draw3DLine(a, b, is_special ? special_color : color);
        }
    }

}

void DebugDrawer::DrawGridXY(irr::video::IVideoDriver* driver, float z_plane_height, float half_extent_m,
                             float minor_step_m, int major_every, const irr::video::SColor& minor_color, const irr::video::SColor& major_color,
                             const irr::video::SColor& y_axis_color, const irr::video::SColor& x_axis_color)
{
    if (!driver) return;
    if (minor_step_m <= 0 || half_extent_m <= 0) return;
    if (major_every < 1) major_every = 1;

    SetDebugDriverMaterial(driver);

    const int N = (int)floorf(half_extent_m / minor_step_m);
    const float maxv = N * minor_step_m;

    for (int i = -N; i <= N; i++) {
        const float v = i * minor_step_m;
        const bool is_major = (i % major_every) == 0;

        // Lines parallel to X (varying Y)
        {
            irr::video::SColor c = is_major ? major_color : minor_color;
            if (i == 0) c = y_axis_color;  // y=0 line (X axis in plane)
            driver->draw3DLine(
                irr::core::vector3df(-maxv, v, z_plane_height),
                irr::core::vector3df( maxv, v, z_plane_height),
                c
            );
        }

        // Lines parallel to Y (varying X)
        {
            irr::video::SColor c = is_major ? major_color : minor_color;
            if (i == 0) c = x_axis_color;  // x=0 line (Y axis in plane)
            driver->draw3DLine(
                irr::core::vector3df(v, -maxv, z_plane_height),
                irr::core::vector3df(v,  maxv, z_plane_height),
                c
            );
        }
    }
}

void DebugDrawer::DrawVector3dAxisArrows(irr::video::IVideoDriver* driver,
                                         const chrono::ChVector3d& vec,
                                         const irr::video::SColor& color,
                                         const chrono::ChVector3d& origin,
                                         float head_length_m,
                                         float head_width_m,
                                         bool draw_components)
{
    if (!driver) return;

    SetDebugDriverMaterial(driver);

    const irr::core::vector3df start(
        (irr::f32)origin.x(),
        (irr::f32)origin.y(),
        (irr::f32)origin.z()
    );

    const irr::core::vector3df x_end(
        (irr::f32)(origin.x() + vec.x()),
        (irr::f32)origin.y(),
        (irr::f32)origin.z()
    );

    const irr::core::vector3df y_end(
        (irr::f32)(origin.x() + vec.x()),
        (irr::f32)(origin.y() + vec.y()),
        (irr::f32)origin.z()
    );

    const irr::core::vector3df z_end(
        (irr::f32)(origin.x() + vec.x()),
        (irr::f32)(origin.y() + vec.y()),
        (irr::f32)(origin.z() + vec.z())
    );

    if (!draw_components) {
        DrawArrow3D(driver, start, z_end, color, head_length_m, head_width_m);
        return;
    }

    DrawArrow3D(driver, start, x_end, irr::video::SColor(255, 255, 0, 0), head_length_m, head_width_m);
    DrawArrow3D(driver, x_end, y_end, irr::video::SColor(255, 0, 255, 0), head_length_m, head_width_m);
    DrawArrow3D(driver, y_end, z_end, irr::video::SColor(255, 0, 0, 255), head_length_m, head_width_m);
}

void DebugDrawer::DrawMeterStick3D(irr::video::IVideoDriver* driver,
                                   const chrono::ChVector3d& origin,
                                   const chrono::ChVector3d& axis_dir,
                                   const irr::video::SColor& color,
                                   bool depth_test)
{
    DrawStick3D(driver, origin, axis_dir,
                /*length_m=*/1.0,
                /*major_step_m=*/0.1,
                /*minor_step_m=*/0.05,
                /*tick_len_m=*/0.08,
                color, depth_test,
                /*use_special_tick=*/false,
                /*special_tick_m=*/0.0,
                /*special_color=*/color);
}

void DebugDrawer::DrawYardStick3D(irr::video::IVideoDriver* driver,
                                  const chrono::ChVector3d& origin,
                                  const chrono::ChVector3d& axis_dir,
                                  const irr::video::SColor& color,
                                  bool depth_test)
{
    DrawStick3D(driver, origin, axis_dir,
                /*length_m=*/0.9144,
                /*major_step_m=*/0.3048,
                /*minor_step_m=*/0.0762,
                /*tick_len_m=*/0.08,
                color, depth_test,
                /*use_special_tick=*/true,
                /*special_tick_m=*/0.3683,
                /*special_color=*/irr::video::SColor(255, 0, 0, 255));
}

void DebugDrawer::DrawPolyline3D(irr::video::IVideoDriver* driver, const std::vector<chrono::ChVector3d>& pts,
    const irr::video::SColor& color, bool depth_test)
{
    // Ensure our driver is valid and we have atleast 2 points to draw
    if (!driver) return;
    if (pts.size() < 2) return;

    // Reset transform so lines are in WORLD space
    driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());

    // Loop over and draw all points 
    for (size_t i = 1; i < pts.size(); ++i) {
        const chrono::ChVector3d& a = pts[i - 1];
        const chrono::ChVector3d& b = pts[i];

        driver->draw3DLine(
            irr::core::vector3df(
                (irr::f32)a.x(),
                (irr::f32)a.y(),
                (irr::f32)a.z()
            ),
            irr::core::vector3df(
                (irr::f32)b.x(),
                (irr::f32)b.y(),
                (irr::f32)b.z()
            ),
            color
        );
    }
}

void DebugDrawer::SetDebugDriverMaterial(irr::video::IVideoDriver* driver)
{
    if (!driver) return;
    driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());

    irr::video::SMaterial m;
    m.Lighting = false;
    m.ZBuffer = irr::video::ECFN_LESSEQUAL;
    m.ZWriteEnable = true;
    m.BackfaceCulling = false;
    m.MaterialType = irr::video::EMT_SOLID;
    driver->setMaterial(m);
}

void DebugDrawer::DrawMeterLabelsXY(irr::IrrlichtDevice* device,
                                 irr::scene::ISceneManager* smgr,
                                 float z_plane,
                                 float half_extent_m,
                                 float step_m,
                                 float scale,
                                 const irr::video::SColor& color,
                                 bool label_x_axis,
                                 bool label_y_axis)
{
        if (!device || !smgr) return;
        if (step_m <= 0 || half_extent_m <= 0) return;

        irr::gui::IGUIEnvironment* guienv = device->getGUIEnvironment();
        if (!guienv) return;

        // Built-in font can be small; but billboard text node scales with world size.
        irr::gui::IGUIFont* font = nullptr;
        if (auto* skin = guienv->getSkin()) {
            font = skin->getFont();
        }
        if (!font) {
            font = guienv->getBuiltInFont();
        }
        if (!font) return;

        const int N = (int)std::floor(half_extent_m / step_m);

        if (label_x_axis) {
            for (int i = -N; i <= N; i++) {
                if (i == 0) continue;
                const float x = i * (float)step_m;

                wchar_t buf[32];
                swprintf(buf, sizeof(buf) / sizeof(wchar_t), L"%.0fm", (double)i * step_m);
                auto* node = smgr->addBillboardTextSceneNode(
                    font,
                    buf,
                    nullptr,
                    irr::core::dimension2df(-scale, -scale),
                    irr::core::vector3df(x, 0.0f, z_plane)
                );
                if (node) {
                    node->setColor(color);
                    node->setMaterialFlag(irr::video::EMF_LIGHTING, false);
                    node->setMaterialFlag(irr::video::EMF_ZWRITE_ENABLE, true);
                    node->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, false);
                    node->setMaterialType(irr::video::EMT_TRANSPARENT_ALPHA_CHANNEL);
                    node->setRotation(irr::core::vector3df(0.f, 0.f, 180.f));
                }
            }

            const float half_step = (float)step_m * 0.5f;
            const float minor_scale = (-scale) * 0.6f;
            for (int i = -N; i < N; i++) {
                const float x = (i * (float)step_m) + half_step;
                if (std::abs(x) < 1e-6f) continue;

                wchar_t buf[32];
                swprintf(buf, sizeof(buf) / sizeof(wchar_t), L"%.1f m", (double)x);
                auto* node = smgr->addBillboardTextSceneNode(
                    font,
                    buf,
                    nullptr,
                    irr::core::dimension2df(minor_scale, minor_scale),
                    irr::core::vector3df(x, 0.0f, z_plane)
                );
                if (node) {
                    node->setColor(color);
                    node->setMaterialFlag(irr::video::EMF_LIGHTING, false);
                    node->setMaterialFlag(irr::video::EMF_ZWRITE_ENABLE, true);
                    node->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, false);
                    node->setMaterialType(irr::video::EMT_TRANSPARENT_ALPHA_CHANNEL);
                    node->setRotation(irr::core::vector3df(0.f, 0.f, 180.f));
                }
            }
        }

        if (label_y_axis) {
            for (int i = -N; i <= N; i++) {
                if (i == 0) continue;
                const float y = i * (float)step_m;

                wchar_t buf[32];
                swprintf(buf, sizeof(buf) / sizeof(wchar_t), L"%.0fm", (double)i * step_m);
                auto* node = smgr->addBillboardTextSceneNode(
                    font,
                    buf,
                    nullptr,
                    irr::core::dimension2df(-scale, -scale),
                    irr::core::vector3df(0.0f, y, z_plane)
                );
                if (node) {
                    node->setColor(color);
                    node->setMaterialFlag(irr::video::EMF_LIGHTING, false);
                    node->setMaterialFlag(irr::video::EMF_ZWRITE_ENABLE, true);
                    node->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, false);
                    node->setMaterialType(irr::video::EMT_TRANSPARENT_ALPHA_CHANNEL);
                    node->setRotation(irr::core::vector3df(0.f, 0.f, 180.f));
                }
            }

            const float half_step = (float)step_m * 0.5f;
            const float minor_scale = (-scale) * 0.6f;
            for (int i = -N; i < N; i++) {
                const float y = (i * (float)step_m) + half_step;
                if (std::abs(y) < 1e-6f) continue;

                wchar_t buf[32];
                swprintf(buf, sizeof(buf) / sizeof(wchar_t), L"%.1f m", (double)y);
                auto* node = smgr->addBillboardTextSceneNode(
                    font,
                    buf,
                    nullptr,
                    irr::core::dimension2df(minor_scale, minor_scale),
                    irr::core::vector3df(0.0f, y, z_plane)
                );
                if (node) {
                    node->setColor(color);
                    node->setMaterialFlag(irr::video::EMF_LIGHTING, false);
                    node->setMaterialFlag(irr::video::EMF_ZWRITE_ENABLE, true);
                    node->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, false);
                    node->setMaterialType(irr::video::EMT_TRANSPARENT_ALPHA_CHANNEL);
                    node->setRotation(irr::core::vector3df(0.f, 0.f, 180.f));
                }
            }
        }
}

void DebugDrawer::DrawHorizontalPlane(irr::video::IVideoDriver* driver,
                                      const chrono::ChVector3d& center,
                                      double half_size,
                                      const irr::video::SColor& color,
                                      bool depth_test)
{
    if (!driver) return;

    irr::video::SMaterial mat;
    mat.Lighting = false;
    mat.ZWriteEnable = true;
    mat.ZBuffer = depth_test ? irr::video::ECFN_LESSEQUAL : irr::video::ECFN_ALWAYS;
    mat.Thickness = 2.0f;
    driver->setMaterial(mat);
    driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());

    const float x = static_cast<float>(center.x());
    const float y = static_cast<float>(center.y());
    const float z = static_cast<float>(center.z());
    const float hs = static_cast<float>(half_size);

    // Draw a square outline at z height
    irr::core::vector3df corners[5] = {
        {x - hs, y - hs, z},
        {x + hs, y - hs, z},
        {x + hs, y + hs, z},
        {x - hs, y + hs, z},
        {x - hs, y - hs, z}  // close the loop
    };

    for (int i = 0; i < 4; ++i) {
        driver->draw3DLine(corners[i], corners[i + 1], color);
    }

    // Draw cross lines for better visibility
    driver->draw3DLine(corners[0], corners[2], color);
    driver->draw3DLine(corners[1], corners[3], color);
}
