#include "DebugDrawer.h"

#include <algorithm>
#include <cmath>

#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

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
                                 float text_height_m,
                                 const irr::video::SColor& color,
                                 bool label_x_axis,
                                 bool label_y_axis)
{
        if (!device || !smgr) return;
        if (step_m <= 0 || half_extent_m <= 0) return;

        irr::gui::IGUIEnvironment* guienv = device->getGUIEnvironment();
        if (!guienv) return;

        // Built-in font can be small; but billboard text node scales with world size.
        irr::gui::IGUIFont* font = guienv->getBuiltInFont();
        if (!font) return;

        const int N = (int)std::floor(half_extent_m / step_m);

        if (label_x_axis) {
            for (int i = -N; i <= N; i++) {
                if (i == 0) continue;
                const float x = i * (float)step_m;

                wchar_t buf[32];
                swprintf(buf, sizeof(buf) / sizeof(wchar_t), L"%.0f m", (double)i * step_m);

                auto* node = smgr->addBillboardTextSceneNode(
                    font,
                    buf,
                    nullptr,
                    irr::core::dimension2df(text_height_m, text_height_m),
                    irr::core::vector3df(x, 0.0f, z_plane)
                );
                if (node) {
                    node->setColor(color);
                    node->setMaterialFlag(irr::video::EMF_LIGHTING, false);
                    node->setMaterialFlag(irr::video::EMF_ZWRITE_ENABLE, true);
                }
            }
        }

        if (label_y_axis) {
            for (int i = -N; i <= N; i++) {
                if (i == 0) continue;
                const float y = i * (float)step_m;

                wchar_t buf[32];
                swprintf(buf, sizeof(buf) / sizeof(wchar_t), L"%.0f m (%.0f ft)", (double)i * step_m, METERS_TO_FEET((double)i * step_m));

                auto* node = smgr->addBillboardTextSceneNode(
                    font,
                    buf,
                    nullptr,
                    irr::core::dimension2df(text_height_m+0.6f, text_height_m),
                    irr::core::vector3df(0.0f, y, z_plane)
                );
                if (node) {
                    node->setColor(color);
                    node->setMaterialFlag(irr::video::EMF_LIGHTING, false);
                    node->setMaterialFlag(irr::video::EMF_ZWRITE_ENABLE, true);

                    //node->setScale(irr::core::vector3df(-1.f,1.f,-1.f));
                }
            }
        }
    }
