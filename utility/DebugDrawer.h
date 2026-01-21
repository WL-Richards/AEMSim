#pragma once
#include <vector>
#include <chrono/core/ChVector3.h>


namespace irr
{
    class IrrlichtDevice;
    
    namespace scene
    {
        class ISceneManager;
    }
    
    namespace video
    {
        class IVideoDriver;
        class SColor;
    }
}


// Defines for easier debug drawing, also set the driver material (wrapped in do {} while(0) so it behaves like one statement)
#define DrawDebugGridXY(driver, z_plane, half_extent_m, minor_step_m, major_every,  \
minor_color, major_color, x_axis_color, y_axis_color)                               \
    do {                                                                            \
        DebugDrawer::SetDebugDriverMaterial(driver);                                \
        DebugDrawer::DrawGridXY(driver, z_plane, half_extent_m, minor_step_m,       \
        major_every, minor_color, major_color,                                      \
        x_axis_color, y_axis_color);                                                \
    } while (0)


#define DrawDebugPolyline(driver, pts, color, depth_test)                          \
    do {                                                                           \
        DebugDrawer::SetDebugDriverMaterial(driver);                               \
        DebugDrawer::DrawPolyline3D(driver, pts, color, depth_test);               \
    } while (0)

#define DrawDebugMeterLabelsXY(device, smgr, z_plane, half_extent_m, step_m, scale, color, label_x_axis, label_y_axis) DebugDrawer::DrawMeterLabelsXY(device, smgr, z_plane, half_extent_m, step_m, scale, color, label_x_axis, label_y_axis)   

#define DrawDebugVector3DComponents(driver, vec) DebugDrawer::DrawVector3dAxisArrows(driver, vec, irr::video::SColor(255, 255, 255, 255), chrono::ChVector3d(0, 0, 0), 0.15f, 0.06f, true)
#define DrawDebugVector3DComponentsAtOrigin(driver, vec, origin) DebugDrawer::DrawVector3dAxisArrows(driver, vec, irr::video::SColor(255, 255, 255, 255), origin, 0.15f, 0.06f, true)

#define DrawDebugVector3D(driver, vec) DebugDrawer::DrawVector3dAxisArrows(driver, vec, irr::video::SColor(255, 255, 255, 255), chrono::ChVector3d(0, 0, 0), 0.15f, 0.06f, false)
#define DrawDebugVector3DColor(driver, vec, color) DebugDrawer::DrawVector3dAxisArrows(driver, vec, color, chrono::ChVector3d(0, 0, 0), 0.15f, 0.06f, false)
#define DrawDebugVector3DAtOrigin(driver, vec, origin) DebugDrawer::DrawVector3dAxisArrows(driver, vec, irr::video::SColor(255, 255, 255, 255), origin, 0.15f, 0.06f, false)
#define DrawDebugVector3DAtOriginColor(driver, vec, origin, color) DebugDrawer::DrawVector3dAxisArrows(driver, vec, color, origin, 0.15f, 0.06f, false)

// Draw a 1 meter or 1 yard stick with ticks, aligned to axis_dir starting at origin.
#define DrawDebugMeterStick(driver, origin, axis_dir, color, depth_test) \
    DebugDrawer::DrawMeterStick3D(driver, origin, axis_dir, color, depth_test)
#define DrawDebugYardStick(driver, origin, axis_dir, color, depth_test) \
    DebugDrawer::DrawYardStick3D(driver, origin, axis_dir, color, depth_test)

class DebugDrawer
{
public:
    
    /**
     * Produces a XY visible grid of evenly sized cells for debuh
     * @param driver The video driver we are using to draw the lines
     * @param z_plane_height The Z level at which the grid should be drawn
     * @param half_extent_m How many meters is half the grid
     * @param minor_step_m Smallest possible cell step size
     * @param major_every How often does a major line occur
     * @param minor_color Color of the minor step lines
     * @param major_color Color of the major step lines
     * @param x_axis_color Color of the X axis at the origin
     * @param y_axis_color Color of the Y axis at the origin
     */
    static void DrawGridXY(irr::video::IVideoDriver* driver,
                           float z_plane_height,
                           float half_extent_m,
                           float minor_step_m,
                           int major_every,
                           const irr::video::SColor& minor_color,
                           const irr::video::SColor& major_color,
                           const irr::video::SColor& y_axis_color,
                           const irr::video::SColor& x_axis_color);

    /**
     * Create a 3d line between a vector of Vector3s
     * @param driver The video driver we are using to draw the lines
     * @param pts The list of points we are using within the line
     * @param color The color we are drawing hte line as 
     * @param depth_test Should this line be occluded by objects or always visible
     */
    static void DrawPolyline3D(
        irr::video::IVideoDriver* driver,
        const std::vector<chrono::ChVector3d>& pts,
        const irr::video::SColor& color,
        bool depth_test = true
    );

    /**
     * Add labels along the grid a specific intervals that denote the distance
     * @param device 
     * @param smgr 
     * @param z_plane 
     * @param half_extent_m 
     * @param step_m 
     * @param scale 
     * @param color 
     * @param label_x_axis 
     * @param label_y_axis 
     */
    static void DrawMeterLabelsXY(irr::IrrlichtDevice* device,
                                 irr::scene::ISceneManager* smgr,
                                 float z_plane,
                                 float half_extent_m,
                                 float step_m,
                                 float scale,
                                 const irr::video::SColor& color,
                                 bool label_x_axis = true,
                                 bool label_y_axis = true);

    /**
     * Draws the components of a vector as a series of arrows: X (red), Y (green), Z (blue).
     * @param driver The video driver we are using to draw the lines
     * @param vec The vector whose components we are visualizing
     * @param origin The world-space origin for the first arrow
     * @param head_length_m Length of the arrow head in meters
     * @param head_width_m Width of the arrow head in meters
     */
    static void DrawVector3dAxisArrows(irr::video::IVideoDriver* driver,
                                       const chrono::ChVector3d& vec,
                                       const irr::video::SColor& color,
                                       const chrono::ChVector3d& origin = chrono::ChVector3d(0, 0, 0),
                                       float head_length_m = 0.15f,
                                       float head_width_m = 0.06f,
                                       bool draw_components = true
                                       );
    
    /**
     * Draws a 1 meter stick with tick marks aligned to axis_dir.
     */
    static void DrawMeterStick3D(irr::video::IVideoDriver* driver,
                                 const chrono::ChVector3d& origin,
                                 const chrono::ChVector3d& axis_dir,
                                 const irr::video::SColor& color,
                                 bool depth_test = true);

    /**
     * Draws a 1 yard stick with tick marks aligned to axis_dir.
     */
    static void DrawYardStick3D(irr::video::IVideoDriver* driver,
                                const chrono::ChVector3d& origin,
                                const chrono::ChVector3d& axis_dir,
                                const irr::video::SColor& color,
                                bool depth_test = true);

    /**
     * Configure the video driver's material to be the debug material so whatever we draw next will be consistnet in material
     * @param driver
     */
    static void SetDebugDriverMaterial(irr::video::IVideoDriver* driver);

    /**
     * Draws a horizontal square/plane at a given Z height centered at (x, y).
     * Useful for visualizing Z thresholds like funnel_halfway_z.
     */
    static void DrawHorizontalPlane(irr::video::IVideoDriver* driver,
                                    const chrono::ChVector3d& center,
                                    double half_size,
                                    const irr::video::SColor& color,
                                    bool depth_test = true);
};

#define DrawDebugHorizontalPlane(driver, center, half_size, color, depth_test) \
    do { \
        DebugDrawer::SetDebugDriverMaterial(driver); \
        DebugDrawer::DrawHorizontalPlane(driver, center, half_size, color, depth_test); \
    } while (0)



