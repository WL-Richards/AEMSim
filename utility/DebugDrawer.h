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

#define DrawDebugMeterLabelsXY(device, smgr, z_plane, half_extent_m, step_m, text_height_m, color, label_x_axis, label_y_axis) DebugDrawer::DrawMeterLabelsXY(device, smgr, z_plane, half_extent_m, step_m, text_height_m, color, label_x_axis, label_y_axis);       

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
                           const irr::video::SColor& x_axis_color,
                           const irr::video::SColor& y_axis_color);

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
     * @param text_height_m 
     * @param color 
     * @param label_x_axis 
     * @param label_y_axis 
     */
    static void DrawMeterLabelsXY(irr::IrrlichtDevice* device,
                                 irr::scene::ISceneManager* smgr,
                                 float z_plane,
                                 float half_extent_m,
                                 float step_m,
                                 float text_height_m,
                                 const irr::video::SColor& color,
                                 bool label_x_axis = true,
                                 bool label_y_axis = true);
    

    /**
     * Configure the video driver's material to be the debug material so whatever we draw next will be consistnet in material
     * @param driver 
     */
    static void SetDebugDriverMaterial(irr::video::IVideoDriver* driver);
};



