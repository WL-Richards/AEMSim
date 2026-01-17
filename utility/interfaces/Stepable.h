#pragma once

namespace irr::video
{
    class IVideoDriver;
}

/* Interface to describe how provide a basis for calling tick functions at specific points in the loop*/
class Stepable
{
public:
    
    /**
     * Called during the physics step phase of the simulation
     */
    virtual void DoPhysicsStep() = 0;

    /*
    * Called between vis.Render() and vis.EndScene allows for updating render elements
    */
    virtual void DoRenderStep(irr::video::IVideoDriver* drv) = 0;
    
protected:
    ~Stepable() = default;
};
