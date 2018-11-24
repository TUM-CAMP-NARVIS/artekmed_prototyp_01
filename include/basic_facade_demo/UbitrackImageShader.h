//
// Created by Ulrich Eck on 18.03.18.
//

#ifndef BASIC_FACADE_DEMO_UBITRACKVIDEOBACKGROUNDSHADER_H
#define BASIC_FACADE_DEMO_UBITRACKVIDEOBACKGROUNDSHADER_H

#include "basic_facade_demo/UbitrackImage.h"
#include <Visualization/Shader/ShaderWrapper.h>

#include <utFacade/BasicTextureUpdate.h>


namespace open3d {

namespace glsl {

class UbitrackImageShader : public glsl::ShaderWrapper
{
public:
    ~UbitrackImageShader() override { Release(); }

protected:
    UbitrackImageShader(const std::string &name)
    : ShaderWrapper(name)
    , bound_once_(false)
    {
        Compile();
    }

protected:
    bool Compile() final;
    void Release() final;
    bool BindGeometry(const Geometry &geometry, const RenderOption &option,
            const ViewControl &view) final;
    bool RenderGeometry(const Geometry &geometry, const RenderOption &option,
            const ViewControl &view) final;
    void UnbindGeometry() final;

    bool BindGeometryOnce(const Geometry &geometry, const RenderOption &option,
            const ViewControl &view);
    void UnbindGeometryOnce();


protected:
    virtual bool PrepareRendering(const Geometry &geometry,
            const RenderOption &option, const ViewControl &view) = 0;
    virtual bool PrepareBinding(const Geometry &geometry,
            const RenderOption &option, const ViewControl &view) = 0;

protected:
    GLuint vertex_position_;
    GLuint vertex_position_buffer_;
    GLuint vertex_UV_;
    GLuint vertex_UV_buffer_;
    GLuint image_texture_;
    // this is provided by textureupdate
    //GLuint image_texture_buffer_;
    GLuint vertex_scale_;

    GLHelper::GLVector3f vertex_scale_data_;

    bool bound_once_;

    std::unique_ptr<Ubitrack::Facade::BasicTextureUpdate> ubitrack_textureupdate_ptr;

};

class ImageShaderForUbitrackImage : public UbitrackImageShader
{
public:
    ImageShaderForUbitrackImage() : UbitrackImageShader("ImageShaderForUbitrackImage") {}

protected:
    bool PrepareRendering(const Geometry &geometry,
            const RenderOption &option, const ViewControl &view) final;
    bool PrepareBinding(const Geometry &geometry,
            const RenderOption &option, const ViewControl &view) final;
};

}	// namespace glsl

}	// namespace open3d


#endif //BASIC_FACADE_DEMO_UBITRACKVIDEOBACKGROUNDSHADER_H
