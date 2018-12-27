//
// Created by Ulrich Eck on 18.03.18.
//

#include "artekmed/UbitrackImageShader.h"

// is generated automatically during build
#include "UbitrackShader.h"

#include <algorithm>

#include <Visualization/Utility/ColorMap.h>

#include "artekmed/UbitrackImage.h"

#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("ArtekmedP1.UbitrackImageShader"));

namespace open3d{

namespace glsl {

bool UbitrackImageShader::Compile()
{
    if (CompileShaders(UbitrackImageVertexShader, NULL, UbitrackImageFragmentShader) == false) {
        LOG4CPP_ERROR(logger, "Compiling shaders failed.");
        return false;
    }
    vertex_position_ = glGetAttribLocation(program_, "vertex_position");
    vertex_UV_ = glGetAttribLocation(program_, "vertex_UV");
    image_texture_ = glGetUniformLocation(program_, "image_texture");
    vertex_scale_ = glGetUniformLocation(program_, "vertex_scale");
    return true;
}

void UbitrackImageShader::Release()
{
    UnbindGeometry();
    if (bound_once_) {
        UnbindGeometryOnce();
    }
    ReleaseProgram();
}


bool UbitrackImageShader::BindGeometry(const Geometry &geometry,
        const RenderOption &option, const ViewControl &view)
{
    // this method avoids buffer reallocation for each frame - it's only done once.
    bool result = true;
    if (!bound_once_) {
        result = BindGeometryOnce(geometry, option, view);
    }

    if ((!ubitrack_textureupdate_ptr) || (!ubitrack_textureupdate_ptr->isInitialized())) {
        LOG4CPP_ERROR(logger, "TextureUpdate instance is not initialized.");
        return false;
    }

    auto ubitrackimage = dynamic_cast<const UbitrackImage&>(geometry);
    ubitrack_textureupdate_ptr->updateTexture(ubitrackimage.ubitrack_image_ptr);

    if (option.interpolation_option_ ==
            RenderOption::TextureInterpolationOption::Nearest) {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    } else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                GL_LINEAR_MIPMAP_LINEAR);
//        glGenerateMipmap(GL_TEXTURE_2D);
    }

    bound_ = true;
    return result;
}

bool UbitrackImageShader::BindGeometryOnce(const Geometry &geometry,
        const RenderOption &option, const ViewControl &view)
{
    // If there is already geometry, we first unbind it.
    // We use GL_STATIC_DRAW. When geometry changes, we clear buffers and
    // rebind the geometry. Note that this approach is slow. If the geometry is
    // changing per frame, consider implementing a new ShaderWrapper using
    // GL_STREAM_DRAW, and replace UnbindGeometry() with Buffer Object
    // Streaming mechanisms.
    UnbindGeometryOnce();

    // Prepare data to be passed to GPU
    if (PrepareBinding(geometry, option, view) == false) {
        LOG4CPP_ERROR(logger, "Binding failed when preparing data.");
        return false;
    }

    // Create buffers and bind the geometry
    const GLfloat vertex_position_buffer_data[18] = {
            -1.0f, -1.0f, 0.0f, // lb
            1.0f, -1.0f, 0.0f, // rb
            1.0f, 1.0f, 0.0f, // rt
            -1.0f, -1.0f, 0.0f, // lb
            1.0f, 1.0f, 0.0f, // rt
            -1.0f, 1.0f, 0.0f, // lt
    };

    auto ubitrackimage = dynamic_cast<const UbitrackImage&>(geometry);
    int width = ubitrackimage.ubitrack_image_ptr->width();
    int height = ubitrackimage.ubitrack_image_ptr->height();
    unsigned int origin = ubitrackimage.ubitrack_image_ptr->origin();
    float tx = float( width ) / ubitrack_textureupdate_ptr->pow2width();
    float ty0 = origin ? float( height ) / ubitrack_textureupdate_ptr->pow2height() : 0;
    float ty1 = origin ? 0 : float( height ) / ubitrack_textureupdate_ptr->pow2height();

    const GLfloat vertex_UV_buffer_data[12] = {
            0.0f, ty1, // lb
            tx, ty1, // rb
            tx, ty0, // rt
            0.0f, ty1, // lb
            tx, ty0, // rt
            0.0f, ty0, // lt
    };
    glGenBuffers(1, &vertex_position_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_position_buffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertex_position_buffer_data),
            vertex_position_buffer_data, GL_STATIC_DRAW);
    glGenBuffers(1, &vertex_UV_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_UV_buffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertex_UV_buffer_data),
            vertex_UV_buffer_data, GL_STATIC_DRAW);


    bound_once_ = true;
    return true;
}

bool UbitrackImageShader::RenderGeometry(const Geometry &geometry,
        const RenderOption &option, const ViewControl &view)
{
    if (PrepareRendering(geometry, option, view) == false) {
        LOG4CPP_ERROR(logger, "Rendering failed during preparation.");
        return false;
    }

    glUseProgram(program_);
    glUniform3fv(vertex_scale_, 1, vertex_scale_data_.data());

    // texture update
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, ubitrack_textureupdate_ptr->textureId());
    glUniform1i(image_texture_, 0);

    glEnableVertexAttribArray(vertex_position_);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_position_buffer_);
    glVertexAttribPointer(vertex_position_, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(vertex_UV_);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_UV_buffer_);
    glVertexAttribPointer(vertex_UV_, 2, GL_FLOAT, GL_FALSE, 0, NULL);
    glDrawArrays(draw_arrays_mode_, 0, draw_arrays_size_);
    glDisableVertexAttribArray(vertex_position_);
    glDisableVertexAttribArray(vertex_UV_);

    return true;
}

void UbitrackImageShader::UnbindGeometry()
{
    if (bound_) {

        // noop

        bound_ = false;
    }

}

void UbitrackImageShader::UnbindGeometryOnce()
{
    if (bound_once_) {
        glDeleteBuffers(1, &vertex_position_buffer_);
        glDeleteBuffers(1, &vertex_UV_buffer_);

        ubitrack_textureupdate_ptr->cleanupTexture();

        bound_once_ = false;
    }
}

bool ImageShaderForUbitrackImage::PrepareRendering(const Geometry &geometry,
        const RenderOption &option,const ViewControl &view)
{
    if (geometry.GetGeometryType() != Geometry::GeometryType::Image) {
        LOG4CPP_ERROR(logger, "Rendering type is not Image.");
        return false;
    }

    auto ubitrackimage = dynamic_cast<const UbitrackImage&>(geometry);
    if ((!ubitrack_textureupdate_ptr) || (!ubitrack_textureupdate_ptr->isInitialized())) {
        LOG4CPP_ERROR(logger, "TextureUpdate is not initializesd.");
        return false;
    }

    int width = ubitrackimage.ubitrack_image_ptr->width();
    int height = ubitrackimage.ubitrack_image_ptr->height();

    GLfloat ratio_x, ratio_y;
    switch (option.image_stretch_option_) {
    case RenderOption::ImageStretchOption::StretchKeepRatio:
        ratio_x = GLfloat(width) / GLfloat(view.GetWindowWidth());
        ratio_y = GLfloat(height) / GLfloat(view.GetWindowHeight());
        if (ratio_x < ratio_y) {
            ratio_x /= ratio_y;
            ratio_y = 1.0f;
        } else {
            ratio_y /= ratio_x;
            ratio_x = 1.0f;
        }
        break;
    case RenderOption::ImageStretchOption::StretchWithWindow:
        ratio_x = 1.0f;
        ratio_y = 1.0f;
        break;
    case RenderOption::ImageStretchOption::OriginalSize:
    default:
        ratio_x = GLfloat(width) / GLfloat(view.GetWindowWidth());
        ratio_y = GLfloat(height) / GLfloat(view.GetWindowHeight());
        break;
    }


    vertex_scale_data_(0) = ratio_x;
    vertex_scale_data_(1) = ratio_y;
    vertex_scale_data_(2) = 1.0f;

    glDisable(GL_DEPTH_TEST);
    return true;
}

bool ImageShaderForUbitrackImage::PrepareBinding(const Geometry &geometry,
        const RenderOption &option, const ViewControl &view)
{
    if (geometry.GetGeometryType() != Geometry::GeometryType::Image) {
        LOG4CPP_ERROR(logger, "Rendering type is not Image.");
        return false;
    }

    ubitrack_textureupdate_ptr = std::unique_ptr<Ubitrack::Vision::TextureUpdate>(new Ubitrack::Vision::TextureUpdate());

    auto ubitrackimage = dynamic_cast<const UbitrackImage&>(geometry);
    if (!ubitrackimage.HasData()) {
        LOG4CPP_ERROR(logger, "Binding failed with empty image.");
        return false;
    }

    if (!ubitrack_textureupdate_ptr->isInitialized())
        ubitrack_textureupdate_ptr->initializeTexture(ubitrackimage.ubitrack_image_ptr);

    draw_arrays_mode_ = GL_TRIANGLES;
    draw_arrays_size_ = 6;
    return true;
}

}	// namespace glsl

}	// namespace open3d