// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#pragma once

#include <vector>
#include <Eigen/Core>
#include <artekmed/Visualization/Shader/ShaderWrapper.h>

namespace artekmed {

    using namespace open3d;

namespace glsl {
    
class SimpleBlackShader : public ShaderWrapper
{
public:
    ~SimpleBlackShader() override { Release(); }

protected:
    SimpleBlackShader(const std::string &name) : ShaderWrapper(name) { Compile(); }

protected:
    bool Compile() final;
    void Release() final;
    bool BindGeometry(const Geometry &geometry, const RenderOption &option,
            const ViewControl &view) final;
    bool RenderGeometry(const Geometry &geometry, const RenderOption &option,
            const ViewControl &view) final;
    void UnbindGeometry() final;

protected:
    virtual bool PrepareRendering(const Geometry &geometry,
            const RenderOption &option, const ViewControl &view) = 0;
    virtual bool PrepareBinding(const Geometry &geometry,
            const RenderOption &option, const ViewControl &view,
            std::vector<Eigen::Vector3f> &points) = 0;

protected:
    GLuint vertex_position_;
    GLuint vertex_position_buffer_;
    GLuint MVP_;
};
    
class SimpleBlackShaderForPointCloudNormal : public SimpleBlackShader
{
public:
    SimpleBlackShaderForPointCloudNormal() :
            SimpleBlackShader("SimpleBlackShaderForPointCloudNormal") {}

protected:
    bool PrepareRendering(const Geometry &geometry,
            const RenderOption &option, const ViewControl &view) final;
    bool PrepareBinding(const Geometry &geometry,
            const RenderOption &option, const ViewControl &view,
            std::vector<Eigen::Vector3f> &points) final;
};

class SimpleBlackShaderForTriangleMeshWireFrame : public SimpleBlackShader
{
public:
    SimpleBlackShaderForTriangleMeshWireFrame() :
            SimpleBlackShader("SimpleBlackShaderForTriangleMeshWireFrame") {}

protected:
    bool PrepareRendering(const Geometry &geometry,
            const RenderOption &option, const ViewControl &view) final;
    bool PrepareBinding(const Geometry &geometry,
            const RenderOption &option, const ViewControl &view,
            std::vector<Eigen::Vector3f> &points) final;
};

}    // namespace open3d::glsl

}    // namespace open3d
