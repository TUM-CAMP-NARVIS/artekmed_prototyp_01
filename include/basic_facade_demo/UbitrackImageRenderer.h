//
// Created by Ulrich Eck on 18.03.18.
//

#ifndef BASIC_FACADE_DEMO_UBITRACKIMAGERENDERER_H
#define BASIC_FACADE_DEMO_UBITRACKIMAGERENDERER_H

#include <Visualization/Shader/GeometryRenderer.h>

#include "basic_facade_demo/UbitrackImageShader.h"

namespace three {
namespace glsl {

class UbitrackImageRenderer: public GeometryRenderer {
public:
    ~UbitrackImageRenderer() override {}

public:
    bool Render(const RenderOption& option, const ViewControl& view) override;
    bool AddGeometry(std::shared_ptr<const Geometry> geometry_ptr) override;
    bool UpdateGeometry() override;

protected:
    ImageShaderForUbitrackImage image_shader_;
};

}} // three::glsl
#endif //BASIC_FACADE_DEMO_UBITRACKIMAGERENDERER_H
