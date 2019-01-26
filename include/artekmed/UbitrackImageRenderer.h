//
// Created by Ulrich Eck on 18.03.18.
//

#ifndef ARTEKMED_UBITRACKIMAGERENDERER_H
#define ARTEKMED_UBITRACKIMAGERENDERER_H

#include <artekmed/Visualization/Shader/GeometryRenderer.h>

#include "artekmed/Visualization/Shader/UbitrackImageShader.h"

namespace artekmed {

    using namespace open3d;

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

}} // open3d::glsl
#endif //ARTEKMED_UBITRACKIMAGERENDERER_H
