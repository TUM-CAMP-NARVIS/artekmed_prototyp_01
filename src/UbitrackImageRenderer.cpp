//
// Created by Ulrich Eck on 18.03.18.
//

#include "basic_facade_demo/UbitrackImageRenderer.h"

namespace three {
namespace glsl {

bool UbitrackImageRenderer::Render(const RenderOption &option, const ViewControl &view)
{
    if (is_visible_ == false || geometry_ptr_->IsEmpty()) return true;
    return image_shader_.Render(*geometry_ptr_, option, view);
}

bool UbitrackImageRenderer::AddGeometry(std::shared_ptr<const Geometry> geometry_ptr)
{
    if (geometry_ptr->GetGeometryType() !=
            Geometry::GeometryType::Image) {
        return false;
    }
    geometry_ptr_ = geometry_ptr;
    return UpdateGeometry();
}

bool UbitrackImageRenderer::UpdateGeometry()
{
    image_shader_.InvalidateGeometry();
    return true;
}

}} // namespace three::glsl
