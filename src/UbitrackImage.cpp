//
// Created by Ulrich Eck on 18.03.18.
//

#include "artekmed/UbitrackImage.h"

namespace open3d {

void UbitrackImage::Clear()
{
    ubitrack_image_ptr.reset();
}

bool UbitrackImage::IsEmpty() const
{
    return !HasData();
}

Eigen::Vector2d UbitrackImage::GetMinBound() const
{
    return Eigen::Vector2d(0.0, 0.0);
}

Eigen::Vector2d UbitrackImage::GetMaxBound() const
{
    if (ubitrack_image_ptr)
        return Eigen::Vector2d(ubitrack_image_ptr->width(), ubitrack_image_ptr->height());
    return Eigen::Vector2d(0., 0.);
}

std::shared_ptr<UbitrackImage> CreateEmptyUbitrackImage()
{
    auto image = std::make_shared<UbitrackImage>();
    return image;
}

}	// namespace open3d