//
// Created by Ulrich Eck on 18.03.18.
//

#ifndef ARTEKMED_UBITRACKVIDEOBACKGROUND_H
#define ARTEKMED_UBITRACKVIDEOBACKGROUND_H

#include <vector>
#include <memory>
#include <mutex>
#include <Eigen/Core>

#include <Core/Geometry/Geometry2D.h>
#include <Core/Utility/Console.h>

#include <boost/shared_ptr.hpp>
#include <utVision/Image.h>


namespace open3d {

class UbitrackImage: public Geometry2D {
public:

public:
    UbitrackImage()
            :Geometry2D(Geometry::GeometryType::Image) {};
    ~UbitrackImage() override {};

public:
    void Clear() override;
    bool IsEmpty() const override;
    Eigen::Vector2d GetMinBound() const override;
    Eigen::Vector2d GetMaxBound() const override;

public:
    virtual bool HasData() const
    {
        if (ubitrack_image_ptr)
            return true;
        return false;
    }

public:
    Ubitrack::Measurement::ImageMeasurement ubitrack_image_ptr;
};

std::shared_ptr<UbitrackImage> CreateEmptyUbitrackImage();


} // open3d

#endif //ARTEKMED_UBITRACKVIDEOBACKGROUND_H
