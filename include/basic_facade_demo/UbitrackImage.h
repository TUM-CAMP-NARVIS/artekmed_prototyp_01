//
// Created by Ulrich Eck on 18.03.18.
//

#ifndef BASIC_FACADE_DEMO_UBITRACKVIDEOBACKGROUND_H
#define BASIC_FACADE_DEMO_UBITRACKVIDEOBACKGROUND_H

#include <vector>
#include <memory>
#include <mutex>
#include <Eigen/Core>

#include <Core/Geometry/Geometry2D.h>
#include <Core/Utility/Console.h>

#include <utFacade/BasicFacadeTypes.h>


namespace three {

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
            return ubitrack_image_ptr->isValid() && ubitrack_image_ptr->getByteCount() > 0;
        return false;
    }

public:
    std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement> ubitrack_image_ptr;
};

std::shared_ptr<UbitrackImage> CreateEmptyUbitrackImage();


} // three

#endif //BASIC_FACADE_DEMO_UBITRACKVIDEOBACKGROUND_H
