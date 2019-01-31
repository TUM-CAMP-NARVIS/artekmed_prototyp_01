//
// Created by Ulrich Eck on 15.03.18.
//

// implementation of UbitrackPointCloudConnector

#include <functional>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include "artekmed/UbitrackPointCloudConnector.h"
#include "artekmed/EigenWrapper.h"
#include "artekmed/PointCloudProcessing.h"

#include <boost/bind.hpp>

#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("ArtekmedP1.UbitrackPointCloudConnector"));


UbitrackPointCloudConnector::UbitrackPointCloudConnector(const std::string& _components_path)
        : UbitrackBaseConnector(_components_path)
{

}

void UbitrackPointCloudConnector::add_cameras() {

    m_cameras.emplace_back(std::make_shared<artekmed::RGBDCameraConnector>("camera01"));
    m_cameras.emplace_back(std::make_shared<artekmed::RGBDCameraConnector>("camera02"));
//    m_cameras.emplace_back(std::make_shared<artekmed::RGBDCameraConnector>("camera03"));
}


