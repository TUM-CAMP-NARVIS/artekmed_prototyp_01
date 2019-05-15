//
// Created by narvis on 15.05.19.
//

#ifndef ARTEKMED_P1_TEMPERATURECOLORS_H
#define ARTEKMED_P1_TEMPERATURECOLORS_H

#include <algorithm>

namespace artekmed
{
	namespace util
	{
		//Converts a [0-1] float range into a BGR color 0=bule, 1=red
		Eigen::Vector3d colorTemperature(float temp)
		{
			temp = std::min(1.f,std::max(0.f,temp));
			const float blue = std::max((temp)*-0.5f+1.f,0.f);
			const float green = std::max(std::abs(temp-0.5f)*-0.5f+1.0f,0.f);
			const float red = std::max((temp-0.5f)*0.5f,0.f);
			return {blue,green,red};
		}
	}
}

#endif //ARTEKMED_P1_TEMPERATURECOLORS_H
