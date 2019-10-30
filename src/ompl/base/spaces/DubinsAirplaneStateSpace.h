#ifndef OMPL_BASE_SPACES_DUBINS_AIRPLANE_STATE_SPACE
#define OMPL_BASE_SPACES_DUBINS_AIRPLANE_STATE_SPACE

#include <math.h>
#include "ompl/base/spaces/SimpleSE3StateSpace.h"
#include "ompl/base/spaces/DubinsStateSpace.h"

namespace ompl
{
    namespace base
    {
        class DubinsAirplaneStateSpace : public SimpleSE3StateSpace
        {
            enum DubinsAirplanePathAltitudeType
            {
                LOW = 0,
                MID = 1,
                HIGH = 2
            };

            static const DubinsAirplanePathAltitudeType dubinsAirplanePathAltitudeType[3];

            class Helix
            {
            public:
                int n = 0;
                double turnRadius = 0.0;
                double climbAngle = 0.0;
            };

            class DubinsAirplanePath
            {
            public:
                DubinsAirplanePath();
            };
        };
    }  // namespace base
}  // namespace ompl

#endif