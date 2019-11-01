#ifndef OMPL_BASE_SPACES_DUBINS_AIRPLANE_STATE_SPACE
#define OMPL_BASE_SPACES_DUBINS_AIRPLANE_STATE_SPACE

#include <stdio.h>
#include <math.h>
#include "ompl/base/spaces/SimpleSE3StateSpace.h"
#include "ompl/base/spaces/DubinsStateSpace.h"

namespace ompl
{
    namespace base
    {
        class DubinsAirplaneStateSpace : public SimpleSE3StateSpace
        {
        public:
            enum DubinsAirplanePathAltitudeType
            {
                LOW = 0,
                MID = 1,
                HIGH = 2
            };

            static const DubinsAirplanePathAltitudeType dubinsAirplanePathAltitudeType[3];

            enum HelixType
            {
                LEFT = 0,
                NOHELIX = 1,
                RIGHT = 2
            };

            static const HelixType helixType[3];

            class Helix
            {
            public:
                int n = 0;
                double turnRadius = 0.0;
                double climbAngle = 0.0;
                HelixType type;

                Helix(HelixType type, double r, double a, int n) : n(n), turnRadius(r), climbAngle(a), type(type)
                {
                }

                double projectedLength() const
                {
                    return n * 2 * M_PI * turnRadius;
                }

                double length() const
                {
                    return n * 2 * M_PI * turnRadius * climbAngle;
                }

                void projectedInterpolate(double &x, double &y, double t) const;

                void interpolate(double &x, double &y, double &z, double t) const;
            };

            class DubinsAirplanePath
            {
            public:
                DubinsStateSpace::DubinsPath projectedPath;
                Helix helix;
                DubinsAirplanePathAltitudeType type;
                double climbAngle;
                double turnRadius;

                DubinsAirplanePath() : helix(NOHELIX, 0, 0, 0)
                {
                }

                DubinsAirplanePath(DubinsStateSpace::DubinsPath &projectedPath, Helix helix,
                                   DubinsAirplanePathAltitudeType type, double climbAngle, double turnRadius)
                  : projectedPath(projectedPath)
                  , helix(helix)
                  , type(type)
                  , climbAngle(climbAngle)
                  , turnRadius(turnRadius)
                {
                }

                DubinsAirplanePath(DubinsStateSpace::DubinsPath &projectedPath, DubinsAirplanePathAltitudeType type,
                                   double climbAngle, double turnRadius)
                  : projectedPath(projectedPath)
                  , helix(NOHELIX, 0, 0, 0)
                  , type(type)
                  , climbAngle(climbAngle)
                  , turnRadius(turnRadius)
                {
                }

                double length() const
                {
                    return (projectedPath.length() * turnRadius / cos(climbAngle)) + helix.length();
                }

                double projectedLength() const
                {
                    return projectedPath.length() * turnRadius + helix.projectedLength();
                }
            };

            StateSpacePtr projectedStateSpace;
            double climbAngle;
            double turnRadius;

            DubinsAirplaneStateSpace(double turnRadius, double climbAngle)
              : climbAngle(climbAngle), turnRadius(turnRadius)
            {
                projectedStateSpace = std::make_shared<DubinsStateSpace>(turnRadius,false);
            }

            // OVERRIDES

            bool isMetricSpace() const override
            {
                return false;
            }

            bool hasSymmetricDistance() const override
            {
                return false;
            }

            bool hasSymmetricInterpolate() const override
            {
                return false;
            }

            void sanityChecks() const override
            {
                double zero = std::numeric_limits<double>::epsilon();
                double eps = std::numeric_limits<float>::epsilon();
                int flags = ~(StateSpace::STATESPACE_INTERPOLATION | StateSpace::STATESPACE_TRIANGLE_INEQUALITY |
                              StateSpace::STATESPACE_DISTANCE_BOUND);
                flags &= ~StateSpace::STATESPACE_DISTANCE_SYMMETRIC;
                StateSpace::sanityChecks(zero, eps, flags);
            }

            double distance(const State *state1, const State *state2) const override;

            void interpolate(const State *from, const State *to, double t, State *state) const override;

            // MEMBERS
            virtual void interpolate(const State *from, const State *to, double t, bool &firstTime,
                                     DubinsAirplanePath &path, State *state) const;

            DubinsAirplanePath dubins(const State *state1, const State *state2) const;

            void interpolate(const State *from, const DubinsAirplanePath &path, double t, State *state) const;
        };

        class DubinsAirplaneMotionValidator : public MotionValidator
        {
        public:
            DubinsAirplaneMotionValidator(SpaceInformation *si) : MotionValidator(si)
            {
                defaultSettings();
            }
            DubinsAirplaneMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
            {
                defaultSettings();
            }
            ~DubinsAirplaneMotionValidator() override = default;
            bool checkMotion(const State *s1, const State *s2) const override;
            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override;

        private:
            DubinsAirplaneStateSpace *stateSpace_;
            void defaultSettings();
        };
    }  // namespace base
}  // namespace ompl

#endif