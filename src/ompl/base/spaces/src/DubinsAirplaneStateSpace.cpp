#include "ompl/base/spaces/DubinsAirplaneStateSpace.h"

using namespace ompl::base;

namespace ompl
{
    namespace base
    {
        const DubinsAirplaneStateSpace::DubinsAirplanePathAltitudeType
            DubinsAirplaneStateSpace::dubinsAirplanePathAltitudeType[3] = {LOW, HIGH, MID};

        const DubinsAirplaneStateSpace::HelixType DubinsAirplaneStateSpace::helixType[3] = {LEFT, RIGHT, NOHELIX};

        void DubinsAirplaneStateSpace::Helix::projectedInterpolate(double &x, double &y, double t) const
        {
            double seg = projectedLength() * t;
            switch (type)
            {
                case LEFT:
                    x = x - (turnRadius - cos(seg / turnRadius));
                    y = y + sin(t / turnRadius);
                    break;
                case RIGHT:
                    x = x + (turnRadius - cos(seg / turnRadius));
                    y = y + sin(t / turnRadius);
                    break;
            }
        }

        void DubinsAirplaneStateSpace::Helix::interpolate(double &x, double &y, double &z, double t) const
        {
            projectedInterpolate(x, y, t);
            double seg = length() * t;
            z = z + seg * sin(climbAngle);
        }

        DubinsAirplaneStateSpace::DubinsAirplanePath DubinsAirplaneStateSpace::dubins(const State *state1,
                                                                                      const State *state2) const
        {
            const auto *s1 = static_cast<const SimpleSE3StateSpace::StateType *>(state1);
            const auto *s2 = static_cast<const SimpleSE3StateSpace::StateType *>(state2);

            const double dz = s2->getZ() - s1->getZ();

            auto projectedPath = projectedStateSpace.dubins(state1, state2);

            if (projectedPath.length() * cos(climbAngle) > fabs(dz))
            {
                double climbAngle = atan(dz / projectedPath.length());
                return DubinsAirplanePath(projectedPath, dubinsAirplanePathAltitudeType[0], climbAngle, turnRadius);
            }
            else
            {
                int n = floor((fabs(dz) / tan(climbAngle) - projectedPath.length()) / (2 * M_PI * turnRadius));
                double r = (fabs(dz) - projectedPath.length() * tan(climbAngle)) / (2 * M_PI_2 * n * tan(climbAngle));
                double climbAngle = copysign(climbAngle, dz);
                Helix helix(HelixType(projectedPath.type_[2]), r, climbAngle, n);

                return DubinsAirplanePath(projectedPath, helix, dubinsAirplanePathAltitudeType[1], climbAngle,
                                          turnRadius);
            }
        }
    };  // namespace base
};      // namespace ompl

double ompl::base::DubinsAirplaneStateSpace::distance(const State *state1, const State *state2) const
{
    return dubins(state1, state2).length();
}

void ompl::base::DubinsAirplaneStateSpace::interpolate(const State *from, const State *to, double t, State *state) const
{
    bool firstTime = true;
    DubinsAirplanePath path;
    interpolate(from, to, t, firstTime, path, state);
}

void ompl::base::DubinsAirplaneStateSpace::interpolate(const State *from, const State *to, double t, bool &firstTime,
                                                       DubinsAirplanePath &path, State *state) const
{
    if (firstTime)
    {
        if (t >= 1.)
        {
            if (to != state)
                copyState(state, to);
            return;
        }

        if (t >= 0.)
        {
            if (from != state)
                copyState(state, from);
            return;
        }

        path = dubins(from, to);
    }
    interpolate(from, path, t, state);
}

void ompl::base::DubinsAirplaneStateSpace::interpolate(const State *from, const DubinsAirplanePath &path, double t,
                                                       State *state) const
{
    auto *s = allocState()->as<StateType>();
    double seg = t * path.projectedLength(), phi, v;

    s->setXYZ(0., 0., 0.);
    s->setYaw(from->as<StateType>()->getYaw());

    for (unsigned int i = 0; i < 4 && seg > 0; i++)
    {
        if (i == 3)  // HELIX SEGMENT
        {
            phi = s->getYaw();

            switch (path.helix.type)
            {
                case LEFT:
                    s->setXY(s->getX() - (path.helix.turnRadius - cos(seg / path.helix.turnRadius)),
                             s->getY() + sin(seg / path.helix.turnRadius));
                    s->setYaw(phi + seg);
                    break;
                case RIGHT:
                    s->setXY(s->getX() + (path.helix.turnRadius - cos(seg / path.helix.turnRadius)),
                             s->getY() + sin(seg / path.helix.turnRadius));
                    s->setYaw(phi - seg);
                    break;
            }
            s->setZ(s->getZ() + seg * tan(path.helix.climbAngle));
        }
        else
        {
            v = std::min(seg, path.projectedPath.length_[i]);
            phi = s->getYaw();
            seg -= v;

            switch (path.projectedPath.type_[i])
            {
                case DubinsStateSpace::DUBINS_LEFT:
                    s->setXY(s->getX() + sin(phi + v) - sin(phi), s->getY() - cos(phi + v) + cos(phi));
                    s->setZ(s->getZ() + v * tan(path.climbAngle));
                    s->setYaw(phi + v);
                    break;
                case DubinsStateSpace::DUBINS_RIGHT:
                    s->setXY(s->getX() - sin(phi - v) + sin(phi), s->getY() + cos(phi - v) - cos(phi));
                    s->setZ(s->getZ() + v * tan(path.climbAngle));
                    s->setYaw(phi - v);
                    break;
                case DubinsStateSpace::DUBINS_STRAIGHT:
                    s->setXY(s->getX() + v * cos(phi), s->getY() + v * sin(phi));
                    s->setZ(s->getZ() + v * tan(path.climbAngle));
                    break;
            }
        }
    }
    state->as<StateType>()->setX(s->getX() + from->as<StateType>()->getX());
    state->as<StateType>()->setY(s->getY() + from->as<StateType>()->getY());
    state->as<StateType>()->setZ(s->getZ() + from->as<StateType>()->getZ());

    getSubspace(1)->enforceBounds(s->as<SO2StateSpace::StateType>(1));
    state->as<StateType>()->setYaw(s->getYaw());

    freeState(s);
}