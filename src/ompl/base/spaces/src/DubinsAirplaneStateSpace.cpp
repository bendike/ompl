#include "ompl/base/spaces/DubinsAirplaneStateSpace.h"
#include <boost/math/constants/constants.hpp>
#include <queue>
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"

using namespace ompl::base;

namespace ompl
{
    namespace base
    {
        const double twopi = 2. * boost::math::constants::pi<double>();
        const double DUBINS_EPS = 1e-6;
        const double DUBINS_ZERO = -1e-7;

        inline double mod2pi(double x)
        {
            if (x < 0 && x > DUBINS_ZERO)
                return 0;
            double xm = x - twopi * floor(x / twopi);
            if (twopi - xm < .5 * DUBINS_EPS)
                xm = 0.;
            return xm;
        }

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
                case NOHELIX:
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

            auto projectedPath = projectedStateSpace->as<DubinsStateSpace>()->dubins(state1, state2);
            double climbAngle = 0;
            return DubinsAirplanePath(projectedPath, dubinsAirplanePathAltitudeType[0], climbAngle, turnRadius);

            if (projectedPath.length() * cos(climbAngle) > fabs(dz))
            {
                double climbAngle = atan(dz / projectedPath.length());
                return DubinsAirplanePath(projectedPath, dubinsAirplanePathAltitudeType[0], climbAngle, turnRadius);
            }
            else
            {
                int n = floor((fabs(dz) / tan(climbAngle) - projectedPath.length()) / (2 * M_PI * turnRadius));
                double r = (fabs(dz) - projectedPath.length() * tan(climbAngle)) / (2 * M_PI_2 * n * tan(climbAngle));
                double climbAngle_ = copysign(climbAngle, dz);
                Helix helix(HelixType(projectedPath.type_[2]), r, climbAngle, n);

                return DubinsAirplanePath(projectedPath, helix, dubinsAirplanePathAltitudeType[1], climbAngle_,
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

        if (t <= 0.)
        {
            if (from != state)
                copyState(state, from);
            return;
        }

        path = dubins(from, to);
        firstTime = false;
    }
    interpolate(from, path, t, state);
}

void ompl::base::DubinsAirplaneStateSpace::interpolate(const State *from, const DubinsAirplanePath &path, double t,
                                                       State *state) const
{
    auto *s = allocState()->as<StateType>();
    double seg = t * path.projectedLength(), phi, v, th;

    s->setXYZ(0., 0., 0.);
    s->setYaw(from->as<StateType>()->getYaw());

    for (unsigned int i = 0; i < 3 && seg > 0; i++)
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
                case NOHELIX:
                    break;
            }
            s->setZ(s->getZ() + seg * tan(path.helix.climbAngle));
        }
        else
        {
            v = std::min(seg, path.projectedPath.length_[i] * turnRadius);

            th = v / turnRadius;
            // phi = s->getYaw() < 0 ? s->getYaw() - 3 * M_PI_2 : s->getYaw() - M_PI_2;
            phi = s->getYaw();
            seg -= v;

            switch (path.projectedPath.type_[i])
            {
                case DubinsStateSpace::DUBINS_LEFT:
                    // s->setX(s->getX() + turnRadius * (cos(phi - th) - cos(phi)));
                    // s->setY(s->getY() + turnRadius * (sin(phi + th) - sin(phi)));
                    s->setXY(s->getX() + (sin(phi + th) - sin(phi)) * turnRadius,
                             s->getY() - (cos(phi + th) - cos(phi)) * turnRadius);
                    s->setZ(s->getZ() + v * tan(path.climbAngle));
                    s->setYaw(s->getYaw() + th);
                    break;
                case DubinsStateSpace::DUBINS_RIGHT:
                    // s->setX(s->getX() + turnRadius * (-cos(phi + th) + cos(phi)));
                    // s->setY(s->getY() + turnRadius * (sin(phi + th) - sin(phi)));
                    s->setXY(s->getX() - (sin(phi - th) - sin(phi)) * turnRadius,
                             s->getY() + (cos(phi - th) - cos(phi)) * turnRadius);
                    s->setZ(s->getZ() + v * tan(path.climbAngle));
                    s->setYaw(s->getYaw() - th);
                    break;
                case DubinsStateSpace::DUBINS_STRAIGHT:
                    s->setXY(s->getX() - (v * cos(phi)), s->getY() - (v * sin(phi)));
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

void ompl::base::DubinsAirplaneMotionValidator::defaultSettings()
{
    stateSpace_ = dynamic_cast<DubinsAirplaneStateSpace *>(si_->getStateSpace().get());
    if (stateSpace_ == nullptr)
        throw Exception("No state space for motion validator");
}

bool ompl::base::DubinsAirplaneMotionValidator::checkMotion(const State *s1, const State *s2,
                                                            std::pair<State *, double> &lastValid) const
{
    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true, firstTime = true;
    DubinsAirplaneStateSpace::DubinsAirplanePath path;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    if (nd > 1)
    {
        /* temporary storage for the checked state */
        State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, firstTime, path, test);
            if (!si_->isValid(test))
            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first != nullptr)
                    stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, path, lastValid.first);
                result = false;
                break;
            }
        }
        si_->freeState(test);
    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, path, lastValid.first);
            result = false;
        }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool ompl::base::DubinsAirplaneMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2))
        return false;

    bool result = true, firstTime = true;
    DubinsAirplaneStateSpace::DubinsAirplanePath path;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    /* initialize the queue of test positions */
    std::queue<std::pair<int, int>> pos;
    if (nd >= 2)
    {
        pos.emplace(1, nd - 1);

        /* temporary storage for the checked state */
        State *test = si_->allocState();

        /* repeatedly subdivide the path segment in the middle (and check the middle) */
        while (!pos.empty())
        {
            std::pair<int, int> x = pos.front();

            int mid = (x.first + x.second) / 2;
            stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, firstTime, path, test);

            if (!si_->isValid(test))
            {
                result = false;
                break;
            }

            pos.pop();

            if (x.first < mid)
                pos.emplace(x.first, mid - 1);
            if (x.second > mid)
                pos.emplace(mid + 1, x.second);
        }

        si_->freeState(test);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}