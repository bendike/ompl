#include <ompl/base/ScopedState.h>
#include <ompl/base/Planner.h>
#include <ompl/base/spaces/DubinsAirplaneStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <boost/program_options.hpp>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

double interpolation = 100;
double solverRunTime = 10;

bool isStateValid(const ob::SpaceInformation *si, const ob::State *state)
{
    return si->satisfiesBounds(state);
}

void plan()
{
    ob::StateSpacePtr space = std::make_shared<ob::DubinsAirplaneStateSpace>(0.5, 0.25);

    ob::RealVectorBounds bounds(3);
    bounds.setLow(-3);
    bounds.setHigh(3);

    space->as<ob::SimpleSE3StateSpace>()->setBounds(bounds);

    og::SimpleSetup ss(space);

    ob::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker([si](const ob::State *state) { return isStateValid(si, state); });

    si->setMotionValidator(std::make_shared<ob::DubinsAirplaneMotionValidator>(si));

    ob::ScopedState<ob::SimpleSE3StateSpace> start(space);
    start->setXYZ(0.5, 0.0, -1.0);
    start->setYaw(0.0);

    ob::ScopedState<ob::SimpleSE3StateSpace> goal(space);
    goal->setXYZ(-0.5, 0.0, 1.0);
    goal->setYaw(0.0);
    ss.setStartAndGoalStates(start, goal);

    auto planner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    ss.setup();

    ob::PlannerStatus solved = ss.solve(solverRunTime);

    if (solved)
    {
        ss.simplifySolution();
        std::cout << "Found solution:" << std::endl;
        og::PathGeometric path = ss.getSolutionPath();
        path.interpolate(interpolation);
        path.printAsMatrix(std::cout);

        std::ofstream pathfile;
        pathfile.open("/home/bendike/ompl_ws/data/paths/path.txt");
        path.printAsMatrix(pathfile);
        pathfile.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

bool legalNumber(char *str)
{
    while (*str != 0)
    {
        if (!isdigit(*str++))
        {
            return false;
        }
    }
    return true;
}

void parseArgs(int argc, char *argv[])
{
    for (int i = 1; i < argc; i++)
    {
        char *arg = argv[i];
        std::cout << arg << " ";

        if (strcmp(arg, "--time") == 0 || strcmp(arg, "-t") == 0)
        {
            if (legalNumber(argv[i + 1]))
            {
                solverRunTime = atol(argv[i + 1]);
            }
        }

        if (strcmp(arg, "--interpolation") == 0 || strcmp(arg, "-i") == 0)
        {
            if (legalNumber(argv[i + 1]))
            {
                interpolation = atol(argv[i + 1]);
            }
        }
    }
    std::cout << "\n";
}

int main(int argc, char *argv[])
{
    parseArgs(argc, argv);

    plan();
}
