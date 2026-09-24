#define main cm_kirara_ball_original_move_main
#include "CM_KIRARA_BALL_MOVE.cpp"
#undef main

#include <iomanip>
#include <sstream>

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "usage: CM_KIRARA_BALL_MOVE_VALIDATOR CONFIG.yaml"
                  << std::endl;
        return 2;
    }

    const YAML::Node config = YAML::LoadFile(argv[1]);
    Obs[0] = config["ball_collision"]["x"].as<double>();
    Obs[1] = config["ball_collision"]["y"].as<double>();
    Obs[2] = config["ball_collision"]["z"].as<double>();
    radius = config["ball_collision"]["r"].as<double>();
    radius_arm = config["arm"]["r"].as<double>();
    param_eff = config["cost"]["eff"].as<double>();
    param_rot = config["cost"]["rot"].as<double>();
    param_cos = config["cost"]["cos"].as<double>();
    param_cos_2 = config["cost"]["cos_2"].as<double>();
    param_object = config["cost"]["object"].as<double>();
    urdf_filename = config["file_path"]["pinocchio_file"].as<std::string>();
    const std::string modelPath =
        config["model_path"]["pinocchio_path"].as<std::string>();

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);
    pinocchio::GeometryModel geometryModel;
    pinocchio::urdf::buildGeom(
        model, urdf_filename, pinocchio::COLLISION,
        geometryModel, modelPath);
    geometryModel.addAllCollisionPairs();
    pinocchio::GeometryData geometryData(geometryModel);

    ob::StateSpacePtr space(new ob::RealVectorStateSpace(12));
    ob::RealVectorBounds bounds(12);
    bounds.setLow(-2.0 * M_PI);
    bounds.setHigh(2.0 * M_PI);
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    std::cout << "READY move " << model.nq << ' '
              << geometryModel.collisionPairs.size() << std::endl;
    std::string line;
    while (std::getline(std::cin, line))
    {
        if (line == "X")
            break;
        std::istringstream input(line);
        char command = 0;
        std::uint64_t sequence = 0;
        input >> command >> sequence;
        if (command != 'Q')
        {
            std::cout << "E " << sequence << " invalid_command" << std::endl;
            continue;
        }
        ob::ScopedState<> state(space);
        bool parsed = true;
        for (int joint = 0; joint < 12; ++joint)
        {
            if (!(input >> state[joint]))
            {
                parsed = false;
                break;
            }
        }
        if (!parsed)
        {
            std::cout << "E " << sequence << " expected_12_joints" << std::endl;
            continue;
        }

        std::ostringstream suppressed;
        std::streambuf *oldBuffer = std::cout.rdbuf(suppressed.rdbuf());
        bool valid = false;
        try
        {
            valid = isStateValid(
                state.get(), model, data, geometryModel, geometryData);
        }
        catch (...)
        {
            std::cout.rdbuf(oldBuffer);
            throw;
        }
        std::cout.rdbuf(oldBuffer);
        std::cout << "V " << sequence << ' ' << (valid ? 1 : 0)
                  << std::endl;
    }
    return 0;
}
