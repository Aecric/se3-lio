// Fixed-map pure localization entry point; command line as Super-LIO's run_loclite_online.
#include <iostream>

#include "loc_node.h"

int main(int argc, char **argv) {
    try {
        auto cli = loclite::ParseCommandLine(argc, argv);
        if (cli.help) {
            std::cout << "Usage: ros2 run se3_lio run_loclite_online --config <ROS parameter YAML> "
                         "[--map_path <directory containing global.pcd>] [--ros-args ...]\n";
            return 0;
        }
        if (cli.config.empty())
            throw std::runtime_error("--config is required (use config/loclite_livox.yaml)");
        auto &args = cli.args;
        // Load the file first; ordinary ROS command-line overrides follow it.
        args.insert(args.begin() + 1, {"--ros-args", "--params-file", cli.config, "--"});
        if (!cli.map_path.empty())
            args.insert(args.end(), {"--ros-args", "-p", "system.map_path:=" + cli.map_path});
        std::vector<char *> raw;
        for (auto &arg : args) raw.push_back(arg.data());
        rclcpp::init(static_cast<int>(raw.size()), raw.data());

        auto node = std::make_shared<se3_lio::ros2_node::LocNode>();
        node->run();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "se3_lio localization: " << error.what() << '\n';
        if (rclcpp::ok()) rclcpp::shutdown();
        return 1;
    }
}
