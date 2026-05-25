#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h> // For sleep
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "global_robot_localization/global_robot_localization.hpp"
#include "global_robot_localization/action/localize_in_map.hpp"
#include "global_robot_localization/sim_node.hpp"
#include "global_robot_localization/map_model.hpp"

using namespace std::chrono_literals;

TEST(GlobalLocFunctionalTest, TestNodeBringup) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<global_robot_localization::GlobalRobotLocalization>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    // give the node some time to come up
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // while (rclcpp::ok() && !node->is_complete()) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }

    executor.cancel();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    executor.remove_node(node);
    rclcpp::shutdown();
}

TEST(GlobalLocFunctionalTest, TestNodeActionNoMap) {
    // bring up the node and send it an action request.
    // this request should fail (generally), because the
    // map provider is not up (generally).
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<global_robot_localization::GlobalRobotLocalization>();
    auto client_node = std::make_shared<rclcpp::Node>("action_client");

    auto actionClient = rclcpp_action::create_client<global_robot_localization::action::LocalizeInMap>(client_node, "LocalizeInMap");

    using ActionT = global_robot_localization::action::LocalizeInMap;
    //using GoalHandleT = rclcpp_action::ClientGoalHandle<ActionT>;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(client_node);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    bool client_up = actionClient->wait_for_action_server(5s);

    if (client_up) std::cout << "server up!" << std::endl;

    ActionT::Goal goal;
    rclcpp_action::Client<ActionT>::SendGoalOptions options;

    // send the message to the node
    auto goal_future = actionClient->async_send_goal(goal, options);
    goal_future.wait_for(5s);

    bool goal_response;

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
        goal_response = false;
        std::cout << "Goal rejected" << std::endl;
    } else {
        goal_response = true;
        std::cout << "Goal accepted" << std::endl;
    }

    executor.cancel();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    executor.remove_node(node);
    executor.remove_node(client_node);
    rclcpp::shutdown();

    ASSERT_FALSE(goal_response);
    
}

TEST(GlobalLocFunctionalTest, TestNodeAction) {
    // bring up the localization node and the sim node 
    // and send a localization request. The request
    // should succeed and return the proper robot localization.
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);

    // load up the config file
    auto params_file = 
        ament_index_cpp::get_package_share_directory("global_robot_localization") +
        "/params/test.yaml";

    rclcpp::NodeOptions node_options;
    node_options.arguments({
        "--ros-args",
        "--params-file",
        params_file
    });

    auto node = std::make_shared<global_robot_localization::GlobalRobotLocalization>(node_options);
    auto sim_node = std::make_shared<global_robot_localization::SimNode>();
    auto client_node = std::make_shared<rclcpp::Node>("action_client");

    auto actionClient = rclcpp_action::create_client<global_robot_localization::action::LocalizeInMap>(client_node, "LocalizeInMap");

    using ActionT = global_robot_localization::action::LocalizeInMap;
    //using GoalHandleT = rclcpp_action::ClientGoalHandle<ActionT>;
    //using WrappedResultT = rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(sim_node);
    executor.add_node(client_node);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    bool client_up = actionClient->wait_for_action_server(5s);

    if (client_up) std::cout << "server up!" << std::endl;

    ActionT::Goal goal;
    rclcpp_action::Client<ActionT>::SendGoalOptions options;

    // send the message to the node
    auto goal_future = actionClient->async_send_goal(goal, options);
    goal_future.wait_for(5s);

    bool goal_response;

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
        goal_response = false;
        std::cout << "Goal rejected" << std::endl;
    } else {
        goal_response = true;
        std::cout << "Goal accepted" << std::endl;
    }

    bool goal_accomplished = false;

    // this is all a bit challenging without vscode intellisense
    // working for types associated with node actions, so I am doing the stupid thing!
    double x = std::numeric_limits<double>::quiet_NaN();
    double y = std::numeric_limits<double>::quiet_NaN();
    double yaw = std::numeric_limits<double>::quiet_NaN();

    std::array<double, 36UL> covariance;

    if (goal_response){
        auto result_future = actionClient->async_get_result(goal_handle);
        auto res_fut = result_future.wait_for(30s);
        if (res_fut == std::future_status::ready)
        {
            auto wrapped_result = result_future.get();
            std::cout << wrapped_result.result->message << std::endl;
            goal_accomplished = true;
            auto pose = wrapped_result.result->pose.pose.pose.position;
            auto orientation = wrapped_result.result->pose.pose.pose.orientation;
            yaw = global_robot_localization::yawFromQuaternion(orientation);
            x = pose.x;
            y = pose.y;
            std::cout << "estimated pose: x=" << pose.x << " y=" << pose.y
                << " yaw=" << yaw << " cost=" << wrapped_result.result->cost << std::endl;
            covariance = wrapped_result.result->pose.pose.covariance;
            std::cout << "covariance: ";
            // stone knives and bearskins, man!
            for (size_t r = 0; r < 6; ++r) {
                for (size_t c = 0; c < 6; ++c) {
                    std::cout << covariance[r * 6 + c] << "\t";
                }
                std::cout << "\n";
            }
            std::cout << std::endl;   
        }
    }

    executor.cancel();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    executor.remove_node(node);
    executor.remove_node(sim_node);
    executor.remove_node(client_node);
    rclcpp::shutdown();

    double x_truth;
    sim_node->get_parameter("robot_x", x_truth);
    double y_truth;
    sim_node->get_parameter("robot_y", y_truth);   
    double yaw_truth;
    sim_node->get_parameter("robot_yaw", yaw_truth);  

    ASSERT_TRUE(goal_response);
    ASSERT_TRUE(goal_accomplished);
    // yes this is a little shabby, I am cleaning up after codex.
    EXPECT_LT(global_robot_localization::yawError(yaw, yaw_truth), 5.0 * global_robot_localization::kPi / 180.0);
    EXPECT_LT(std::hypot(x - x_truth, y - y_truth), 0.18);
    EXPECT_GT(covariance[0], 0.0);
    EXPECT_GT(covariance[7], 0.0);
    EXPECT_GT(covariance[14], 0.0);
    EXPECT_GT(covariance[21], 0.0);
    EXPECT_GT(covariance[28], 0.0);
    EXPECT_GT(covariance[35], 0.0);
    for (size_t i = 0; i < covariance.size(); ++i) {
        ASSERT_TRUE(std::isfinite(covariance[i])) << "Non-finite at index " << i;
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}