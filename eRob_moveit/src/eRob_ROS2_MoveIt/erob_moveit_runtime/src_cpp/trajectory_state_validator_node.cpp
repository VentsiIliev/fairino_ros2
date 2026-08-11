#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "erob_moveit_runtime/srv/validate_trajectory_states.hpp"

#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>

using ValidateTrajectoryStates =
    erob_moveit_runtime::srv::ValidateTrajectoryStates;

namespace
{

constexpr int32_t ERROR_NONE = 1;
constexpr int32_t ERROR_INVALID_REQUEST = -1;
constexpr int32_t ERROR_BOUNDS = -2;
constexpr int32_t ERROR_COLLISION = -10;

double steadySeconds()
{
    using Clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(
        Clock::now().time_since_epoch()).count();
}

bool valuesAreFinite(const std::vector<double>& values)
{
    return std::all_of(
        values.begin(),
        values.end(),
        [](double value) { return std::isfinite(value); });
}

std::string contactSummary(
    const collision_detection::CollisionResult::ContactMap& contacts)
{
    if (contacts.empty())
    {
        return "";
    }

    std::ostringstream out;
    out << " contacts=[";

    bool first = true;
    for (const auto& contact_pair : contacts)
    {
        if (!first)
        {
            out << ", ";
        }
        first = false;
        out
            << contact_pair.first.first
            << "<->"
            << contact_pair.first.second;
    }

    out << "]";
    return out.str();
}

}  // namespace

class TrajectoryStateValidatorNode : public rclcpp::Node
{
public:
    TrajectoryStateValidatorNode()
        : Node("trajectory_state_validator")
    {
        RCLCPP_INFO(get_logger(), "Trajectory state validator starting...");
    }

    void initialize()
    {
        auto node_ptr = shared_from_this();

        loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
            node_ptr);
        model_ = loader_->getModel();
        if (!model_)
        {
            throw std::runtime_error(
                "Trajectory state validator failed to load robot model");
        }

        planning_scene_monitor_ =
            std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
                node_ptr,
                loader_,
                "trajectory_state_validator_planning_scene_monitor");

        if (!planning_scene_monitor_->getPlanningScene())
        {
            throw std::runtime_error(
                "Trajectory state validator failed to create PlanningSceneMonitor");
        }

        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();

        if (!planning_scene_monitor_->requestPlanningSceneState(
                "/get_planning_scene"))
        {
            RCLCPP_WARN(
                get_logger(),
                "Could not request initial planning scene; continuing with monitored scene updates");
        }

        service_ = create_service<ValidateTrajectoryStates>(
            "/validate_trajectory_states",
            [this](
                const std::shared_ptr<ValidateTrajectoryStates::Request> request,
                std::shared_ptr<ValidateTrajectoryStates::Response> response)
            {
                handle(request, response);
            });

        RCLCPP_INFO(
            get_logger(),
            "Trajectory state validator ready");
    }

private:
    struct Failure
    {
        bool failed = false;
        int32_t error_code = ERROR_NONE;
        std::size_t index = std::numeric_limits<std::size_t>::max();
        std::string message;
    };

    rclcpp::Service<ValidateTrajectoryStates>::SharedPtr service_;
    robot_model_loader::RobotModelLoaderPtr loader_;
    moveit::core::RobotModelPtr model_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    void fail(
        const std::shared_ptr<ValidateTrajectoryStates::Response>& response,
        int32_t error_code,
        const std::string& message,
        double started_s,
        uint32_t failed_index = 0,
        uint32_t states_checked = 0) const
    {
        response->success = false;
        response->valid = false;
        response->error_code = error_code;
        response->message = message;
        response->failed_index = failed_index;
        response->states_checked = states_checked;
        response->validation_time_s = steadySeconds() - started_s;
    }

    bool validateRequest(
        const ValidateTrajectoryStates::Request& request,
        const std::shared_ptr<ValidateTrajectoryStates::Response>& response,
        double started_s) const
    {
        const std::size_t joint_count = request.joint_names.size();
        const std::size_t state_count =
            static_cast<std::size_t>(request.state_count);

        if (joint_count == 0)
        {
            fail(response, ERROR_INVALID_REQUEST, "request has no joint names", started_s);
            return false;
        }

        if (state_count == 0)
        {
            fail(response, ERROR_INVALID_REQUEST, "request has no states", started_s);
            return false;
        }

        if (request.positions.size() != state_count * joint_count)
        {
            std::ostringstream out;
            out
                << "positions length "
                << request.positions.size()
                << " does not match state_count * joint_count "
                << state_count
                << " * "
                << joint_count;
            fail(response, ERROR_INVALID_REQUEST, out.str(), started_s);
            return false;
        }

        if (!valuesAreFinite(request.positions))
        {
            fail(response, ERROR_INVALID_REQUEST, "positions contain non-finite values", started_s);
            return false;
        }

        if (request.group_name.empty())
        {
            fail(response, ERROR_INVALID_REQUEST, "request has no group_name", started_s);
            return false;
        }

        return true;
    }

    Failure validateOne(
        const ValidateTrajectoryStates::Request& request,
        const planning_scene::PlanningSceneConstPtr& scene,
        const moveit::core::JointModelGroup* group,
        std::size_t index) const
    {
        const std::size_t joint_count = request.joint_names.size();
        const std::size_t offset = index * joint_count;

        std::vector<double> q(
            request.positions.begin() + static_cast<std::ptrdiff_t>(offset),
            request.positions.begin() + static_cast<std::ptrdiff_t>(offset + joint_count));

        moveit::core::RobotState state(model_);
        state.setToDefaultValues();

        try
        {
            state.setVariablePositions(request.joint_names, q);
        }
        catch (const std::exception& exc)
        {
            Failure failure;
            failure.failed = true;
            failure.error_code = ERROR_INVALID_REQUEST;
            failure.index = index;
            failure.message = std::string("failed to set joint positions: ") + exc.what();
            return failure;
        }

        state.update();

        if (!state.satisfiesBounds(group))
        {
            Failure failure;
            failure.failed = true;
            failure.error_code = ERROR_BOUNDS;
            failure.index = index;
            std::ostringstream out;
            out << "state " << index << " violates joint bounds";
            failure.message = out.str();
            return failure;
        }

        if (!request.check_collisions)
        {
            return Failure{};
        }

        if (scene->isStateColliding(state, request.group_name, false))
        {
            collision_detection::CollisionRequest collision_request;
            collision_detection::CollisionResult collision_result;
            collision_request.group_name = request.group_name;
            collision_request.contacts = true;
            collision_request.max_contacts = 20;
            collision_request.max_contacts_per_pair = 5;

            scene->checkCollision(
                collision_request,
                collision_result,
                state);

            Failure failure;
            failure.failed = true;
            failure.error_code = ERROR_COLLISION;
            failure.index = index;

            std::ostringstream out;
            out
                << "collision at state "
                << index
                << "/"
                << (static_cast<std::size_t>(request.state_count) - 1)
                << contactSummary(collision_result.contacts);
            failure.message = out.str();
            return failure;
        }

        return Failure{};
    }

    Failure validateRange(
        const ValidateTrajectoryStates::Request& request,
        const planning_scene::PlanningSceneConstPtr& scene,
        const moveit::core::JointModelGroup* group,
        std::size_t begin,
        std::size_t end,
        std::atomic<std::size_t>& earliest_failure) const
    {
        Failure best;

        for (std::size_t index = begin; index < end; ++index)
        {
            const std::size_t cutoff =
                earliest_failure.load(std::memory_order_relaxed);
            if (index > cutoff)
            {
                break;
            }

            Failure failure = validateOne(request, scene, group, index);
            if (!failure.failed)
            {
                continue;
            }

            std::size_t observed = earliest_failure.load(std::memory_order_relaxed);
            while (failure.index < observed
                   && !earliest_failure.compare_exchange_weak(
                       observed,
                       failure.index,
                       std::memory_order_relaxed))
            {
            }

            best = std::move(failure);
            break;
        }

        return best;
    }

    void handle(
        const std::shared_ptr<ValidateTrajectoryStates::Request> request,
        std::shared_ptr<ValidateTrajectoryStates::Response> response)
    {
        const double started_s = steadySeconds();

        if (!validateRequest(*request, response, started_s))
        {
            return;
        }

        const auto* group = model_->getJointModelGroup(request->group_name);
        if (group == nullptr)
        {
            fail(
                response,
                ERROR_INVALID_REQUEST,
                "unknown planning group: " + request->group_name,
                started_s);
            return;
        }

        planning_scene_monitor::LockedPlanningSceneRO locked_scene(
            planning_scene_monitor_);
        if (!locked_scene)
        {
            fail(
                response,
                ERROR_INVALID_REQUEST,
                "PlanningScene is unavailable",
                started_s);
            return;
        }

        const planning_scene::PlanningSceneConstPtr scene =
            static_cast<planning_scene::PlanningSceneConstPtr>(locked_scene);

        const std::size_t state_count =
            static_cast<std::size_t>(request->state_count);
        const uint32_t hardware_workers =
            std::max(1u, std::thread::hardware_concurrency());
        const uint32_t requested_workers =
            request->max_workers == 0 ? 1u : request->max_workers;
        const std::size_t worker_count = std::max<std::size_t>(
            1,
            std::min<std::size_t>(
                state_count,
                std::min<uint32_t>(requested_workers, hardware_workers)));

        std::atomic<std::size_t> earliest_failure{
            std::numeric_limits<std::size_t>::max()};
        std::vector<Failure> failures(worker_count);
        std::vector<std::thread> workers;
        workers.reserve(worker_count);

        const std::size_t chunk =
            (state_count + worker_count - 1) / worker_count;
        for (std::size_t worker = 0; worker < worker_count; ++worker)
        {
            const std::size_t begin = worker * chunk;
            const std::size_t end = std::min(state_count, begin + chunk);
            if (begin >= end)
            {
                continue;
            }

            workers.emplace_back(
                [this, request, scene, group, begin, end, &earliest_failure, &failures, worker]()
                {
                    failures[worker] = validateRange(
                        *request,
                        scene,
                        group,
                        begin,
                        end,
                        earliest_failure);
                });
        }

        for (auto& worker : workers)
        {
            worker.join();
        }

        Failure best;
        for (const auto& failure : failures)
        {
            if (!failure.failed)
            {
                continue;
            }
            if (!best.failed || failure.index < best.index)
            {
                best = failure;
            }
        }

        if (best.failed)
        {
            response->success = true;
            response->valid = false;
            response->error_code = best.error_code;
            response->message = best.message;
            response->failed_index = static_cast<uint32_t>(best.index);
            response->states_checked = static_cast<uint32_t>(
                std::min<std::size_t>(
                    state_count,
                    earliest_failure.load(std::memory_order_relaxed) + 1));
            response->validation_time_s = steadySeconds() - started_s;
            return;
        }

        response->success = true;
        response->valid = true;
        response->error_code = ERROR_NONE;
        response->message = "ok";
        response->failed_index = 0;
        response->states_checked = static_cast<uint32_t>(state_count);
        response->validation_time_s = steadySeconds() - started_s;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryStateValidatorNode>();

    try
    {
        node->initialize();
    }
    catch (const std::exception& exc)
    {
        RCLCPP_FATAL(
            node->get_logger(),
            "Failed to initialize trajectory state validator: %s",
            exc.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
