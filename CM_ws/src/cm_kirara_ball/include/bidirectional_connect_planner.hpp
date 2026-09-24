#ifndef CM_BIDIRECTIONAL_CONNECT_PLANNER_HPP
#define CM_BIDIRECTIONAL_CONNECT_PLANNER_HPP

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <ompl/base/Planner.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/geometric/PathGeometric.h>

namespace cm_planning
{

// Project-owned, bidirectional sampling planner.  It grows one search tree
// from each endpoint, alternates the active side, and repeatedly advances the
// opposite side toward the newest state.  Collision validity remains entirely
// delegated to the SpaceInformation supplied by the calling application.
class BidirectionalConnectPlanner final : public ompl::base::Planner
{
public:
    explicit BidirectionalConnectPlanner(
        const ompl::base::SpaceInformationPtr &spaceInformation)
        : ompl::base::Planner(spaceInformation, "CM_BidirectionalConnectPlanner")
    {
        initializeTree(startTree_);
        initializeTree(goalTree_);
    }

    ~BidirectionalConnectPlanner() override
    {
        releaseTree(startTree_);
        releaseTree(goalTree_);
    }

    void setRange(double range)
    {
        range_ = range;
    }

    double getRange() const
    {
        return range_;
    }

    void clear() override
    {
        ompl::base::Planner::clear();
        releaseTree(startTree_);
        releaseTree(goalTree_);
        sampler_.reset();
    }

    ompl::base::PlannerStatus solve(
        const ompl::base::PlannerTerminationCondition &termination) override
    {
        checkValidity();
        releaseTree(startTree_);
        releaseTree(goalTree_);

        const ompl::base::State *start = pis_.nextStart();
        if (start == nullptr)
        {
            return ompl::base::PlannerStatus::INVALID_START;
        }

        const ompl::base::State *goal = pis_.nextGoal(termination);
        if (goal == nullptr)
        {
            return ompl::base::PlannerStatus::INVALID_GOAL;
        }

        const std::size_t startRoot = addNode(startTree_, start, noParent());
        const std::size_t goalRoot = addNode(goalTree_, goal, noParent());

        if (si_->checkMotion(startTree_.nodes[startRoot]->state,
                             goalTree_.nodes[goalRoot]->state))
        {
            publishSolution(startRoot, goalRoot);
            return ompl::base::PlannerStatus::EXACT_SOLUTION;
        }

        if (!sampler_)
        {
            sampler_ = si_->allocStateSampler();
        }
        ompl::base::State *sample = si_->allocState();
        bool growFromStart = true;

        while (!termination)
        {
            sampler_->sampleUniform(sample);
            Tree &active = growFromStart ? startTree_ : goalTree_;
            Tree &opposite = growFromStart ? goalTree_ : startTree_;

            std::size_t activeIndex = noParent();
            const AdvanceResult activeResult =
                advanceOnce(active, sample, activeIndex);
            if (activeResult != AdvanceResult::Blocked)
            {
                std::size_t oppositeIndex = noParent();
                AdvanceResult oppositeResult = AdvanceResult::Blocked;
                do
                {
                    oppositeResult = advanceOnce(
                        opposite, active.nodes[activeIndex]->state, oppositeIndex);
                }
                while (oppositeResult == AdvanceResult::Advanced && !termination);

                if (oppositeResult == AdvanceResult::Reached)
                {
                    if (growFromStart)
                    {
                        publishSolution(activeIndex, oppositeIndex);
                    }
                    else
                    {
                        publishSolution(oppositeIndex, activeIndex);
                    }
                    si_->freeState(sample);
                    return ompl::base::PlannerStatus::EXACT_SOLUTION;
                }
            }

            growFromStart = !growFromStart;
        }

        si_->freeState(sample);
        return ompl::base::PlannerStatus::TIMEOUT;
    }

private:
    enum class AdvanceResult
    {
        Blocked,
        Advanced,
        Reached
    };

    struct Node
    {
        ompl::base::State *state{nullptr};
        std::size_t parent{noParent()};
        std::size_t index{noParent()};
    };

    struct Tree
    {
        std::vector<Node *> nodes;
        std::shared_ptr<ompl::NearestNeighbors<Node *>> nearest;
    };

    static std::size_t noParent()
    {
        return std::numeric_limits<std::size_t>::max();
    }

    void initializeTree(Tree &tree)
    {
        tree.nearest = std::make_shared<
            ompl::NearestNeighborsGNATNoThreadSafety<Node *>>();
        tree.nearest->setDistanceFunction(
            [this](Node *const &first, Node *const &second)
            {
                return si_->distance(first->state, second->state);
            });
    }

    void releaseTree(Tree &tree)
    {
        if (tree.nearest)
        {
            tree.nearest->clear();
        }
        for (Node *node : tree.nodes)
        {
            if (node != nullptr)
            {
                si_->freeState(node->state);
                delete node;
            }
        }
        tree.nodes.clear();
    }

    std::size_t addNode(Tree &tree,
                        const ompl::base::State *state,
                        std::size_t parent)
    {
        Node *node = new Node();
        node->state = si_->cloneState(state);
        node->parent = parent;
        node->index = tree.nodes.size();
        tree.nodes.push_back(node);
        tree.nearest->add(node);
        return node->index;
    }

    AdvanceResult advanceOnce(Tree &tree,
                              const ompl::base::State *target,
                              std::size_t &resultIndex)
    {
        Node query;
        query.state = const_cast<ompl::base::State *>(target);
        Node *nearest = tree.nearest->nearest(&query);
        const double distance = si_->distance(nearest->state, target);

        if (distance <= 1e-12)
        {
            resultIndex = nearest->index;
            return AdvanceResult::Reached;
        }

        ompl::base::State *candidate = si_->allocState();
        const bool reachesTarget = distance <= range_;
        if (reachesTarget)
        {
            si_->copyState(candidate, target);
        }
        else
        {
            si_->getStateSpace()->interpolate(
                nearest->state, target, range_ / distance, candidate);
        }

        if (!si_->checkMotion(nearest->state, candidate))
        {
            si_->freeState(candidate);
            resultIndex = noParent();
            return AdvanceResult::Blocked;
        }

        resultIndex = addNode(tree, candidate, nearest->index);
        si_->freeState(candidate);
        return reachesTarget ? AdvanceResult::Reached : AdvanceResult::Advanced;
    }

    void publishSolution(std::size_t startConnection,
                         std::size_t goalConnection)
    {
        std::vector<const ompl::base::State *> ordered;
        for (std::size_t index = startConnection;
             index != noParent();
             index = startTree_.nodes[index]->parent)
        {
            ordered.push_back(startTree_.nodes[index]->state);
        }
        std::reverse(ordered.begin(), ordered.end());

        for (std::size_t index = goalConnection;
             index != noParent();
             index = goalTree_.nodes[index]->parent)
        {
            const ompl::base::State *state = goalTree_.nodes[index]->state;
            if (!ordered.empty() && si_->distance(ordered.back(), state) <= 1e-12)
            {
                continue;
            }
            ordered.push_back(state);
        }

        auto path = std::make_shared<ompl::geometric::PathGeometric>(si_);
        for (const ompl::base::State *state : ordered)
        {
            path->append(state);
        }
        pdef_->addSolutionPath(path, false, 0.0, getName());
    }

    Tree startTree_;
    Tree goalTree_;
    ompl::base::StateSamplerPtr sampler_;
    double range_{0.25};
};

}  // namespace cm_planning

#endif
