// core_plugins.hpp
#pragma once

#include <roadmap_explorer/bt_plugins/BaseBTPlugin.hpp>
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/util/EventLogger.hpp"

namespace roadmap_explorer
{
    class LogIteration : public BTPlugin
    {
        public:
        LogIteration();

        ~LogIteration();

        void registerNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<BTContext> context) override;
    };
};
