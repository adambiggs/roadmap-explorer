/**
    Copyright 2025 Suchetan Saravanan.

    Licensed to the Apache Software Foundation (ASF) under one
    or more contributor license agreements.  See the NOTICE file
    distributed with this work for additional information
    regarding copyright ownership.  The ASF licenses this file
    to you under the Apache License, Version 2.0 (the
    "License"); you may not use this file except in compliance
    with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing,
    software distributed under the License is distributed on an
    "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
    KIND, either express or implied.  See the License for the
    specific language governing permissions and limitations
    under the License.
*/

#ifndef ROADMAP_EXPLORER__BT_PLUGINS__BOOTSTRAP_RANDOM_WALK_PLUGIN_HPP_
#define ROADMAP_EXPLORER__BT_PLUGINS__BOOTSTRAP_RANDOM_WALK_PLUGIN_HPP_

#include "roadmap_explorer/bt_plugins/BaseBTPlugin.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace roadmap_explorer
{

class BootstrapRandomWalkPlugin : public BTPlugin
{
public:
    BootstrapRandomWalkPlugin();
    ~BootstrapRandomWalkPlugin() override;
    
    void registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<BTContext> context) override;
};

} // namespace roadmap_explorer

#endif  // ROADMAP_EXPLORER__BT_PLUGINS__BOOTSTRAP_RANDOM_WALK_PLUGIN_HPP_
