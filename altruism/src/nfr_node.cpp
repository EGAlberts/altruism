/* Copyright (C) 2015-2018 Michele Colledanchise -  All Rights Reserved
 * Copyright (C) 2018-2020 Davide Faconti, Eurecat -  All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "altruism/nfr_node.h"

namespace BT
{
NFRNode::NFRNode(const std::string& name, int weight, std::string property_name) :
  DecoratorNode(name, {}),
  weight_(weight),
  property_name_(0),
  read_parameter_from_ports_(false)
{
  setRegistrationID("NFR");
}

NFRNode::NFRNode(const std::string& name, const NodeConfig& config) :
  DecoratorNode(name, config),
  weight_(1),
  property_name_("unspecified"),
  read_parameter_from_ports_(true)
{}

NodeStatus NFRNode::tick()
{
  if (read_parameter_from_ports_)
  {
    if (!getInput(WEIGHT, weight_))
    {
      throw RuntimeError("Missing parameter [", WEIGHT, "] in NFRNode");
    }

    if (!getInput(PROPERTY_NAME, property_name_))
    {
      throw RuntimeError("Missing parameter [", PROPERTY_NAME, "] in NFRNode");
    }
  }

  if(status() == NodeStatus::IDLE)
  {
    all_skipped_ = true;
  }
  setStatus(NodeStatus::RUNNING);

  while (do_loop)
  {
    NodeStatus const prev_status = child_node_->status();
    NodeStatus child_status = child_node_->executeTick();

    // switch to RUNNING state as soon as you find an active child
    all_skipped_ &= (child_status == NodeStatus::SKIPPED);

    switch (child_status)
    {
      case NodeStatus::SUCCESS: {

      break;
      }
      case NodeStatus::FAILURE: {
        resetChild();
        return (NodeStatus::FAILURE);
      }

      case NodeStatus::RUNNING: {
        return NodeStatus::RUNNING;
      }

      case NodeStatus::SKIPPED: {
        // the child has been skipped. Skip the decorator too.
        return NodeStatus::IDLE;
      }
      case NodeStatus::IDLE: {
        throw LogicError("[", name(), "]: A children should not return IDLE");
      }
    }
  }

  return all_skipped_ ? NodeStatus::SKIPPED : NodeStatus::SUCCESS;
}


void NFRNode::halt()
{
  DecoratorNode::halt();
}


}   // namespace BT
