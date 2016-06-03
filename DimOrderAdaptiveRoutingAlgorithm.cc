/*
 * Copyright 2016 Hewlett Packard Enterprise Development LP
 *
 * Licensed under the Apache License, Version 2.0 (the 'License');
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "network/hyperx/DimOrderAdaptiveRoutingAlgorithm.h"

#include <cassert>

#include <unordered_set>

#include "types/Message.h"
#include "types/Packet.h"

namespace HyperX {

DimOrderAdaptiveRoutingAlgorithm::DimOrderAdaptiveRoutingAlgorithm(
    const std::string& _name, const Component* _parent, Router* _router,
    u64 _latency, u32 _numVcs, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration)
    : RoutingAlgorithm(_name, _parent, _router, _latency),
      numVcs_(router_->numVcs()), dimensionWidths_(_dimensionWidths),
      dimensionWeights_(_dimensionWeights),
      concentration_(_concentration) {
  assert(numVcs_ >= 2);
}

DimOrderAdaptiveRoutingAlgorithm::~DimOrderAdaptiveRoutingAlgorithm() {}

void DimOrderAdaptiveRoutingAlgorithm::processRequest(
    Flit* _flit, RoutingAlgorithm::Response* _response) {
  std::unordered_set<u32> outputPorts;
  Packet* packet = _flit->getPacket();

  // ex: [x,y,z]
  const std::vector<u32>& routerAddress = router_->getAddress();
  // ex: [c,x,y,z]
  const std::vector<u32>* destinationAddress =
      _flit->getPacket()->getMessage()->getDestinationAddress();
  assert(routerAddress.size() == (destinationAddress->size() - 1));

  // first, test if already at destination router
  bool arrived = true;
  for (u32 mydim = 0; mydim < routerAddress.size(); mydim++) {
    if (routerAddress.at(mydim) != destinationAddress->at(mydim+1)) {
      arrived = false;
      break;
    }
  }

  if (arrived) {
    bool res = outputPorts.insert(destinationAddress->at(0)).second;
    (void)res;
    assert(res);
  } else {
    // determine which dimension to work on
    u32 dim;
    u32 portBase = concentration_;
    u32 numPorts = 0;             // excludes terminal ports
    std::vector<u32> portBases;
    portBases.resize(routerAddress.size());

    // get number of ports
    for (dim = 0; dim < routerAddress.size(); dim++) {
      numPorts += dimensionWeights_[dim];
      if (routerAddress.at(dim) == destinationAddress->at(dim+1)) {
        portBases[dim] = 0;
      } else {
        portBases[dim] = portBase;
      }
      portBase += ((dimensionWidths_.at(dim) - 1) * dimensionWeights_.at(dim));
    }

    // else get list of valid ports
    u32 prt = 0;
    std::vector<u32> valid_ports;
    valid_ports.resize(numPorts);

    for (dim = 0; dim < routerAddress.size(); dim++) {
      if (portBases[dim] != 0) {
        // more router-to-router hops needed
        u32 src = routerAddress.at(dim);
        u32 dst = destinationAddress->at(dim+1);
        if (dst < src) {
          dst += dimensionWidths_.at(dim);
        }
        u32 offset = (dst - src - 1) * dimensionWeights_.at(dim);
        // add all ports where the two routers are connecting
        for (u32 weight = 0; weight < dimensionWeights_.at(dim); weight++) {
          valid_ports[prt] = portBases[dim] + offset + weight;
          prt++;
        }
      }
    }
    valid_ports.resize(prt);

    // calculate max availability for each port
    u32 vcIdx;
    u32 outputPort;
    f64 max = 0;
    f64 availability;
    f64 total_availability;
    f64 avg_availability;
    std::unordered_map<u32, f64> port_availability =
             std::unordered_map<u32, f64>(numPorts);

    // for each output port
    for (u32 port : valid_ports) {
      total_availability = 0;
      avg_availability = 0;

      // for each VC
      for (u32 vc = 0; vc < numVcs_; vc++) {
        vcIdx = router_->vcIndex(port, vc);
        availability = router_->congestionStatus(vcIdx);
        total_availability += availability;
      }
      avg_availability = total_availability / ((f64)numVcs_);

      // add to vector of ports availability
      port_availability.insert({port, avg_availability});

      if ( avg_availability > max ) {
        max = avg_availability;
      }
    }

    // get ports with max availability
    std::vector<u32> max_ports = std::vector<u32>(numPorts);
    u32 max_counter = 0;

    for ( auto it = port_availability.begin();
         it != port_availability.end(); ++it ) {
      if (it->second == max) {
        max_ports.at(max_counter) = it->first;
        max_counter++;
      }
    }

    // randomly choose an output port with max availability
    u32 portIdx = gSim->rnd.nextU64(0, max_counter - 1);
    outputPort = max_ports.at(portIdx);

    bool res = outputPorts.insert(outputPort).second;
    (void)res;
    assert(res);
  }

  u32 vcSet = (_flit->getVc()+1) % 2;

  assert(outputPorts.size() > 0);
  for (auto it = outputPorts.cbegin(); it != outputPorts.cend(); ++it) {
    u32 outputPort = *it;
    // if arrived, select all VCs in the output port
    for (u32 vc = 0; vc < numVcs_; vc++) {
      _response->add(outputPort, vc);
    }
    // else, pump up the VC set by one
    for (u32 vc = vcSet; vc < numVcs_; vc+=2) {
      _response->add(outputPort, vc);
    }
  }
}

}  // namespace HyperX
