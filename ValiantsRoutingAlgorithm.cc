/*
 * Copyright 2016 Ashish Chaudhari, Franky Romero, Nehal Bhandari, Wasam Altoyan
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
#include "network/hyperx/ValiantsRoutingAlgorithm.h"

#include <cassert>

#include <unordered_set>

#include "types/Message.h"
#include "types/Packet.h"

namespace HyperX {

ValiantsRoutingAlgorithm::ValiantsRoutingAlgorithm(
    const std::string& _name, const Component* _parent, Router* _router,
    u64 _latency, u32 _numVcs, const std::vector<u32>& _dimensionWidths,
    const std::vector<u32>& _dimensionWeights, u32 _concentration)
    : RoutingAlgorithm(_name, _parent, _router, _latency),
      numVcs_(router_->numVcs()), dimensionWidths_(_dimensionWidths),
      dimensionWeights_(_dimensionWeights),
      concentration_(_concentration) {}

ValiantsRoutingAlgorithm::~ValiantsRoutingAlgorithm() {}

void ValiantsRoutingAlgorithm::processRequest(
    Flit* _flit, RoutingAlgorithm::Response* _response) {

  u32 packetId = _flit->getPacket()->getId();
  u32 messageId = _flit->getPacket()->getMessage()->getId();

  std::unordered_set<u32> outputPorts;
  u32 outputPort;

  // ex: [x,y,z]
  const std::vector<u32>& routerAddress = router_->getAddress();

  Packet* packet = _flit->getPacket();
  Message* message = packet->getMessage();

  // create the routing extension if needed
  if (packet->getRoutingExtension() == nullptr) {
    assert(packet->getHopCount() == 1);

    // create routing extension header
    std::vector<u32>* re = new std::vector<u32>(1 + routerAddress.size());
    re->at(0) = U32_MAX;  // dummy
    packet->setRoutingExtension(re);

    // random intermediate address
    for (u32 idx = 1; idx < re->size(); idx++) {
      re->at(idx) = gSim->rnd.nextU64(0, dimensionWidths_.at(idx - 1) - 1);
    }
  }

  // get a const pointer to the address (with leading dummy)
  const std::vector<u32>* intermediateAddress =
      reinterpret_cast<const std::vector<u32>*>(packet->getRoutingExtension());

  // ex: [c,x,y,z]
  const std::vector<u32>* destinationAddress = message->getDestinationAddress();
  assert(routerAddress.size() == (destinationAddress->size() - 1));
  assert(intermediateAddress->size() == destinationAddress->size());

  // determine which stage we are in based on VC set
  //  if this is a terminal port, force to stage 0
  u32 stage;

  if (packet->getHopCount() == 1) {
    stage = 0;
  } else {
    stage = ((_flit->getVc() % 2) == 0) ? 0 : 1;
  }


  // determine the next intermediate dimension to work on
  u32 iDim;
  u32 iPortBase = concentration_;
  for (iDim = 0; iDim < routerAddress.size(); iDim++) {
    if (routerAddress.at(iDim) != intermediateAddress->at(iDim+1)) {
      break;
    }
    iPortBase += ((dimensionWidths_.at(iDim) - 1) * dimensionWeights_.at(iDim));
  }


  // determine the next dimension to work on
  u32 dDim;
  u32 dPortBase = concentration_;
  for (dDim = 0; dDim < routerAddress.size(); dDim++) {
    if (routerAddress.at(dDim) != destinationAddress->at(dDim+1)) {
      break;
    }
    dPortBase += ((dimensionWidths_.at(dDim) - 1) * dimensionWeights_.at(dDim));
  }

  // determine what to work on (intermediate or destination node)
  u32 dim;
  u32 portBase;
  const std::vector<u32>* routingTo;

  // If a routing extension exists we are in intermediate stage
  if (stage == 0) {
    if (iDim == routerAddress.size()) {
      // done with intermediate, go to destination
      dim = dDim;
      portBase = dPortBase;
      stage = 1;
      routingTo = destinationAddress;
    } else {
      // still didn't reach intermediate node
      dim = iDim;
      portBase = iPortBase;
      routingTo = intermediateAddress;
    }

  } else {
    // working in destination stage
    dim = dDim;
    portBase = dPortBase;
    routingTo = destinationAddress;
  }


  // test if already at destination router
  if (dim == routerAddress.size()) {
    assert(stage == 1);

    bool res = outputPorts.insert(routingTo->at(0)).second;
    (void)res;
    assert(res);

    outputPort = routingTo->at(0);

    // Add all VCs
    for (u32 vc = 0; vc < numVcs_; vc++) {
      _response->add(outputPort, vc);
    }

    // delete the routing extension
    delete intermediateAddress;
    packet->setRoutingExtension(nullptr);

  } else {
    // more router-to-router hops needed
    u32 src = routerAddress.at(dim);
    u32 dst = routingTo->at(dim+1);

    assert(src != dst);

    if (dst < src) {
      dst += dimensionWidths_.at(dim);
    }
    u32 offset = (dst - src - 1) * dimensionWeights_.at(dim);
    // add all ports where the two routers are connecting
    for (u32 weight = 0; weight < dimensionWeights_.at(dim); weight++) {
      bool res = outputPorts.insert(portBase + offset + weight).second;
      (void)res;
      assert(res);
    }
  }

assert(outputPorts.size() > 0);
  for (auto it = outputPorts.cbegin(); it != outputPorts.cend(); ++it) {
    outputPort = *it;
    // select all VCs in the output port
    for (u32 vc = stage; vc < numVcs_; vc+=2) {
      _response->add(outputPort, vc);
    }
  }
}

}  // namespace HyperX
