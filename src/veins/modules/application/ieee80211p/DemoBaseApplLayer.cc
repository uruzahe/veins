//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

using namespace veins;

// My code, Begin
// #include <boost/thread.hpp>
#include <pthread.h>
#include <time.h>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <vector>
// ----- Begin My Code -----
#include <nlohmann/json.hpp>

using json = nlohmann::json;

int existFile(const char* path)
{
    FILE* fp = fopen(path, "r");
    if (fp == NULL) {
        return 0;
    }

    fclose(fp);
    return 1;
}


int carla_lock_wait(std::string data_sync_dir) {
  std::string carla_lock_file_path = data_sync_dir + "carla.lock";

  if (existFile(carla_lock_file_path.c_str())) {
    return carla_lock_wait(data_sync_dir);
  } else {
    return 1;
  }
}


void lock(const char *oldpath, const char *newpath) {
  while (symlink(oldpath, newpath) == -1) {
    continue;
  }
}


void set_cpm_payloads_for_carla(std::string sumo_id, std::string data_sync_dir, std::vector<std::string> payloads) {
    std::string packet_data_file_name = sumo_id + "_packet.json";
    std::string packet_lock_file_name = sumo_id + "_packet.json.lock";

//    lock((data_sync_dir + packet_data_file_name).c_str(), (data_sync_dir + packet_lock_file_name).c_str());
    std::ofstream ofs(data_sync_dir + packet_data_file_name, std::ios::in | std::ios::ate);
    if (ofs.is_open()) {
        for (auto payload = payloads.begin(); payload != payloads.end(); payload++) {
            ofs << *payload << std::endl;
        }
    }
    ofs.close();
//    unlink((data_sync_dir + packet_lock_file_name).c_str());
}


std::vector<std::string> get_cpm_payloads_from_carla(std::string sumo_id, std::string data_sync_dir, bool read_only) {
    std::string sensor_data_file_name = sumo_id + "_sensor.json";
    std::string sensor_lock_file_name = sumo_id + "_sensor.json.lock";

    std::vector<std::string> payloads = {};
    std::string payload;

//    lock((data_sync_dir + sensor_data_file_name).c_str(), (data_sync_dir + sensor_lock_file_name).c_str());
    std::ifstream ifs(data_sync_dir + sensor_data_file_name);
    if (ifs.is_open()) {
        while (!ifs.eof()) {
          std::getline(ifs, payload);
          if (payload != "") {
            payloads.push_back(payload);
          } else {
            continue;
          }
        }
    }
    ifs.close();

    if (!read_only) {
      std::ofstream ofs(data_sync_dir + sensor_data_file_name);
      ofs.close();
    }

//    unlink((data_sync_dir + sensor_lock_file_name).c_str());
    return payloads;
}


// ----- End My Code -----

void DemoBaseApplLayer::initialize(int stage)
{
    BaseApplLayer::initialize(stage);

    if (stage == 0) {

        // initialize pointers to other modules
        if (FindModule<TraCIMobility*>::findSubModule(getParentModule())) {
            mobility = TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();
        }
        else {
            traci = nullptr;
            mobility = nullptr;
            traciVehicle = nullptr;
        }

        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        mac = FindModule<DemoBaseApplLayerToMac1609_4Interface*>::findSubModule(getParentModule());
        ASSERT(mac);

        // read parameters
        headerLength = par("headerLength");
        sendBeacons = par("sendBeacons").boolValue();
        beaconLengthBits = par("beaconLengthBits");
        beaconUserPriority = par("beaconUserPriority");
        beaconInterval = par("beaconInterval");

        dataLengthBits = par("dataLengthBits");
        dataOnSch = par("dataOnSch").boolValue();
        dataUserPriority = par("dataUserPriority");

        wsaInterval = par("wsaInterval").doubleValue();
        currentOfferedServiceId = -1;

        isParked = false;

        findHost()->subscribe(BaseMobility::mobilityStateChangedSignal, this);
        findHost()->subscribe(TraCIMobility::parkingStateChangedSignal, this);

        sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
        sendWSAEvt = new cMessage("wsa evt", SEND_WSA_EVT);

        generatedBSMs = 0;
        generatedWSAs = 0;
        generatedWSMs = 0;
        receivedBSMs = 0;
        receivedWSAs = 0;
        receivedWSMs = 0;

        // My Code, Begin.
        EV_TRACE << "My Code" << std::endl;
        carlaVeinsDataDir = par("carlaVeinsDataDir").stringValue();
        sendCPM = par("sendCPM").boolValue();
        is_dynamic_simulation = par("is_dynamic_simulation").boolValue();
        carlaTimeStep = par("carlaTimeStep").doubleValue();

        sendCPMEvt = new cMessage("cpm evt", SEND_CPM_EVT);
        sumo_id = mobility->getExternalId();
        obtainedCPMs = {};
        if (is_dynamic_simulation) {
          reservedCPMs = {} ;
        } else {
          reservedCPMs = get_cpm_payloads_from_carla(sumo_id, carlaVeinsDataDir, true);
        }
        veinsLockFile = sumo_id + "_veins.lock";
        veinsTxtFile = sumo_id + "_veins.txt";

        generatedCPMs = 0;
        receivedCPMs = 0;
        // My Code, End.
    }
    else if (stage == 1) {

        // store MAC address for quick access
        myId = mac->getMACAddress();

        // simulate asynchronous channel access

        if (dataOnSch == true && !mac->isChannelSwitchingActive()) {
            dataOnSch = false;
            EV_ERROR << "App wants to send data on SCH but MAC doesn't use any SCH. Sending all data on CCH" << std::endl;
        }
        simtime_t firstBeacon = simTime();

        if (par("avoidBeaconSynchronization").boolValue() == true) {

            simtime_t randomOffset = dblrand() * beaconInterval;
            firstBeacon = simTime() + randomOffset;

            if (mac->isChannelSwitchingActive() == true) {
                if (beaconInterval.raw() % (mac->getSwitchingInterval().raw() * 2)) {
                    EV_ERROR << "The beacon interval (" << beaconInterval << ") is smaller than or not a multiple of  one synchronization interval (" << 2 * mac->getSwitchingInterval() << "). This means that beacons are generated during SCH intervals" << std::endl;
                }
                firstBeacon = computeAsynchronousSendingTime(beaconInterval, ChannelType::control);
            }

            if (sendBeacons) {
                scheduleAt(firstBeacon, sendBeaconEvt);
            }

            // My Code, Begin.
            if (sendCPM) {
              scheduleAt(firstBeacon, sendCPMEvt);
            }
            // My Code, End.
        }
    }
}

simtime_t DemoBaseApplLayer::computeAsynchronousSendingTime(simtime_t interval, ChannelType chan)
{

    /*
     * avoid that periodic messages for one channel type are scheduled in the other channel interval
     * when alternate access is enabled in the MAC
     */

    simtime_t randomOffset = dblrand() * beaconInterval;
    simtime_t firstEvent;
    simtime_t switchingInterval = mac->getSwitchingInterval(); // usually 0.050s
    simtime_t nextCCH;

    /*
     * start event earliest in next CCH (or SCH) interval. For alignment, first find the next CCH interval
     * To find out next CCH, go back to start of current interval and add two or one intervals
     * depending on type of current interval
     */

    if (mac->isCurrentChannelCCH()) {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval * 2;
    }
    else {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval;
    }

    firstEvent = nextCCH + randomOffset;

    // check if firstEvent lies within the correct interval and, if not, move to previous interval

    if (firstEvent.raw() % (2 * switchingInterval.raw()) > switchingInterval.raw()) {
        // firstEvent is within a sch interval
        if (chan == ChannelType::control) firstEvent -= switchingInterval;
    }
    else {
        // firstEvent is within a cch interval, so adjust for SCH messages
        if (chan == ChannelType::service) firstEvent += switchingInterval;
    }

    return firstEvent;
}

void DemoBaseApplLayer::populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId, int serial)
{
    wsm->setRecipientAddress(rcvId);
    wsm->setBitLength(headerLength);

    if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        bsm->setSenderPos(curPosition);
        bsm->setSenderSpeed(curSpeed);
        bsm->setPsid(-1);
        bsm->setChannelNumber(static_cast<int>(Channel::cch));
        bsm->addBitLength(beaconLengthBits);
        wsm->setUserPriority(beaconUserPriority);
    }
    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        wsa->setChannelNumber(static_cast<int>(Channel::cch));
        wsa->setTargetChannel(static_cast<int>(currentServiceChannel));
        wsa->setPsid(currentOfferedServiceId);
        wsa->setServiceDescription(currentServiceDescription.c_str());
    }
    else {
        if (dataOnSch)
            wsm->setChannelNumber(static_cast<int>(Channel::sch1)); // will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        else
            wsm->setChannelNumber(static_cast<int>(Channel::cch));
        wsm->addBitLength(dataLengthBits);
        wsm->setUserPriority(dataUserPriority);
    }
}

void DemoBaseApplLayer::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
    Enter_Method_Silent();
    if (signalID == BaseMobility::mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == TraCIMobility::parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}

void DemoBaseApplLayer::handlePositionUpdate(cObject* obj)
{
    ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
    curPosition = mobility->getPositionAt(simTime());
    curSpeed = mobility->getCurrentSpeed();
}

void DemoBaseApplLayer::handleParkingUpdate(cObject* obj)
{
    isParked = mobility->getParkingState();
}

void DemoBaseApplLayer::handleLowerMsg(cMessage* msg)
{
    BaseFrame1609_4* wsm = dynamic_cast<BaseFrame1609_4*>(msg);
    ASSERT(wsm);

    if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        receivedBSMs++;
        onBSM(bsm);
    }
    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        receivedWSAs++;
        onWSA(wsa);
    } // My Code, Begin
    else if (VeinsCarlaCpm* cpm = dynamic_cast<VeinsCarlaCpm*>(wsm)) {
//        std::cout << sumo_id << " received cpm messages" << std::endl;
//        std::cout << "payloads: " << cpm->getPayload() << std::endl;
        receivedCPMs++;
        obtainedCPMs.push_back((std::string) cpm->getPayload());
    } // My Code, End.
    else {
        receivedWSMs++;
        onWSM(wsm);
    }

    delete (msg);
}

// My Code, Begin
void DemoBaseApplLayer::syncCarlaVeinsData(cMessage* msg)
{
    std::vector<std::string> targetCPMs;
    double next_time_step = carlaTimeStep;

    if (is_dynamic_simulation) {
      // save received cpms
      set_cpm_payloads_for_carla(sumo_id, carlaVeinsDataDir, obtainedCPMs);
      obtainedCPMs.clear();
      obtainedCPMs.shrink_to_fit();

      // send CPMs
      std::vector<std::string> new_payloads = get_cpm_payloads_from_carla(sumo_id, carlaVeinsDataDir, false);

      for (auto payload = new_payloads.begin(); payload != new_payloads.end(); payload++) {
        targetCPMs.push_back(*payload);
      }

    } else {
      auto payload = reservedCPMs.begin();

      while (payload != reservedCPMs.end()) {
        json payload_json = json::parse(*payload);
        double timestamp = payload_json["timestamp"].get<double>();
        double simtime = simTime().dbl();
        // std::cout << "sumo_id" << sumo_id << "simTime: " << simtime << " timestamp: " << timestamp << std::endl;

        if (timestamp <= simtime - carlaTimeStep) {
          // std::cout << "The packet is too old, so erase it." << std::endl;
          reservedCPMs.erase(payload);
        } else if (simtime - carlaTimeStep < timestamp && timestamp <= simtime) {
          // std::cout << "The packet is created now, so send it." << std::endl;
          targetCPMs.push_back(*payload);
          reservedCPMs.erase(payload);
        } else {
          // std::cout << "The packet should be sent in the next timestamp, so break" << std::endl;
          break;
        }
      }
    }

    for (auto payload = targetCPMs.begin(); payload != targetCPMs.end(); payload++) {
      json payload_json = json::parse(*payload);

      VeinsCarlaCpm* cpm = new VeinsCarlaCpm();
      populateWSM(cpm);

      cpm->setPayload((*payload).c_str());
      cpm->setBitLength(payload_json["option"]["size"].get<int>() * 8);
      sendDown(cpm);
    }
}

// My Code, End.
void DemoBaseApplLayer::handleSelfMsg(cMessage* msg)
{
    switch (msg->getKind()) {
    case SEND_BEACON_EVT: {
        DemoSafetyMessage* bsm = new DemoSafetyMessage();
        populateWSM(bsm);
        sendDown(bsm);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        break;
    }
    case SEND_WSA_EVT: {
        DemoServiceAdvertisment* wsa = new DemoServiceAdvertisment();
        populateWSM(wsa);
        sendDown(wsa);
        scheduleAt(simTime() + wsaInterval, sendWSAEvt);
        break;
    }
    case SEND_CPM_EVT: {
        // My Code, Begin
        // std::chrono::system_clock::time_point  start, end; // 型は auto で可
        // start = std::chrono::system_clock::now(); // 計測開始時間

        syncCarlaVeinsData(msg);

        // end = std::chrono::system_clock::now();  // 計測終了時間
        // double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count();
        // std::cout << "diff_time: " << elapsed * (1.0 / (1000 * 1000 * 1000)) << std::endl;

        scheduleAt(simTime() + carlaTimeStep, sendCPMEvt);
        break;
        // My Code, Begin
    }
    default: {
        if (msg) EV_WARN << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
        break;
    }
    }
}

void DemoBaseApplLayer::finish()
{
    recordScalar("generatedWSMs", generatedWSMs);
    recordScalar("receivedWSMs", receivedWSMs);

    recordScalar("generatedBSMs", generatedBSMs);
    recordScalar("receivedBSMs", receivedBSMs);

    recordScalar("generatedWSAs", generatedWSAs);
    recordScalar("receivedWSAs", receivedWSAs);

    // Begin My Code.
    recordScalar("generatedCPMs", generatedCPMs);
    recordScalar("receivedCPMs", receivedCPMs);
    // End My Code.
}

DemoBaseApplLayer::~DemoBaseApplLayer()
{
    // My Code, Begin.
    cancelAndDelete(sendCPMEvt);
    // My Code, End.
    cancelAndDelete(sendBeaconEvt);
    cancelAndDelete(sendWSAEvt);
    findHost()->unsubscribe(BaseMobility::mobilityStateChangedSignal, this);
}

void DemoBaseApplLayer::startService(Channel channel, int serviceId, std::string serviceDescription)
{
    if (sendWSAEvt->isScheduled()) {
        throw cRuntimeError("Starting service although another service was already started");
    }

    mac->changeServiceChannel(channel);
    currentOfferedServiceId = serviceId;
    currentServiceChannel = channel;
    currentServiceDescription = serviceDescription;

    simtime_t wsaTime = computeAsynchronousSendingTime(wsaInterval, ChannelType::control);
    scheduleAt(wsaTime, sendWSAEvt);
}

void DemoBaseApplLayer::stopService()
{
    cancelEvent(sendWSAEvt);
    currentOfferedServiceId = -1;
}

void DemoBaseApplLayer::sendDown(cMessage* msg)
{
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDown(msg);
}

void DemoBaseApplLayer::sendDelayedDown(cMessage* msg, simtime_t delay)
{
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDelayedDown(msg, delay);
}

void DemoBaseApplLayer::checkAndTrackPacket(cMessage* msg)
{
    if (dynamic_cast<DemoSafetyMessage*>(msg)) {
        EV_TRACE << "sending down a BSM" << std::endl;
        generatedBSMs++;
    }
    else if (dynamic_cast<DemoServiceAdvertisment*>(msg)) {
        EV_TRACE << "sending down a WSA" << std::endl;
        generatedWSAs++;
    } // Begin My Code.
    else if (dynamic_cast<VeinsCarlaCpm*>(msg)) {
        EV_TRACE << "sending down a cpm" << std::endl;
        generatedCPMs++;
    } // End My Code.
    else if (dynamic_cast<BaseFrame1609_4*>(msg)) {
        EV_TRACE << "sending down a wsm" << std::endl;
        generatedWSMs++;
    }
}
