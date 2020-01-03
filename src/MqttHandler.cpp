/*
 * MqttHandler.cpp
 *
 *  Created on: Nov 7, 2019
 *      Author: Fahad Usman
 */
#include "MqttHandler.h"

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "CommandMsg.h"
#include "StationEdgeDevice.h"

MqttCommunicator::MqttCommunicator(StationEdgeDevice * d) : communicator(d),
        willmsg(MQTT_DFLT_TOPIC, "MQTT_DFLT_LWT_PAYLOAD", MQTT_DFLT_QOS, true), client(
                MQTT_DFLT_SERVER_ADDRESS, MQTT_DFLT_CLIENT_ID), willOpts(
                willmsg) {

    client.set_callback(cb);
    cb.setCommunicator(this);
    conopts.set_will(willOpts);
    conntok = NULL;
    isConnected = false;
    publishTopic = MQTT_DFLT_TOPIC;
    clientID = MQTT_DFLT_CLIENT_ID;
    persistDir = MQTT_DFLT_PERSIST_DIR;
    QoS = MQTT_DFLT_QOS;
    cleanSession = MQTT_DFLT_CLEAN_SESSION;
    timeout = MQTT_DFLT_TIMEOUT;
    sendMessagesThreadPtr = new std::thread(&MqttCommunicator::sendQueuedMessagesThread, this);
    commandTopic = DFLT_MQTT_CMD_TOPIC;
}

void MqttCommunicator::connect() {
    LOG(INFO) << "connecting to broker";
    try {
        conntok = client.connect(conopts);
        conntok->wait();
        LOG(INFO) << "MQTT Client Connected: " << clientID;
        isConnected = true;
    } catch (const mqtt::exception& exc) {
        LOG(FATAL) << "MQTT Exception: " << exc.what();
        //TODO: Handle connection failure, reconnect etc.
        return;
    }
}

void MqttCommunicator::sendMessage(const char * message,
        const unsigned int length) {
    if (not isConnected) {
        LOG(FATAL) << "sendMessage: Mqtt client is not connected.";
        return;
    }
    LOG(INFO) << "Sending MQTT Message: " << message << "\ttopic: " << publishTopic
            << ", length: " << length;
    mqtt::delivery_token_ptr pubtok;
    pubtok = client.publish(publishTopic, message, length, QoS, cleanSession);
    pubtok->wait_for(timeout);
}

void MqttCommunicator::disconnect() {
    if (isConnected) {
        LOG(INFO) << "disconnecting: " << this->clientID;
        conntok = client.disconnect();
        conntok->wait();
        isConnected = false;
        LOG(INFO) << "successfully disconnected.: " << this->clientID;
    } else {
        LOG(WARNING) << "disconnect called for: " << clientID
                << ", isConnected is already false.";
    }
}

void MqttCommunicator::sendQueuedMessagesThread() {
    LOG(INFO) << "starting MqttCommunicator Thread";
    while (true){
        CommDataBuffer * commPtr = NULL;
        {
            std::lock_guard<std::mutex> guard(transmitQueueMutex);
            if (transmitQueue.size() > 0) {
                commPtr = transmitQueue.begin()->second;
                transmitQueue.erase(transmitQueue.begin());
            }
        }

        if (commPtr != NULL) {
            LOG(INFO) << "Sending Message with t: " << commPtr->getTimestamp();
            std::string jsonMessage = commPtr->serializeJson();
            sendMessage(jsonMessage.c_str(), jsonMessage.length());
            delete commPtr;
            commPtr = NULL;
        }
        std::this_thread::sleep_for(sendMessagesThreadLoopInterval);
    }
}

void MqttCommunicator::subscribe(){
    client.subscribe(commandTopic, QoS);
    LOG(INFO) << "Subscribing to: " << commandTopic;
}

void MqttCommunicator::processIncomingMessage(const char * msg, const int len) {
    try {
        rapidjson::Document doc;
        LOG(INFO) << "processIncomingMessage(" << msg << ", " << len << ")";
        if (doc.Parse(msg).HasParseError()) {
            LOG(ERROR) << "Invalid JSON message received: ";
        } else if (doc.HasMember(COMMAND_KEY) and doc[COMMAND_KEY].IsInt()
                and doc.HasMember(VALUE_KEY) and doc[VALUE_KEY].IsInt()) {
            CommandMsg * cmd = new CommandMsg(
                    static_cast<CommandRegister>(doc[COMMAND_KEY].GetInt()),
                    doc[VALUE_KEY].GetInt());
            edgeDevicePtr->processIncomingCommand(cmd);
        }
    } catch (const std::exception & exc) {
        LOG(ERROR) << "STD Exception: " << exc.what();
    }
}

void user_callback::message_arrived(mqtt::const_message_ptr msg) {
    try {
        std::cout << "topic: " << msg->get_topic() << "\tpayload: "
                << msg->to_string() << "\n";
        if (commPtr != NULL){
            commPtr->processIncomingMessage(msg->to_string().c_str(), msg->to_string().length());
        } else {
            LOG(ERROR) << "Message arrived, commPtr is NULL";
        }

    } catch (const mqtt::exception& exc) {
        LOG(ERROR) << "MQTT Exception: " << exc.what();
    } catch (const std::exception & exc) {
        LOG(ERROR) << "STD Exception: " << exc.what();
    }

}

MqttCommunicator::~MqttCommunicator() {
    if (isConnected) {
        disconnect();
    }
    return;
}

void user_callback::connected(const std::string& cause) {
    connected_ = true;
    LOG(INFO) << "MQTT client connected: " << cause;
    commPtr->subscribe();
}

void user_callback::setCommunicator(communicator* c) {
    commPtr = c;
}
