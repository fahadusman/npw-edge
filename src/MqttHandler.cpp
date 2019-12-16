/*
 * MqttHandler.cpp
 *
 *  Created on: Nov 7, 2019
 *      Author: Fahad Usman
 */
#include "MqttHandler.h"
MqttCommunicator::MqttCommunicator() :
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
        return;
    }
}

void MqttCommunicator::sendMessage(const char * message,
        const unsigned int length) {
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
