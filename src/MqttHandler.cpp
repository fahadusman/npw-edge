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
    conopts.set_will(willOpts);
    conntok = NULL;
    isConnected = false;
    topic = MQTT_DFLT_TOPIC;
    clientID = MQTT_DFLT_CLIENT_ID;
    persistDir = MQTT_DFLT_PERSIST_DIR;
    QoS = MQTT_DFLT_QOS;
    cleanSession = MQTT_DFLT_CLEAN_SESSION;
    timeout = MQTT_DFLT_TIMEOUT;

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
    LOG(INFO) << "Sending MQTT Message: " << message << "\ttopic: " << topic
            << ", length: " << length;
    mqtt::delivery_token_ptr pubtok;
    pubtok = client.publish(topic, message, length, QoS, cleanSession);
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

MqttCommunicator::~MqttCommunicator() {
    if (isConnected) {
        disconnect();
    }
    return;
}
