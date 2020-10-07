/*
 * MqttHandler.cpp
 *
 *  Created on: Nov 7, 2019
 *      Author: Fahad Usman
 */
#include <EdgeDevice.h>
#include "MqttHandler.h"

#include <chrono>
#include <sstream>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "CommandMsg.h"

MqttCommunicator::MqttCommunicator(EdgeDevice *d,
        rapidjson::Value &communicatorObj) :
        communicator(d, false), willmsg(MQTT_DFLT_TOPIC,
                "MQTT_DFLT_LWT_PAYLOAD", MQTT_DFLT_QOS, true),
                willOpts(willmsg) {

    cb.setCommunicator(this);
    conopts.set_will(willOpts);
    serverAddress = MQTT_DFLT_SERVER_ADDRESS;
    conntok = NULL;
    isConnected = false;
    publishTopic = MQTT_DFLT_TOPIC;
    clientID = MQTT_DFLT_CLIENT_ID;
    persistDir = MQTT_DFLT_PERSIST_DIR;
    QoS = MQTT_DFLT_QOS;
    cleanSession = MQTT_DFLT_CLEAN_SESSION;
    timeout = MQTT_DFLT_TIMEOUT;
    commandTopic = DFLT_MQTT_CMD_TOPIC;

    try {
        serverAddress = communicatorObj["server"].GetString();
        publishTopic = communicatorObj["publish_topic"].GetString();
        clientID = communicatorObj["client_id"].GetString();
        QoS = communicatorObj["qos"].GetInt();
        commandTopic = communicatorObj["command_topic"].GetString();


    } catch (const std::exception &e) {
        LOG(FATAL) << "Got exception while parsing config,json file: "
                << e.what();
    }
    asyncClientPtr = new mqtt::async_client(serverAddress, clientID);
    asyncClientPtr->set_callback(cb);
    sendMessagesThreadPtr = new std::thread(&MqttCommunicator::sendQueuedMessagesThread, this);
}

void MqttCommunicator::connect() {
    while (not isConnected) {
        LOG(INFO) << "connecting to broker";
        try {
            asyncClientPtr->connect(conopts)->wait();
            LOG(INFO) << "MQTT Client Connected: " << clientID;
            isConnected = true;
        } catch (const mqtt::exception& exc) {
            LOG(ERROR) << "MQTT Exception: " << exc.what();
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

void MqttCommunicator::sendMessage(const char * message,
        const unsigned int length) {
    if (not isConnected) {
        LOG(ERROR) << "sendMessage: Mqtt client is not connected.";
        return;
    }
    LOG(INFO) << "Sending MQTT Message: " << message << "\ttopic: " << publishTopic
            << ", length: " << length;
    try {
        asyncClientPtr->publish(publishTopic, message, length, QoS,
                cleanSession)->wait_for(timeout);
    } catch (const mqtt::exception &exc) {
        LOG(ERROR) << "MQTT Exception: " << exc.what();
    }
}

void MqttCommunicator::disconnect() {
    if (isConnected) {
        LOG(INFO) << "disconnecting: " << this->clientID;
        try {
            asyncClientPtr->disconnect()->wait();
        } catch (const mqtt::exception &exc) {
            LOG(ERROR) << "MQTT Exception: " << exc.what();
        }
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
        while (not isConnected) {
            LOG(WARNING) << "MQTT Client not connected...";
            connect();
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
        CommDataBuffer * commPtr = getQueuedMessage();

        if (commPtr != nullptr) {
            uint64_t currentTime =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count();

            if (0 != commPtr->getExpiryTime()
                    and currentTime > commPtr->getExpiryTime()) {
                LOG(WARNING) << "Discarding expired message, id: "
                        << commPtr->getBufferId()
                        << "\tt: " << commPtr->getTimestamp()
                        << "\tExp time: " << commPtr->getExpiryTime();
            } else {
                std::string jsonMessage = commPtr->serializeJson();
                sendMessage(jsonMessage.c_str(), jsonMessage.length());
            }
            removeMessageFromQueue(commPtr->getBufferId());
            commPtr = nullptr;
        }
        std::this_thread::sleep_for(sendMessagesThreadLoopInterval);
    }
}

void MqttCommunicator::subscribe(){
    asyncClientPtr->subscribe(commandTopic, QoS);
    LOG(INFO) << "Subscribing to: " << commandTopic;
}

bool MqttCommunicator::processIncomingMessage(const char * msg, const int & len) {
    try {
        rapidjson::Document doc;
        LOG(INFO) << "processIncomingMessage(" << msg << ", " << len << ")";
        if (doc.Parse(msg).HasParseError()) {
            LOG(ERROR) << "Invalid JSON message received: ";
            return false;
        }

//        {"timestamp":1579516307016,"values":[{"id":"PSI.Device1.MAX_TIME_PERIODIC","v":10,"q":true,"t":1579516306266}]}
        if (doc.HasMember("values") and doc["values"].IsArray()) {
            for (auto &cmdDoc : doc["values"].GetArray()) {
                if (cmdDoc.HasMember("id") and cmdDoc["id"].IsString()
                        and cmdDoc.HasMember("v") and cmdDoc["v"].IsInt()) {
                    LOG(INFO) << "id: " << cmdDoc["id"].GetString()
                            << "\tv: " << cmdDoc["v"].GetInt();
                    std::istringstream idStream(cmdDoc["id"].GetString());
                    std::string ch, dev, tag;
                    std::getline(idStream, ch, '.');
                    std::getline(idStream, dev, '.');
                    std::getline(idStream, tag, '.');

                    LOG(INFO) << "ch: " << ch << " dev: " << dev << " tag: " << tag;

                    edgeDevicePtr->processIncomingCommand(tag,
                            (uint32_t) cmdDoc["v"].GetInt());
                } else {
                    LOG(WARNING)
                            << "Message received on command topic is not properly formed.";
                }

            }

        } else {
            LOG(WARNING) << "Message received on command topic is not properly formed.";
        }

    } catch (const std::exception & exc) {
        LOG(ERROR) << "STD Exception: " << exc.what();
        return false;
    }
    return true;
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
    LOG(INFO) << "MQTT subscribe done";
}

void user_callback::setCommunicator(communicator* c) {
    commPtr = c;
}
