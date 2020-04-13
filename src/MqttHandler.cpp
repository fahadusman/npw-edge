/*
 * MqttHandler.cpp
 *
 *  Created on: Nov 7, 2019
 *      Author: Fahad Usman
 */
#include <EdgeDevice.h>
#include "MqttHandler.h"

#include "chrono"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "CommandMsg.h"

MqttCommunicator::MqttCommunicator(EdgeDevice * d) : communicator(d, false),
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
    while (not isConnected) {
        LOG(INFO) << "connecting to broker";
        try {
            conntok = client.connect(conopts);
            conntok->wait();
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
        while (not isConnected) {
            LOG(WARNING) << "MQTT Client not connected...";
            std::this_thread::sleep_for(std::chrono::seconds(1));
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
                LOG(INFO) << "Sending Message with t: "
                        << commPtr->getTimestamp();
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
    client.subscribe(commandTopic, QoS);
    LOG(INFO) << "Subscribing to: " << commandTopic;
}

bool MqttCommunicator::processIncomingMessage(const char * msg, const int & len) {
    try {
        rapidjson::Document doc;
        LOG(INFO) << "processIncomingMessage(" << msg << ", " << len << ")";
        if (doc.Parse(msg).HasParseError()) {
            LOG(ERROR) << "Invalid JSON message received: ";
            return false;
        } else if (doc.HasMember(COMMAND_KEY) and doc[COMMAND_KEY].IsInt()
                and doc.HasMember(VALUE_KEY) and doc[VALUE_KEY].IsInt()) {
            CommandRegister command = static_cast<CommandRegister>(doc[COMMAND_KEY].GetInt());
            int32_t value = doc[VALUE_KEY].GetInt();
            uint8_t deviceId = 0;
            if (doc.HasMember(DEVICEID_KEY) and doc[DEVICEID_KEY].IsInt()) {
                deviceId = doc[DEVICEID_KEY].GetInt();
            }

            switch (command) {
            case NPW_NUM_PACK:
                setNpwPacketsToBuffer(value);
                break;
            break;
            case ACK_NPW_BUFF:
                removeMessageFromQueue(value);
                break;
            default:
                LOG(INFO) << "Passing incoming command to edge device";
                CommandMsg * cmd = new CommandMsg(command, value, deviceId);
                edgeDevicePtr->processIncomingCommand(cmd);
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
}

void user_callback::setCommunicator(communicator* c) {
    commPtr = c;
}
