/*
 * MqttHandler.h
 *
 *  Created on: Nov 5, 2019
 *      Author: Fahad Usman
 */

#ifndef INCLUDE_MQTTHANDLER_H_
#define INCLUDE_MQTTHANDLER_H_

#include <iostream>
#include <cstdlib>
#include <string>
#include <map>
#include <vector>
#include <cstring>
#include "mqtt/client.h"
#include "mqtt/message.h"
#include <glog/logging.h>
#include "rapidjson/document.h"

#include "communicator.h"
#include "DelayFlag.h"

const std::string MQTT_DFLT_SERVER_ADDRESS { "tcp://localhost:1883" };
const std::string MQTT_DFLT_CLIENT_ID { "NPW_APP" };
const std::string MQTT_DFLT_PERSIST_DIR { "./persist" };
//const char* MQTT_DFLT_LWT_PAYLOAD = "Last will and testament.";

const std::string MQTT_DFLT_TOPIC { "array" };
const int MQTT_DFLT_QOS = 1;
const bool MQTT_DFLT_CLEAN_SESSION = false;
const auto MQTT_DFLT_TIMEOUT = std::chrono::seconds(10);
const std::string DFLT_MQTT_CMD_TOPIC("command_topic");


/////////////////////////////////////////////////////////////////////////////
// Class to receive callbacks

class user_callback: public virtual mqtt::callback {
    bool connected_;
    communicator * commPtr;

    void connection_lost(const std::string& cause) override;
    void delivery_complete(mqtt::delivery_token_ptr tok) override;
    void connected(const std::string& cause) override;
	void message_arrived(mqtt::const_message_ptr msg) override;

public:
    void clearConnectedFlag() {
        connected_ = false;
    }

    bool isConnected() {
        return connected_;
    }
    user_callback() {
        connected_ = false;
        commPtr = NULL;
    }
    void setCommunicator(communicator * c);
};

class MqttCommunicator: public communicator {
protected:
    std::string address, clientID, publishTopic, persistDir, commandTopic, serverAddress;

    mqtt::message willmsg;
    mqtt::async_client * asyncClientPtr;
    user_callback cb;
    mqtt::connect_options conopts;
    mqtt::will_options willOpts;
    mqtt::token_ptr conntok;
    bool isConnected() {
        return cb.isConnected();
    }
    bool cleanSession;
    unsigned int QoS;
    std::chrono::duration<int64_t> timeout;
    std::queue<DelayFlag> delayFlagQueue;
public:
    MqttCommunicator(EdgeDevice * e, rapidjson::Value &communicatorObj);
    void connect() override;
    void disconnect() override;
    void sendMessage(const char * message, const unsigned int length) override;
    bool processIncomingMessage(const char * message, const int & length) override;

    virtual void sendQueuedMessagesThread() override;
    ~MqttCommunicator();
    void subscribe() override;
};

#endif /* INCLUDE_MQTTHANDLER_H_ */
