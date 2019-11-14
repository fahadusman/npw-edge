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
#include "communicator.h"

const std::string MQTT_DFLT_SERVER_ADDRESS { "tcp://localhost:1883" };
const std::string MQTT_DFLT_CLIENT_ID { "NPW_APP" };
const std::string MQTT_DFLT_PERSIST_DIR { "./persist" };
//const char* MQTT_DFLT_LWT_PAYLOAD = "Last will and testament.";

const std::string MQTT_DFLT_TOPIC { "hello" };
const int MQTT_DFLT_QOS = 1;
const bool MQTT_DFLT_CLEAN_SESSION = false;
const auto MQTT_DFLT_TIMEOUT = std::chrono::seconds(10);

/////////////////////////////////////////////////////////////////////////////
// Class to receive callbacks

class user_callback: public virtual mqtt::callback {
    bool connected_;

    void connection_lost(const std::string& cause) override {
        connected_ = false;
        std::cout << "\nConnection lost" << std::endl;
        if (!cause.empty())
            std::cout << "\tcause: " << cause << std::endl;
    }

    void delivery_complete(mqtt::delivery_token_ptr tok) override {
        std::cout << "\n\t[Delivery complete for token: "
                << (tok ? tok->get_message_id() : -1) << "]" << std::endl;
    }
    void connected(const std::string& cause) override {
        connected_ = true;
        LOG(INFO) << "MQTT client connected: " << cause;
    }

//	void message_arrived(const message * msg) override {
//	}

    bool isConnected() {
        return connected_;
    }

public:
    user_callback() {
        connected_ = false;
    }
};

class MqttCommunicator: public communicator {
private:
    std::string address, clientID, topic, persistDir;

    mqtt::message willmsg;
    mqtt::async_client client; //(address, clientID);
    user_callback cb;
    mqtt::connect_options conopts;
    mqtt::will_options willOpts;
    mqtt::token_ptr conntok;
    bool isConnected;
    bool cleanSession;
    unsigned int QoS;
    std::chrono::duration<int64_t> timeout;
public:
    MqttCommunicator();
    void connect();
    void disconnect();
    void sendMessage(const char * message, const unsigned int length);
    ~MqttCommunicator();

};

#endif /* INCLUDE_MQTTHANDLER_H_ */
