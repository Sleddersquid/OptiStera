#include <iostream>

#include <open62541pp/node.hpp>
#include <open62541pp/server.hpp>

#include <signal.h>

#include <thread>
#include <chrono>

// Sources:
// https://open62541pp.github.io/open62541pp/server_valuecallback_8cpp-example.html

int noe = 1;

// Variable value callback to write current time before every read operation
class CurrentTimeCallback : public opcua::ValueCallbackBase {
    void onRead(
        opcua::Session &session,
        const opcua::NodeId &id,
        [[maybe_unused]] const opcua::NumericRange *range,
        const opcua::DataValue &value) override
    {
        opcua::Node node(session.connection(), id);
        const auto valueOld = value.value().scalar<int>();
        const auto valueNow = noe++;
        std::cout << "Value: " << valueNow << std::endl;
        node.writeValueScalar(valueNow);
    }

    void onWrite(
        [[maybe_unused]] opcua::Session &session,
        [[maybe_unused]] const opcua::NodeId &id,
        [[maybe_unused]] const opcua::NumericRange *range,
        [[maybe_unused]] const opcua::DataValue &value) override {}
};

void signalHandler(int signum);

opcua::Server server_1;

int main() {

    uint16_t sleep_time;
    const opcua::NodeId currentTimeId(1, 1000);
    int i;

    // Initialize the node with a value
    opcua::Node(server_1, opcua::ObjectId::ObjectsFolder)
        .addVariable(currentTimeId, "Time")
        .writeDisplayName({"en-US", "Current value"})
        .writeDescription({"en-US", "Current value. See value"})
        .writeDataType<int>()
        .writeValueScalar(i++);

    CurrentTimeCallback currentTimeCallback;
    server_1.setVariableNodeValueCallback(currentTimeId, currentTimeCallback);

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    while(true) {
        sleep_time = server_1.runIterate();
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
    }
}

void signalHandler(int signum) {
    server_1.stop();
    exit(signum);
}