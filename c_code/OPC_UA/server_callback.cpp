#include <iostream>

#include <open62541pp/node.hpp>
#include <open62541pp/server.hpp>

// Sources: 
// https://open62541pp.github.io/open62541pp/server_valuecallback_8cpp-example.html


int noe = 1;

// Variable value callback to write current time before every read operation
class CurrentTimeCallback : public opcua::ValueCallbackBase {
    void onRead(
        opcua::Session& session,
        const opcua::NodeId& id,
        [[maybe_unused]] const opcua::NumericRange* range,
        const opcua::DataValue& value
    ) override {
        opcua::Node node(session.connection(), id);
        const auto valueOld = value.value().scalar<int>();
        const auto valueNow = noe++;
        std::cout << "Old value: " << valueOld << std::endl;
        std::cout << "New value: " << valueNow << std::endl;
        node.writeValueScalar(valueNow);
    }

    void onWrite(
        [[maybe_unused]] opcua::Session& session,
        [[maybe_unused]] const opcua::NodeId& id,
        [[maybe_unused]] const opcua::NumericRange* range,
        [[maybe_unused]] const opcua::DataValue& value
    ) override {}
};

int main() {
    opcua::Server server;

    const opcua::NodeId currentTimeId(1, 1000);

    // Initialize the node with a value
    opcua::Node(server, opcua::ObjectId::ObjectsFolder)
        .addVariable(currentTimeId, "Value")
        .writeDisplayName({"en-US", "Current value"})
        .writeDescription({"en-US", "Current value. See value"})
        .writeDataType<int>()
        .writeValueScalar(29543687);

    CurrentTimeCallback currentTimeCallback;
    server.setVariableNodeValueCallback(currentTimeId, currentTimeCallback);

    server.run();
}