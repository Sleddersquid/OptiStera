#include <open62541pp/open62541pp.hpp>

#include <open62541pp/server.hpp>
#include <open62541pp/node.hpp>

#include <iostream>
#include <thread>
#include <chrono>

// Soruces:
// https://open62541pp.github.io/open62541pp/server_8cpp-example.html

uint32_t delay_server_iterate = 0;
int hehehehehehe = 0;

int main() {
    opcua::ServerConfig config;
    config.setApplicationName("open62541pp server opencv");
    config.setApplicationUri("urn:open62541pp.server.application");
    config.setProductUri("https://open62541pp.github.io");
    
 
    opcua::Server server(std::move(config));
 
    // Add a variable node to the Objects node
    opcua::Node parentNode(server, opcua::ObjectId::ObjectsFolder);
    opcua::Node myIntegerNode = parentNode.addVariable(
        {1, 1000},
        "Variable",
        opcua::VariableAttributes{}
            .setDisplayName({"en-US", "Variable"})
            .setDescription({"en-US", "A Variable"})
            .setDataType<int>()
    );

    opcua::Node myIntegerNode1 = parentNode.addVariable(
        {1, 1001},
        "Variable2",
        opcua::VariableAttributes{}
            .setDisplayName({"en-US", "Variable"})
            .setDescription({"en-US", "A Variable"})
            .setDataType<int>()
    );
 
    // Write a value (attribute) to the node
    while (true) {
        myIntegerNode.writeValueScalar(hehehehehehe++);
        delay_server_iterate = server.runIterate();
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_server_iterate));
    }
}