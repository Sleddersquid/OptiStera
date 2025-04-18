#include <iostream>

#include <open62541pp/node.hpp>
#include <open62541pp/server.hpp>
// #include <open62541pp/plugin/accesscontrol_default.hpp>

// Sources
// https://open62541pp.github.io/open62541pp/server_datasource_8cpp-example.html

int main() {
    opcua::ServerConfig config;
    config.setApplicationName("open62541pp server example");
    config.setApplicationUri("urn:open62541pp.server.application");
    config.setProductUri("https://sleddersquid.io/open62541pp");

    opcua::Server server(std::move(config));

    // Add a variable node to the Objects node
    opcua::Node parentNode(server, opcua::ObjectId::ObjectsFolder);
    opcua::Node myIntegerNode = parentNode.addVariable(
        {1, "TheAnswer"},
        "The Answer",
        opcua::VariableAttributes{}
            .setDisplayName({"en-US", "The Answer"})
            .setDescription({"en-US", "Answer to the Ultimate Question of Life"})
            .setDataType(opcua::DataTypeId::Index)
    );

    // Write a value (attribute) to the node
    myIntegerNode.writeValueScalar(42);

    // Read the value (attribute) from the node
    std::cout << "The answer is: " << myIntegerNode.readValueScalar<int>() << std::endl;

    server.run();
}
