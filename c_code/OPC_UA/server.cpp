#include <open62541pp/open62541pp.hpp>

#include <open62541pp/server.hpp>
#include <open62541pp/node.hpp>

#include <iostream>
#include <thread>
#include <chrono>


// Soruces:
// https://github.com/open62541pp/open62541pp/tree/master


int main() {
    opcua::ServerConfig config;
    config.setApplicationName("open62541pp minimal server");
    config.setApplicationUri("urn:open62541pp.minimal.server");
    config.setProductUri("http://open62541.org");

    opcua::Server server(std::move(config));

    // Add a variable node to the Objects node
    opcua::Node parentNode(server, opcua::ObjectId::ObjectsFolder);

    opcua::Node myIntegerNode = parentNode.addVariable({1, 1000}, "Node 1");
    // Write value attribute
    myIntegerNode.writeValueScalar(4);

    server.run();

    std::cout << "End of file" << std::endl;
    return 0;
}
