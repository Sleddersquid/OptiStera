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
    opcua::Node parentNode1(server, opcua::ObjectId::ObjectsFolder);
    opcua::Node parentNode2(server, opcua::ObjectId::ObjectsFolder);

    opcua::Node myIntegerNode1 = parentNode1.addVariable({1, 1000}, "Node 1");
    opcua::Node myIntegerNode2 = parentNode2.addVariable({1, 1001}, "Node 2");
    // Write value attribute
    myIntegerNode1.writeValueScalar(42);
    myIntegerNode2.writeValueScalar(50);

    server.run();

    std::this_thread::sleep_for(std::chrono::seconds(10));

    server.stop();

    std::cout << "End of file" << std::endl;
    return 0;
}
