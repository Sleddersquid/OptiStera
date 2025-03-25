#include <open62541pp/open62541pp.hpp>

#include <open62541pp/server.hpp>
#include <open62541pp/node.hpp>

#include <iostream>
#include <thread>
#include <chrono>

// Soruces:
// https://open62541pp.github.io/open62541pp/server_8cpp-example.html
// 

uint32_t delay_server_iterate = 0;

int main() {
    opcua::ServerConfig config;
    config.setApplicationName("open62541pp server objectRecgonition");
    config.setApplicationUri("urn:open62541pp.server.application");
    config.setProductUri("https://open62541pp.github.io");
    
    // node ids -> {namespace, id}
    opcua::NodeId parentNodeId = {1, 1000};
    opcua::NodeId cameraNodeXId = {1, 1001};
    opcua::NodeId cameraNodeYId = {1, 1002};
    opcua::NodeId cameraNodeRadiusId = {1, 1003};

    opcua::Server server(std::move(config));
 
    // Add a variable node to the Objects node
    // This node is the node connected to the objects folder, and is a parent node for all different nodes
    // https://open62541pp.github.io/open62541pp/group__AddNodes.html#ga1209377c80ebe7fd6e79146a7ecd8866
    // https://open62541pp.github.io/open62541pp/classopcua_1_1ua_1_1ObjectAttributes.html
    // https://open62541pp.github.io/open62541pp/classopcua_1_1LocalizedText.html
    // https://open62541pp.github.io/open62541pp/classopcua_1_1ua_1_1ReferenceTypeId.html
    opcua::Result<opcua::NodeId> parentNode = 
        opcua::services::addFolder(
            server,
            opcua::ObjectId::ObjectsFolder,
            parentNodeId,
            "Object Values",
            opcua::ObjectAttributes{}
                .setDisplayName({"en-US", "Camera Values"}),
            opcua::ReferenceTypeId::Organizes
        );
        
    opcua::Result<opcua::NodeId> Node_Camera_X =
        opcua::services::addVariable(
            server, // The server on which this little shitling is to be mounted on
            parentNodeId, // Parent folder
            cameraNodeXId, // id for this node, {namespace, id}
            "Position X", // Name for this node
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE) // READ and WRITE access
                .setDataType<int>(), // Type of data, can be int 16, 32, 64 and etc.
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent
        ); // Possible to add a default variable to the node, before any write operation is done to the node


    opcua::Result<opcua::NodeId> Node_Camera_Y =
        opcua::services::addVariable(
            server, // The server on which this little shitling is to be mounted on
            parentNodeId, // Parent folder
            cameraNodeYId, // id for this node, {namespace, id}
            "Position Y", // Name for this node
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE) // READ and WRITE access
                .setDataType<int>(), // Type of data, can be int 16, 32, 64 and etc.
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent
        ); // Possible to add a default variable to the node, before any write operation is done to the node

    opcua::Result<opcua::NodeId> Node_Camera_Radius =
        opcua::services::addVariable(
            server, // The server on which this little shitling is to be mounted on
            parentNodeId, // Parent folder
            cameraNodeRadiusId, // id for this node, {namespace, id}
            "Radius", // Name for this node
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE) // READ and WRITE access
                .setDataType<int>(), // Type of data, can be int 16, 32, 64 and etc.
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent
        ); // Possible to add a default variable to the node, before any write operation is done to the node

    // Write a value (attribute) to the node
    while (true) {
        // update the variables with the new values
        // Automate the writing of the node ids
        // https://open62541pp.github.io/open62541pp/namespaceopcua_1_1services.html
        // https://open62541pp.github.io/open62541pp/classopcua_1_1DataValue.html
        // https://open62541pp.github.io/open62541pp/classopcua_1_1Variant.html
        opcua::services::writeDataValue(server, cameraNodeXId, opcua::DataValue(opcua::Variant(1)));
        opcua::services::writeDataValue(server, cameraNodeYId, opcua::DataValue(opcua::Variant(2)));
        opcua::services::writeDataValue(server, cameraNodeRadiusId, opcua::DataValue(opcua::Variant(3)));

        delay_server_iterate = server.runIterate();
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_server_iterate));
    }
}