#include <iostream>
 
#include <open62541pp/server.hpp>
#include <open62541pp/services/nodemanagement.hpp>

#include <gpiod.hpp>

// Sources:
// https://open62541pp.github.io/open62541pp/server_datasource_8cpp-example.html
 
/// Templated data source that stores the data of type `T` internally.

template <typename T>
class DataSource : public opcua::DataSourceBase {
private: 
    gpiod::line _button_line;

    
public:
    DataSource(gpiod::chip __chip, unsigned int __line) {
        this->_button_line = __chip.get_line(__line);
        this->_button_line.request({"input", gpiod::line_request::DIRECTION_INPUT, 0});
    }

    opcua::StatusCode read(
        [[maybe_unused]] opcua::Session& session,
        [[maybe_unused]] const opcua::NodeId& id,
        [[maybe_unused]] const opcua::NumericRange* range,
        opcua::DataValue& dv,
        bool timestamp
    ) override {
        dv.setValue(opcua::Variant(_button_line.get_value()));
        if (timestamp) {
            dv.setSourceTimestamp(opcua::DateTime::now());
            dv.status();
            // dv.setValue((opcua::Variant)button_line.get_value());
        }
        std::cout << "Read value from data source: " << dv.value().to<T>() << "\n";
        return UA_STATUSCODE_GOOD;
    }
 
    opcua::StatusCode write(
        [[maybe_unused]] opcua::Session& session,
        [[maybe_unused]] const opcua::NodeId& id,
        [[maybe_unused]] const opcua::NumericRange* range,
        const opcua::DataValue& dv
    ) override {
        data = dv.value().to<T>();
        std::cout << "Write value to data source: " << data << "\n";
        return UA_STATUSCODE_GOOD;
    }

    T data{};
};
 
int main() {
    opcua::Server server;

    gpiod::chip chip("gpiochip0");

 
    // Add variable node
    const auto parentNode =
        opcua::services::addVariable(
			server, // The server on which this little shitling is to be mounted on
            opcua::ObjectId::ObjectsFolder, // Parent folder
            {1, 1000}, // id for this node, {namespace, id}
            "ParentNode", // Name for this node
            opcua::VariableAttributes{}
	            .setAccessLevel(UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE) // READ and WRITE access
	            .setDataType<int>(), // Type of data, can be int 16, 32, 64 and etc.
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent
		).value(); // Possible to add a default variable to the node, before any write operation is done to the node
 
    const auto CameraX_node =
        opcua::services::addVariable(
            server,
            {1, 1000},
            {1, 1001},
            "Camera X",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE)
                .setDataType<int>(),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent
        ).value();

    const auto CameraY_node =
        opcua::services::addVariable(
            server,
            {1, 1000},
            {1, 1002},
            "Camera Y",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE)
                .setDataType<int>(),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent
        ).value();



    // Define data source
    DataSource<int> dataSource_x(chip, 14);
    DataSource<int> dataSource_y(chip, 15);

    server.setVariableNodeDataSource(CameraX_node, dataSource_x);
    server.setVariableNodeDataSource(CameraY_node, dataSource_y);
 
    server.run();
}
