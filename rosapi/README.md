rosapi
===============

## Nodes

### rosapi (Executable: `rosapi_node`)
Provides services for getting various ROS meta-information, including ROS topic, services, interfaces or
action servers and managing ROS parameters.

#### Services

  * `~/topics` (type: `rosapi_msgs/srv/Topics`)

    Return a list of all the topics being published.

  * `~/interfaces` (type: `rosapi_msgs/srv/Interfaces`)

    Return a list of all the interfaces in the system.

  * `~/topics_for_type` (type: `rosapi_msgs/srv/TopicsForType`)

    Return a list of all the topics that are publishing a given type.

  * `~/topics_and_raw_types` (type: `rosapi_msgs/srv/TopicsAndRawTypes`)

    Return a list of all the topics being published, and their raw types.

  * `~/services` (type: `rosapi_msgs/srv/Services`)

    Return a list of all the services being advertised.

  * `~/services_for_type` (type: `rosapi_msgs/srv/ServicesForType`)

    Return a list of all the services that are publishing a given type.

  * `~/nodes` (type: `rosapi_msgs/srv/Nodes`)

    Return a list of all the nodes that are registered.

  * `~/node_details` (type: `rosapi_msgs/srv/NodeDetails`)

    Return a node description including subscribing, publishing, and services.

  * `~/action_servers` (type: `rosapi_msgs/srv/GetActionServers`)

    Return a list of action servers based on actions standard topics.

  * `~/action_type` (type: `rosapi_msgs/srv/ActionType`)

    Given the name of an action, return its type.

  * `~/topic_type` (type: `rosapi_msgs/srv/TopicType`)

    Given the name of a topic, return the name of the type of that topic.

  * `~/service_type` (type: `rosapi_msgs/srv/ServiceType`)

    Given the name of a service, return the type of that service.

  * `~/publishers` (type: `rosapi_msgs/srv/Publishers`)

    Given the name of a topic, return a list of node names that are publishing on that topic.

  * `~/subscribers` (type: `rosapi_msgs/srv/Subscribers`)

    Given the name of a topic, return a list of node names that are subscribing to that topic.

  * `~/service_providers` (type: `rosapi_msgs/srv/ServiceProviders`)

    Given the name of a service, returns a list of node names that are advertising that service type.

  * `~/service_node` (type: `rosapi_msgs/srv/ServiceNode`)

    Given the name of a service, returns the name of the node that is providing that service.

  * `~/message_details` (type: `rosapi_msgs/srv/MessageDetails`)

    Given the name of a message type, return the TypeDef for that type.

  * `~/service_request_details` (type: `rosapi_msgs/srv/ServiceRequestDetails`)

    Given the name of a service type, return the TypeDef for the request message of that service type.

  * `~/service_response_details` (type: `rosapi_msgs/srv/ServiceResponseDetails`)

    Given the name of a service type, return the TypeDef for the response message of that service type.

  * `~/action_goal_details` (type: `rosapi_msgs/srv/ActionGoalDetails`)

    Given the name of an action type, return the TypeDef for the goal message of that action type.

  * `~/action_result_details` (type: `rosapi_msgs/srv/ActionResultDetails`)

    Given the name of an action type, return the TypeDef for the result message of that action type.

  * `~/action_feedback_details` (type: `rosapi_msgs/srv/ActionFeedbackDetails`)

    Given the name of an action type, return the TypeDef for the feedback message of that action type.

  * `~/set_param` (type: `rosapi_msgs/srv/SetParam`)

    Set a parameter value on a specific node.

  * `~/get_param` (type: `rosapi_msgs/srv/GetParam`)

    Get a parameter value from a specific node.

  * `~/has_param` (type: `rosapi_msgs/srv/HasParam`)

    Check if a parameter exists on a specific node.

  * `~/delete_param` (type: `rosapi_msgs/srv/DeleteParam`)

    Delete a parameter from a specific node.

  * `~/get_param_names` (type: `rosapi_msgs/srv/GetParamNames`)

    Get a list of all parameter names.

  * `~/get_time` (type: `rosapi_msgs/srv/GetTime`)

    Get the current ROS time.

  * `~/get_ros_version` (type: `rosapi_msgs/srv/GetROSVersion`)

    Get the ROS version and distribution name.

#### Parameters

  * `params_timeout` (float, default: `5.0`)

    Timeout in seconds for param-related services.

  * `topics_glob` (string, default: `[*]`)
  * `services_glob` (string, default: `[*]`)
  * `params_glob` (string, default: `[*]`)

    Each of the glob parameters may contain an array of one or more match patterns. Resources that match any of the specified patterns will be returned by calls to the rosapi services.

    An example launch file which enables all information to be returned:

    ```
    <launch>
      <node name="rosapi" pkg="rosapi" exec="rosapi_node">
        <param name="topics_glob" value="'[*]'" />
        <param name="services_glob" value="'[*]'" />
        <param name="params_glob" value="'[*]'" />
      </node>
    </launch>
    ```

    This example launch file enables only rosout and certain camera topics:
    ```
    <launch>
      <node name="rosapi" pkg="rosapi" exec="rosapi_node">
        <param name="topics_glob" value="'[/rosout, /camera/rgb/*]'" />
      </node>
    </launch>
    ```
