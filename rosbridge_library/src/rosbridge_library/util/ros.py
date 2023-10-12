"""ROS utilities"""


def is_topic_published(node, topic_name):
    """Checks if a topic is published on a node."""
    published_topic_data = node.get_publisher_names_and_types_by_node(node.get_name(), "")
    return topic_name in [topic[0] for topic in published_topic_data]


def is_topic_subscribed(node, topic_name):
    """Checks if a topic is subscribed to by a node."""
    subscribed_topic_data = node.get_subscriber_names_and_types_by_node(node.get_name(), "")
    return topic_name in [topic[0] for topic in subscribed_topic_data]
