namespace Rcl.Graph;

/// <summary>
/// Represents a change to the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
public abstract record RosGraphEvent(RosGraph Sender);


/// <summary>
/// A node appeared on or disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Node">The node which appeared or disappeared.</param>
public abstract record RosGraphNodeEvent(RosGraph Sender, RosNode Node) : RosGraphEvent(Sender);

/// <summary>
/// A node appeared on the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Node">The node which appeared on the graph.</param>
public record NodeAppearedEvent(RosGraph Sender, RosNode Node) : RosGraphNodeEvent(Sender, Node);

/// <summary>
/// A node disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Node"></param>
public record NodeDisappearedEvent(RosGraph Sender, RosNode Node) : RosGraphNodeEvent(Sender, Node);


/// <summary>
/// A topic appeared on or disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Topic">The topic which appeared or disappeared.</param>
public abstract record RosGraphTopicEvent(RosGraph Sender, RosTopic Topic) : RosGraphEvent(Sender);

/// <summary>
/// A topic appeared on the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Topic">The topic which appeared on the graph.</param>
public record TopicAppearedEvent(RosGraph Sender, RosTopic Topic) : RosGraphTopicEvent(Sender, Topic);

/// <summary>
/// A topic disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Topic">The topic which disappeared from the graph.</param>
public record TopicDisappearedEvent(RosGraph Sender, RosTopic Topic) : RosGraphTopicEvent(Sender, Topic);


/// <summary>
/// A publisher appeared on or disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Publisher">The publisher which appeared or disappeared.</param>
public abstract record RosGraphPublisherEvent(RosGraph Sender, RosTopicEndPoint Publisher) : RosGraphEvent(Sender);

/// <summary>
/// A publisher appeared on the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Publisher">The publisher which appeared on the graph.</param>
public record PublisherAppearedEvent(RosGraph Sender, RosTopicEndPoint Publisher) : RosGraphPublisherEvent(Sender, Publisher);

/// <summary>
/// A publisher disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Publisher">The publisher which disappeared from the graph.</param>
public record PublisherDisappearedEvent(RosGraph Sender, RosTopicEndPoint Publisher) : RosGraphPublisherEvent(Sender, Publisher);


/// <summary>
/// A subscirber appeared on or disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Subscriber">The subscriber which appeared or disappeared.</param>
public abstract record RosGraphSubscriberEvent(RosGraph Sender, RosTopicEndPoint Subscriber) : RosGraphEvent(Sender);

/// <summary>
/// A subscriber appeared on the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Subscriber">The subscriber which appeared on the graph.</param>
public record SubscriberAppearedEvent(RosGraph Sender, RosTopicEndPoint Subscriber) : RosGraphSubscriberEvent(Sender, Subscriber);

/// <summary>
/// A subscriber disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Subscriber">The subscriber which disappeared from the graph.</param>
public record SubscriberDisappearedEvent(RosGraph Sender, RosTopicEndPoint Subscriber) : RosGraphSubscriberEvent(Sender, Subscriber);


/// <summary>
/// A service appeared on or disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Service">The service which appeared or disappeared.</param>
public abstract record RosGraphServiceEvent(RosGraph Sender, RosService Service) : RosGraphEvent(Sender);

/// <summary>
/// A service appeared on the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Service">The service which appeared on the graph.</param>
public record ServiceAppearedEvent(RosGraph Sender, RosService Service) : RosGraphServiceEvent(Sender, Service);

/// <summary>
/// A service disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Service">The service which disappeared from the graph.</param>
public record ServiceDisappearedEvent(RosGraph Sender, RosService Service) : RosGraphServiceEvent(Sender, Service);


/// <summary>
/// A service server appeared on or disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Server">The service server which appeared or disappeared.</param>
public abstract record RosGraphServerEvent(RosGraph Sender, RosServiceEndPoint Server) : RosGraphEvent(Sender);

/// <summary>
/// A service server appeared on the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Server">The service server which appeared on the graph.</param>
public record ServerAppearedEvent(RosGraph Sender, RosServiceEndPoint Server) : RosGraphServerEvent(Sender, Server);

/// <summary>
/// A service server disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Server">The service server which disappeared from the graph.</param>
public record ServerDisappearedEvent(RosGraph Sender, RosServiceEndPoint Server) : RosGraphServerEvent(Sender, Server);


/// <summary>
/// A service client appeared on or disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Client">The service client which appeared or disappeared.</param>
public abstract record RosGraphClientEvent(RosGraph Sender, RosServiceEndPoint Client) : RosGraphEvent(Sender);

/// <summary>
/// A service client appeared on the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Client">The service client which appeared on the graph.</param>
public record ClientAppearedEvent(RosGraph Sender, RosServiceEndPoint Client) : RosGraphClientEvent(Sender, Client);

/// <summary>
/// A service client disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Client">The service client which disappeared from the graph.</param>
public record ClientDisappearedEvent(RosGraph Sender, RosServiceEndPoint Client) : RosGraphClientEvent(Sender, Client);


/// <summary>
/// An action appeared on or disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Action">The action which appeared or disappeared.</param>
public abstract record RosGraphActionEvent(RosGraph Sender, RosAction Action) : RosGraphEvent(Sender);

/// <summary>
/// An action appeared on the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Action">The action which appeared on the graph.</param>
public record ActionAppearedEvent(RosGraph Sender, RosAction Action) : RosGraphActionEvent(Sender, Action);

/// <summary>
/// An action disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="Action">The action which disappeared from the graph.</param>
public record ActionDisappearedEvent(RosGraph Sender, RosAction Action) : RosGraphActionEvent(Sender, Action);


/// <summary>
/// An action server appeared on or disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="ActionServer">The action server which appeared or disappeared.</param>
public abstract record RosGraphActionServerEvent(RosGraph Sender, RosActionEndPoint ActionServer) : RosGraphEvent(Sender);

/// <summary>
/// An action server appeared on the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="ActionServer">The action server which appeared on the graph.</param>
public record ActionServerAppearedEvent(RosGraph Sender, RosActionEndPoint ActionServer) : RosGraphActionServerEvent(Sender, ActionServer);

/// <summary>
/// An action server disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="ActionServer">The action server which disappeared from the graph.</param>
public record ActionServerDisappearedEvent(RosGraph Sender, RosActionEndPoint ActionServer) : RosGraphActionServerEvent(Sender, ActionServer);


/// <summary>
/// An action client appeared on or disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="ActionClient">The action client which appeared or disappeared.</param>
public abstract record RosGraphActionClientEvent(RosGraph Sender, RosActionEndPoint ActionClient) : RosGraphEvent(Sender);

/// <summary>
/// An action client appeared on the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="ActionClient">The action client which appeared on the graph.</param>
public record ActionClientAppearedEvent(RosGraph Sender, RosActionEndPoint ActionClient) : RosGraphActionClientEvent(Sender, ActionClient);

/// <summary>
/// An action client disappeared from the ROS graph.
/// </summary>
/// <param name="Sender">The sender <see cref="RosGraph"/> instance of this event.</param>
/// <param name="ActionClient">The action client which disappeared from the graph.</param>
public record ActionClientDisappearedEvent(RosGraph Sender, RosActionEndPoint ActionClient) : RosGraphActionClientEvent(Sender, ActionClient);