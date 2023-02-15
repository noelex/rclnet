namespace Rcl.Graph;

public abstract record RosGraphEvent(RosGraph Sender);

public abstract record RosGraphNodeEvent(RosGraph Sender, RosNode Node) : RosGraphEvent(Sender);
public record NodeEstablishedEvent(RosGraph Sender, RosNode Node) : RosGraphNodeEvent(Sender, Node);
public record NodeRemovedEvent(RosGraph Sender, RosNode Node) : RosGraphNodeEvent(Sender, Node);

public abstract record RosGraphTopicEvent(RosGraph Sender, RosTopic Topic) : RosGraphEvent(Sender);
public record TopicEstablishedEvent(RosGraph Sender, RosTopic Topic) : RosGraphTopicEvent(Sender, Topic);
public record TopicRemovedEvent(RosGraph Sender, RosTopic Topic) : RosGraphTopicEvent(Sender, Topic);

public abstract record RosGraphPublisherEvent(RosGraph Sender, RosTopicEndPoint Publisher) : RosGraphEvent(Sender);
public record PublisherEstablishedEvent(RosGraph Sender, RosTopicEndPoint Publisher) : RosGraphPublisherEvent(Sender, Publisher);
public record PublisherRemovedEvent(RosGraph Sender, RosTopicEndPoint Publisher) : RosGraphPublisherEvent(Sender, Publisher);

public abstract record RosGraphSubscriberEvent(RosGraph Sender, RosTopicEndPoint Subscriber) : RosGraphEvent(Sender);
public record SubscriberEstablishedEvent(RosGraph Sender, RosTopicEndPoint Subscriber) : RosGraphSubscriberEvent(Sender, Subscriber);
public record SubscriberRemovedEvent(RosGraph Sender, RosTopicEndPoint Subscriber) : RosGraphSubscriberEvent(Sender, Subscriber);

public abstract record RosGraphServiceEvent(RosGraph Sender, RosService Service) : RosGraphEvent(Sender);
public record ServiceEstablishedEvent(RosGraph Sender, RosService Service) : RosGraphServiceEvent(Sender, Service);
public record ServiceRemovedEvent(RosGraph Sender, RosService Service) : RosGraphServiceEvent(Sender, Service);

public abstract record RosGraphServerEvent(RosGraph Sender, RosServiceEndPoint Server) : RosGraphEvent(Sender);
public record ServerEstablishedEvent(RosGraph Sender, RosServiceEndPoint Server) : RosGraphServerEvent(Sender, Server);
public record ServerRemovedEvent(RosGraph Sender, RosServiceEndPoint Server) : RosGraphServerEvent(Sender, Server);

public abstract record RosGraphClientEvent(RosGraph Sender, RosServiceEndPoint Client) : RosGraphEvent(Sender);
public record ClientEstablishedEvent(RosGraph Sender, RosServiceEndPoint Client) : RosGraphClientEvent(Sender, Client);
public record ClientRemovedEvent(RosGraph Sender, RosServiceEndPoint Client) : RosGraphClientEvent(Sender, Client);

public abstract record RosGraphActionEvent(RosGraph Sender, RosAction Action) : RosGraphEvent(Sender);
public record ActionEstablishedEvent(RosGraph Sender, RosAction Action) : RosGraphActionEvent(Sender, Action);
public record ActionRemovedEvent(RosGraph Sender, RosAction Action) : RosGraphActionEvent(Sender, Action);

public abstract record RosGraphActionServerEvent(RosGraph Sender, RosActionEndPoint ActionServer) : RosGraphEvent(Sender);
public record ActionServerEstablishedEvent(RosGraph Sender, RosActionEndPoint ActionServer) : RosGraphActionServerEvent(Sender, ActionServer);
public record ActionServerRemovedEvent(RosGraph Sender, RosActionEndPoint ActionServer) : RosGraphActionServerEvent(Sender, ActionServer);

public abstract record RosGraphActionClientEvent(RosGraph Sender, RosActionEndPoint ActionClient) : RosGraphEvent(Sender);
public record ActionClientEstablishedEvent(RosGraph Sender, RosActionEndPoint ActionClient) : RosGraphActionClientEvent(Sender, ActionClient);
public record ActionClientRemovedEvent(RosGraph Sender, RosActionEndPoint ActionClient) : RosGraphActionClientEvent(Sender, ActionClient);