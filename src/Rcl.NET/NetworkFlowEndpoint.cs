using System.Net;
using System.Net.Sockets;

namespace Rcl;

/// <summary>
/// Represents a network flow endpoint of a publisher or subscription.
/// </summary>
/// <param name="TransportProtocol">Gets the type of the transport protocol being used by current network flow endpoint.</param>
/// <param name="EndPoint">Gets the IP address and port of the network flow endpoint.</param>
/// <param name="FlowLabel">Gets the flow label of the endpoint. This property is only relevant to a publisher endpoint.</param>
/// <param name="Dscp">Gets the differential service code point of the endpoint. This property is only relevant to a publisher endpoint.</param>
public record NetworkFlowEndpoint
(
    ProtocolType TransportProtocol,
    IPEndPoint EndPoint,
    uint FlowLabel,
    byte Dscp
);