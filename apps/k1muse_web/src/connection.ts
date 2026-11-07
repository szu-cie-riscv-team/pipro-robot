import { FoxgloveClient, Channel, Service } from "@foxglove/ws-protocol";
import { parse } from "@foxglove/rosmsg";
import { MessageReader, MessageWriter } from "@foxglove/rosmsg2-serialization";
import { Target, id } from "./core";

// Only the three small command types are fixed here. Display schemas come from Bridge.
const teleopSchema = `string token
uint32 sequence
geometry_msgs/Twist velocity
================================================================================
MSG: geometry_msgs/Twist
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
`;
export class Connection {
  ws: WebSocket;
  client: FoxgloveClient;
  token = "";
  clientId: string;
  onMessage: (topic: string, message: any) => void = () => {};
  onClose: () => void = () => {};
  onReady: () => void = () => {};
  onError: (error: string) => void = () => {};
  private channels = new Map<number, Channel>();
  private subscriptions = new Map<number, { channel: number; reader: MessageReader }>();
  private desired = new Set<string>();
  private service?: Service;
  private requestWriter?: MessageWriter;
  private responseReader?: MessageReader<any>;
  private calls = new Map<number, { resolve: (value: any) => void; reject: (e: Error) => void; timer: ReturnType<typeof setTimeout> }>();
  private callId = 0;
  private channelId?: number;
  private sequence = 0;
  private teleopWriter = new MessageWriter(parse(teleopSchema, { ros2: true }));
  constructor(url: string, clientId: string, control = false) {
    this.clientId = clientId;
    this.ws = new WebSocket(url, [FoxgloveClient.SUPPORTED_SUBPROTOCOL]);
    this.client = new FoxgloveClient({ ws: this.ws });
    this.client.on("open", () => {
      if (control) this.channelId = this.client.advertise({
        topic: "/k1muse/web/teleop", encoding: "cdr", schemaName: "k1muse_interfaces/msg/WebTeleop",
        schemaEncoding: "ros2msg", schema: teleopSchema
      });
    });
    this.client.on("advertise", channels => {
      for (const c of channels) this.channels.set(c.id, c);
      this.subscribeDesired();
    });
    this.client.on("unadvertise", ids => {
      for (const id of ids) {
        this.channels.delete(id);
        for (const [sub, value] of this.subscriptions) if (value.channel === id) {
          this.client.unsubscribe(sub); this.subscriptions.delete(sub);
        }
      }
    });
    this.client.on("message", message => {
      const sub = this.subscriptions.get(message.subscriptionId);
      if (!sub) return;
      const channel = this.channels.get(sub.channel);
      if (!channel) return;
      try { this.onMessage(channel.topic.split("/").at(-1)!, sub.reader.readMessage(message.data)); }
      catch (e) { this.onError(String(e)); }
    });
    this.client.on("advertiseServices", services => {
      if (!control) return;
      const service = services.find(s => s.name === "/k1muse/web/request");
      if (!service) return;
      try {
        this.service = service;
        this.requestWriter = new MessageWriter(parse(service.request?.schema ?? service.requestSchema!, { ros2: true }));
        this.responseReader = new MessageReader(parse(service.response?.schema ?? service.responseSchema!, { ros2: true }));
        this.onReady();
      } catch (e) { this.onError(String(e)); }
    });
    this.client.on("unadvertiseServices", ids => {
      if (this.service && ids.includes(this.service.id)) { this.service = undefined; this.onError("操作节点服务已离线"); }
    });
    this.client.on("serviceCallResponse", response => {
      const call = this.calls.get(response.callId);
      if (!call || !this.responseReader) return;
      clearTimeout(call.timer); this.calls.delete(response.callId);
      try {
        const data = this.responseReader.readMessage(response.data);
        if (!data.accepted) call.reject(Object.assign(new Error(data.error || "操作被拒绝"), { rejected: true }));
        else call.resolve({ ...data, result: JSON.parse(data.result || "{}") });
      } catch (e) { call.reject(e instanceof Error ? e : new Error(String(e))); }
    });
    this.client.on("serviceCallFailure", response => {
      const call = this.calls.get(response.callId);
      if (call) { clearTimeout(call.timer); this.calls.delete(response.callId); call.reject(new Error(response.message)); }
    });
    this.client.on("status", message => { if (message.level === 2) this.onError(message.message); });
    this.client.on("error", error => this.onError(error.message));
    this.client.on("close", () => {
      this.token = "";
      for (const call of this.calls.values()) { clearTimeout(call.timer); call.reject(new Error("连接已断开")); }
      this.calls.clear(); this.onClose();
    });
  }
  subscribe(topics: string[]) {
    this.desired = new Set(topics.map(t => "/k1muse/web/" + t));
    for (const [id, sub] of this.subscriptions) {
      if (!this.desired.has(this.channels.get(sub.channel)?.topic ?? "")) {
        this.client.unsubscribe(id); this.subscriptions.delete(id);
      }
    }
    this.subscribeDesired();
  }
  private subscribeDesired() {
    for (const [id, c] of this.channels) {
      if (!this.desired.has(c.topic) || [...this.subscriptions.values()].some(s => s.channel === id)) continue;
      try {
        const reader = new MessageReader(parse(c.schema, { ros2: true }));
        this.subscriptions.set(this.client.subscribe(id), { channel: id, reader });
      } catch (e) { this.onError(String(e)); }
    }
  }
  async request(operation: string, args: unknown = {}, requestId = id()): Promise<any> {
    if (!this.service || !this.requestWriter || this.ws.readyState !== WebSocket.OPEN)
      throw new Error("操作服务尚未连接");
    const callId = ++this.callId;
    const data = this.requestWriter.writeMessage({ request_id: requestId, client_id: this.clientId,
      token: this.token, operation, arguments: JSON.stringify(args) });
    return await new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.calls.delete(callId);
        reject(new Error(`请求结果未知，请查询操作结果（${requestId}）`));
      }, 4000);
      this.calls.set(callId, { resolve, reject, timer });
      this.client.sendServiceCallRequest({ serviceId: this.service!.id, callId, encoding: "cdr", data });
    });
  }
  send(v: Target): boolean {
    if (!this.token || this.channelId == null || this.ws.readyState !== WebSocket.OPEN || this.ws.bufferedAmount > 0)
      return false;
    this.client.sendMessage(this.channelId, this.teleopWriter.writeMessage({
      token: this.token, sequence: ++this.sequence,
      velocity: { linear: { x: v.x, y: v.y, z: 0 }, angular: { x: 0, y: 0, z: v.yaw } }
    }));
    return true;
  }
  close() { this.client.close(); }
}
