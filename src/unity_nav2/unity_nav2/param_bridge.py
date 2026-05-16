#!/usr/bin/env python3

"""将 Unity 发布的规划器参数应用于 Nav2 节点"""

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters


class PlannerParamBridge(Node):
    """Translate /planner_param_updates JSON into set_parameters service calls."""

    def __init__(self):
        super().__init__('planner_param_bridge')
        self._param_clients = {}
        self._pending = {}
        self._retries = {}
        self._inflight = set()
        self._sub = self.create_subscription(String, '/planner_param_updates', self._callback, 10)
        self._retry_timer = self.create_timer(0.5, self._retry_pending)
        self.get_logger().info('planner_param_bridge ready on /planner_param_updates')

    def _callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            node_name = str(payload.get('node', '')).strip()
            parameters = payload.get('parameters', {})
            if not node_name or not isinstance(parameters, dict):
                raise ValueError('payload must contain node and parameters object')
        except Exception as exc:
            self.get_logger().warning(f'invalid parameter payload: {exc}; raw={msg.data}')
            return

        parameter_list = [self._make_parameter(name, value) for name, value in parameters.items()]
        if not parameter_list:
            return

        self._pending[node_name] = parameter_list
        self._retries.pop(node_name, None)
        self._try_apply(node_name)

    def _retry_pending(self):
        for node_name in list(self._pending):
            self._try_apply(node_name)

    def _try_apply(self, node_name: str):
        if node_name in self._inflight:
            return

        parameters = self._pending.get(node_name)
        if not parameters:
            return

        client = self._client_for(node_name)
        if not client.wait_for_service(timeout_sec=0.2):
            retries = self._retries.get(node_name, 0) + 1
            self._retries[node_name] = retries
            if retries >= 10:
                self._pending.pop(node_name, None)
                self._retries.pop(node_name, None)
                self.get_logger().error(f'{node_name}: service unavailable after {retries} retries; dropped')
            return

        request = SetParameters.Request()
        request.parameters = parameters
        self._inflight.add(node_name)
        future = client.call_async(request)
        future.add_done_callback(lambda fut, node=node_name: self._log_result(node, fut))

    def _client_for(self, node_name: str):
        service_name = f'/{node_name.strip("/")}/set_parameters'
        if service_name not in self._param_clients:
            self._param_clients[service_name] = self.create_client(SetParameters, service_name)
        return self._param_clients[service_name]

    @staticmethod
    def _make_parameter(name: str, value):
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue()

        if isinstance(value, bool):
            parameter.value.type = ParameterType.PARAMETER_BOOL
            parameter.value.bool_value = value
        elif isinstance(value, int):
            parameter.value.type = ParameterType.PARAMETER_INTEGER
            parameter.value.integer_value = value
        elif isinstance(value, float):
            parameter.value.type = ParameterType.PARAMETER_DOUBLE
            parameter.value.double_value = value
        else:
            parameter.value.type = ParameterType.PARAMETER_STRING
            parameter.value.string_value = str(value)

        return parameter

    def _log_result(self, node_name: str, future):
        self._inflight.discard(node_name)

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f'failed to apply parameters to {node_name}: {exc}')
            return

        failed = [result.reason for result in response.results if not result.successful]
        if failed:
            self.get_logger().warning(f'{node_name} rejected parameters: {failed}')
        else:
            self._pending.pop(node_name, None)
            self._retries.pop(node_name, None)
            self.get_logger().info(f'{node_name} parameters updated')


def main(args=None):
    rclpy.init(args=args)
    node = PlannerParamBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
