from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts' / 'policy_handoff_control.py'


def test_humble_parameter_service_compatibility():
    text = SCRIPT.read_text(encoding='utf-8')
    assert 'rclpy.parameter_client' not in text
    assert 'AsyncParameterClient' not in text
    assert 'from rcl_interfaces.srv import SetParameters' in text
    assert "'/lgh_st3215_driver/set_parameters'" in text
    assert 'request.parameters = [param.to_parameter_msg() for param in params]' in text
    assert 'self.driver_params.call_async(request)' in text


def test_handoff_client_registry_does_not_shadow_node_clients_property():
    text = SCRIPT.read_text(encoding='utf-8')
    assert 'self.clients =' not in text
    assert 'self._service_clients = {' in text
    assert 'client = self._service_clients[key]' in text
    assert "self._service_clients[key].srv_name" in text
