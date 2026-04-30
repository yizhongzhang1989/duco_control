"""Unit tests for the FZI-backend orchestration in control_node.

Spinning up a full ``CartesianControlNode`` requires rclpy + a live
ROS graph, so these tests exercise the orchestration logic via
lightweight stubs:

* ``DummyClient`` mimics the ``call_async`` future-based API of
  ``rclpy.client.Client``.
* ``StubNode`` is a tiny shim that owns the methods under test
  (``_service_call_sync``, ``_switch_controllers_sync``,
  ``_switch_to_fzi_sync``, ``_switch_to_jtc_sync``).  We bind the real
  unbound methods from :class:`CartesianControlNode` onto it so the
  bodies under test are exactly the production code.

The full engage flow (which depends on rclpy timers / executors / TF
buffers) is covered by integration tests run on the real hardware.
"""

from __future__ import annotations

import time
from types import SimpleNamespace
from typing import Any, List, Optional

# controller_manager_msgs is part of every ROS 2 Humble install; this is
# a real ROS test environment, so the import always succeeds.
from controller_manager_msgs.srv import SwitchController  # type: ignore

from duco_cartesian_control.control_node import CartesianControlNode


# --- helpers ---------------------------------------------------------------


class DummyFuture:
    """Mimic ``rclpy.task.Future`` for ``_service_call_sync`` tests."""

    def __init__(self, result: Any = None,
                 raise_exc: Optional[Exception] = None,
                 resolve_after_s: float = 0.0):
        self._result = result
        self._raise = raise_exc
        self._resolve_after = resolve_after_s
        self._created = time.monotonic()
        self._cancelled = False

    def done(self) -> bool:
        return (time.monotonic() - self._created) >= self._resolve_after

    def cancel(self) -> None:
        self._cancelled = True

    def result(self) -> Any:
        if self._raise is not None:
            raise self._raise
        return self._result


class DummyClient:
    """Mimic ``rclpy.client.Client`` for ``_service_call_sync`` tests."""

    def __init__(self, *, available: bool = True,
                 future_factory=None,
                 raise_on_call: Optional[Exception] = None):
        self._available = available
        self._future_factory = future_factory or (lambda req: DummyFuture(
            result=SimpleNamespace(ok=True), resolve_after_s=0.0))
        self._raise_on_call = raise_on_call
        self.calls: List[Any] = []

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self._available

    def call_async(self, request) -> DummyFuture:
        self.calls.append(request)
        if self._raise_on_call is not None:
            raise self._raise_on_call
        return self._future_factory(request)


def _make_stub() -> SimpleNamespace:
    """Build a minimal stub that exposes the methods under test."""
    stub = SimpleNamespace()
    stub._fzi_controller_name = "cartesian_force_controller"
    stub._fzi_jtc_controller_name = "arm_1_controller"
    stub._fzi_service_timeout = 0.5
    stub._fzi_switch_cli = DummyClient()
    # Bind real (unbound) methods from CartesianControlNode onto the stub.
    stub._service_call_sync = (
        CartesianControlNode._service_call_sync.__get__(stub))
    stub._switch_controllers_sync = (
        CartesianControlNode._switch_controllers_sync.__get__(stub))
    stub._switch_to_fzi_sync = (
        CartesianControlNode._switch_to_fzi_sync.__get__(stub))
    stub._switch_to_jtc_sync = (
        CartesianControlNode._switch_to_jtc_sync.__get__(stub))
    return stub


# --- _service_call_sync ---------------------------------------------------


def test_service_call_sync_returns_response_when_ready():
    stub = _make_stub()
    expected = SimpleNamespace(ok=True, message="hi")
    stub._fzi_switch_cli = DummyClient(
        future_factory=lambda req: DummyFuture(result=expected))
    resp = stub._service_call_sync(stub._fzi_switch_cli, object(), 0.5)
    assert resp is expected


def test_service_call_sync_returns_none_when_service_unavailable():
    stub = _make_stub()
    stub._fzi_switch_cli = DummyClient(available=False)
    resp = stub._service_call_sync(stub._fzi_switch_cli, object(), 0.1)
    assert resp is None


def test_service_call_sync_returns_none_on_timeout():
    stub = _make_stub()
    # Future never resolves within the deadline.
    stub._fzi_switch_cli = DummyClient(
        future_factory=lambda req: DummyFuture(resolve_after_s=10.0))
    started = time.monotonic()
    resp = stub._service_call_sync(stub._fzi_switch_cli, object(), 0.05)
    assert resp is None
    assert time.monotonic() - started < 0.5  # actually bounded by timeout


def test_service_call_sync_swallows_result_exception():
    stub = _make_stub()
    stub._fzi_switch_cli = DummyClient(
        future_factory=lambda req: DummyFuture(
            raise_exc=RuntimeError("boom")))
    resp = stub._service_call_sync(stub._fzi_switch_cli, object(), 0.1)
    assert resp is None


# --- _switch_controllers_sync ---------------------------------------------


def test_switch_controllers_sync_builds_strict_request():
    stub = _make_stub()
    captured: List[Any] = []

    def _factory(req):
        captured.append(req)
        return DummyFuture(result=SimpleNamespace(ok=True))

    stub._fzi_switch_cli = DummyClient(future_factory=_factory)
    ok, why = stub._switch_controllers_sync(
        activate=["foo"], deactivate=["bar"])
    assert ok is True
    assert why == "ok"
    assert len(captured) == 1
    req = captured[0]
    assert req.activate_controllers == ["foo"]
    assert req.deactivate_controllers == ["bar"]
    assert req.strictness == SwitchController.Request.STRICT
    assert req.activate_asap is True


def test_switch_controllers_sync_reports_service_timeout():
    stub = _make_stub()
    stub._fzi_switch_cli = DummyClient(available=False)
    ok, why = stub._switch_controllers_sync(
        activate=["x"], deactivate=["y"])
    assert ok is False
    assert "did not respond" in why


def test_switch_controllers_sync_reports_controller_manager_failure():
    stub = _make_stub()
    stub._fzi_switch_cli = DummyClient(
        future_factory=lambda req: DummyFuture(
            result=SimpleNamespace(ok=False)))
    ok, why = stub._switch_controllers_sync(
        activate=["foo"], deactivate=["bar"])
    assert ok is False
    assert "ok=false" in why
    assert "foo" in why and "bar" in why


def test_switch_to_fzi_sync_targets_correct_controllers():
    stub = _make_stub()
    captured: List[Any] = []
    stub._fzi_switch_cli = DummyClient(
        future_factory=lambda req: (captured.append(req)
                                    or DummyFuture(
                                        result=SimpleNamespace(ok=True))))
    ok, _ = stub._switch_to_fzi_sync()
    assert ok is True
    assert captured[0].activate_controllers == ["cartesian_force_controller"]
    assert captured[0].deactivate_controllers == ["arm_1_controller"]


def test_switch_to_jtc_sync_targets_correct_controllers():
    stub = _make_stub()
    captured: List[Any] = []
    stub._fzi_switch_cli = DummyClient(
        future_factory=lambda req: (captured.append(req)
                                    or DummyFuture(
                                        result=SimpleNamespace(ok=True))))
    ok, _ = stub._switch_to_jtc_sync()
    assert ok is True
    assert captured[0].activate_controllers == ["arm_1_controller"]
    assert captured[0].deactivate_controllers == ["cartesian_force_controller"]


# --- concurrent resolution path -------------------------------------------


def test_service_call_sync_picks_up_response_resolved_during_wait():
    """Future is unresolved at start, resolves while ``_service_call_sync``
    is in its sleep loop -- exercises the multi-threaded executor path."""
    stub = _make_stub()
    expected = SimpleNamespace(ok=True)
    fut = DummyFuture(result=expected, resolve_after_s=0.05)
    stub._fzi_switch_cli = DummyClient(future_factory=lambda req: fut)
    resp = stub._service_call_sync(stub._fzi_switch_cli, object(), 0.5)
    assert resp is expected
