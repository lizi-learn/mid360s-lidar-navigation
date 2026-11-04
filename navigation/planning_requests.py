"""Bounded parameter-RPC waiting, separate from sensor freshness and actions."""
import time


class RouteReplyTimeout(TimeoutError):
    pass


def wait_route_reply(future, client, is_current, timeout=8., *, clock=None, sleep=None):
    clock = clock or time.monotonic
    sleep = sleep or time.sleep
    end = clock() + timeout
    try:
        while True:
            # Cancellation wins even if a delayed reply arrived on this tick.
            if not is_current():
                raise ValueError('路线参数请求已取消或目标已更换')
            if future.done():
                return future.result()
            if clock() >= end:
                raise RouteReplyTimeout('路线终点参数响应超时')
            sleep(.02)
    finally:
        if not future.done():
            client.remove_pending_request(future)
