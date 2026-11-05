# RAI navigation companion

This optional service adds natural-language task control to the existing P2 navigation
Mission. It uses the actual upstream RAI `create_react_runnable` graph and ToolRunner,
with `ChatDeepSeek` and NavBridge tools. It does not implement a second path planner
or publish velocity commands. V1/V2 baseline tags remain unchanged.

## Components

- RAI core 2.12.4, source `RobotecAI/rai@b018729833742c6f71e1e9f332ee2b435676fb4f`.
- DeepSeek `deepseek-v4-pro`, non-thinking tool mode for interactive task dispatch;
  selectable with `P2_AGENT_MODEL`. The provider key is `DEEPSEEK_API_KEY` in a private
  environment file; never put it in the browser, source tree or command-line flags.
- NavBridge P2 provider and its additive Console SDK extension.
- `navigation_agent` loopback HTTP service, default port8950; SDK port8940.
- Console `/agent` and `/api/agent/*` proxy, so the existing18930 laptop forwarding
  and existing robot Console origin can serve the assistant without a second login.

## Installation

Use an isolated Python3.10–3.12 environment. Install the SDK project, `langchain-deepseek==1.1.1`
and the pinned upstream RAI core from its `src/rai_core` subdirectory. RAI2.12.4 has not
been published to PyPI at the time of this implementation; do not silently substitute
another package/version. The deployment records the verified source and wheel SHA.
Keep NumPy1.26.4 and OpenCV4.11.0.86 together for this RAI environment.

```sh
navbridge --backend p2 --p2-url http://127.0.0.1:8930 --port 8940 --state-dir /private/path/sdk
```

With the provider key already in the service environment:

```sh
p2-nav-agent --sdk-url http://127.0.0.1:8940 --port 8950 --state-dir /private/path/agent
```

Copy `deploy/agent-config.toml` to the service working directory as `config.toml` to
disable optional external tracing. The SDK and Agent are independent services;
their startup does not start D, drivers, lidar or a navigation task.

## User flow

Open `/agent` on the existing Console. Examples:

- “有哪些点位和路线？”
- “去点位2”：run the saved visit, including its existing heading, introduction and wait.
- “按20到1的路线参观”：resolve and run the matching saved reverse route.
- “现在到哪里了？”
- “暂停” / “继续导航” / “介绍完了，继续下一站”。
- “停止导航”：bypasses cloud inference and requests navigation stop.

A clear navigation instruction directly submits a task; no second confirmation is
added. E-left permits, E-middle pauses and requires an explicit continuation, and
E-right/stick takeover terminates automatic intent as before. The Agent does not
re-arm or replay an ended task on its own. Direction and speech are executed by the
existing Mission, not by a duplicate Agent sequence.

P2 status includes `task_active` and `current_task` from the native Mission/session,
including tasks started outside this chat. An E-middle/left transition can leave an
empty session in `paused` without a goal; the SDK reports that as idle and accepts a
new explicit visit without a stop/cancel round trip. A real paused task, introduction,
guest wait, or outstanding cancellation still occupies the task slot. Failed SDK
submissions remain in the journal as failures and are not running tasks or replayed.

The first version uses saved points and routes. It does not yet create arbitrary
new multi-point itineraries, edit product knowledge, or capture microphone audio.
Typing works immediately; spoken-language capture can be added as a separate input
without changing the navigation task tools. Product information is not invented.

## Persistence and cancellation

Each browser request has a stable UUID and a durable SQLite result. Repeated delivery
of that UUID returns the same result; a different body with the same UUID is rejected.
SDK dispatch uses a stable idempotency key. Restarted queued/working requests are marked
unknown and never replayed. The native mission can continue independently of the model.
Stop advances a local command epoch: a model response arriving after that stop cannot
dispatch the earlier request. Model/observer failure never cancels an accepted task.

UI task status and post-dispatch answers use actual SDK observations. An accepted
task is not reported as arrived. The request journal and conversation stay private.
No point cloud or full occupancy map is sent to the model.
Before each model turn, the companion reads the current point/route catalogue and
navigation status itself and records this snapshot as `read_live_context`. Fresh
facts take precedence over conversational memory, including after an operator has
added a point or changed a route between messages.

## Verification

`tests_agent` runs the actual upstream RAI graph with a scripted LLM and the SDK HTTP
synthetic backend: dispatch, true task receipts, no false arrival, idempotency after
reopen, stop during inference, unknown destinations, pause/resume and cancellation.
An additional two-turn test changes the catalogue between messages and checks that
the actual RAI graph receives the new point in its current system context.
This does not assert physical arrival. Production verification separately records
real1006 status/tool receipts, software checksums and unchanged gait/driver identities.
