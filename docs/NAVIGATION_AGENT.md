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
- “往前走2米” / “前进50厘米”：resolve a target from the fresh body pose, in metres.
- “右转90度” / “原地向左转两圈”：signed accumulated rotation; preserve whole turns.
- “去介绍A产品” / “去介绍A产品，B产品”：visit named saved points in that order.
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

The model receives pause/resume/cancel tools only when the current user message
explicitly requests such controls; a new point or route command cannot inherit a
stop suggestion from old chat history. A completed visit permits a newly requested
route directly. The UI shows current execution separately from the latest chat
task result and updates an accepted task bubble when success is observed. A stopped
control channel does not rewrite the saved task's successful arrival/introduction.

The current request is bound to the live catalogue before an action is offered to
the model. A query cannot dispatch, a specified point cannot be substituted, and a
single navigation request shares one SDK submission/identity across tool calls.
When a model omits an action or its answer fails, a unique, explicitly requested
destination can still be submitted once. Failed or uncertain native submissions
are never automatically retried. An accepted receipt survives model/observer errors;
unknown acceptance remains unknown. The SDK observes prior runs before replacing
the provider's current-run snapshot so a completed but unpolled task is retained.

## Offline validation without provider credit

```sh
python scripts/check-agent-offline.py --output /tmp/agent-offline-validation.json
node --test tests_agent/ui-state.test.cjs
python -m unittest discover -s tests_sdk -v
```

The Agent tests use the actual upstream RAI graph with a local scripted/adversarial
model. The child removes provider credentials and denies Python DNS/connect calls
outside servers bound by that test process, including other localhost services.
No DeepSeek API request is needed. Cases cover request binding, multi-turn state,
HTTP request identity/authentication, cancel races, malformed/lost receipts and
model failure. The integration suite uses the real Session and Mission engines for
both20-point orders, introduction/dwell and manual takeover, with explicitly injected
goal/audio receipts. Its result is a software-state test, not Nav2 collision/physics
simulation, real walking acceptance, or a measurement of DeepSeek language accuracy.

Named-product tours build a temporary Mission snapshot and do not save or alter the
catalogue. Each product must resolve to a real saved point. The point's own script,
linked product script, heading, dwell and guest-wait remain native Mission behavior.
An unknown second product does not dispatch only the first. No product claims are
generated from a name. Configure product/point names and speech on `/reception`.

Relative forward uses the fresh `map -> base_link` pose when the Console prepares
the task, not a lidar-head frame or input multiplied by time. It means a target
offset along the initial body-forward axis, not accumulated path length: obstacle
avoidance may make the travelled path longer. Arrival uses existing navigation
tolerances, not centimetre precision. Rotation uses the native Nav2 Spin accumulated
angle, retaining the unwrapped action distance even when the final heading wraps.
Positive is left/counterclockwise; unspecified direction defaults left. Whole turns
are never considered complete just because the start/end headings are equal.
The generated Spin timeout scales with angle from the existing45seconds per180deg;
the existing obstacle/authority checks, gait inputs and S-shaped start remain.

These new relative/tour capabilities require Console SDK extension1.1; older
Consoles explicitly advertise them as unsupported. Their local tests include the
real Console intent methods and Session/Mission with injected receipts; they do not
claim real walking-distance accuracy or physical multi-turn acceptance.
This version does not edit product knowledge or capture microphone audio.
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
