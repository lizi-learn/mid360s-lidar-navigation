# Optional paired APP gateway

This own-code FastAPI router forwards only allowlisted navigation workbench
requests to `127.0.0.1:8930`. Mount its `api_router` in an already paired backend;
it opens no listening port, changes no robot program and provides no gait input.
Never restart a backend while its cgroup still owns actuator processes.

An Android APP can show `/api/v1/modules/navigation/workbench` inside its map tab,
poll `/snapshot` for verified physical E-left, and provide a native cancel button
using `/actions/cancel` with its `X-YMbot-Client: g12-console/<version>` marker.
The native cancel path obtains local CSRF independently of a stalled WebView.
The full private paired APP and factory backend are not part of this repository.

GET proxy paths are status/map only. POST paths are explicit navigation workflow
operations; simulated operator overrides are not exposed. The real console still
rejects hardware execution. Browser mutations require the workbench token and a
matching origin; the client marker is not authentication and this router must
remain inside the existing paired network/access boundary.
