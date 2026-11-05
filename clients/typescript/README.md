# @p2-navigation/sdk · 0.1.0

Node 18+ / TypeScript. Install the release `.tgz` with `npm install ./p2-navigation-sdk-0.1.0.tgz`.

```typescript
import {NavigationClient} from '@p2-navigation/sdk';
const nav = new NavigationClient('http://127.0.0.1:18940');
const task = await nav.visit('产品展示', {idempotencyKey:'visit-20261105-01'});
console.log(await nav.task(task.id)); // accepted != arrived
// await nav.cancel(task.id); // cancels only this task when supported
```

Start the Python companion first: `navbridge --backend mock`. It makes no motion and never invents arrival. The mock fixture advances only through explicit `/mock/advance` calls. For production providers and WATER compatibility boundaries see `docs/sdk/README.md` in the source release and the documentation site. The static website does not control your robot.

API keys belong in a server process/environment, never in a public page. There is no silent mutation retry. A client wait timeout/abort does not cancel navigation; use explicit cancel or global navigation stop according to your application.
