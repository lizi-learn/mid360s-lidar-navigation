import {NavigationClient} from '@p2-navigation/sdk';
const nav = new NavigationClient('http://127.0.0.1:18940');
const capabilities = await nav.capabilities();
if (capabilities.backend === 'mock') {
  const task = await nav.visit('产品展示', {idempotencyKey:'example-visit-001'});
  console.log(await nav.task(task.id));
  console.log(await nav.cancel(task.id));
}
