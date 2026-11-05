"""Run against --backend mock first. No automatic mock arrival or hardware call."""
from navigation_sdk import NavigationClient
nav=NavigationClient('http://127.0.0.1:18940')
print(nav.capabilities())
if nav.capabilities()['backend']=='mock':
    task=nav.visit('产品展示',idempotency_key='example-visit-001')
    print(nav.task(task['id']))
    print(nav.cancel(task['id']))
