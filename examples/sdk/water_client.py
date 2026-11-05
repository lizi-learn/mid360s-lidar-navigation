"""Loopback bridge example; LF is our explicit bridge profile."""
from navigation_sdk.water import WaterClient
client=WaterClient('127.0.0.1',31001,terminator='\n')
try:
    print(client.markers())
    print(client.status())
    # client.move(marker='产品展示')  # explicit business navigation request
finally:client.close()
