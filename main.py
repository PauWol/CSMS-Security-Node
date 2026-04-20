from core import start, task
from core.comms.mesh import mesh_callback
from lib.detect import Monitor

# Global Instances
monitor = Monitor()


# delayed boot init
@task(0, boot=True)
async def init():
    global monitor
    await monitor.init()


@mesh_callback()
async def mesh_callback(host, msg):
    print(msg)


start()
