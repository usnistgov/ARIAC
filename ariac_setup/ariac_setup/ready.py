import asyncio

from ariac_setup.utils import ROSAsyncAdapter


async def run():
    await ROSAsyncAdapter.wait_for_gz_clock()


def main():
    asyncio.run(run())