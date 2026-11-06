
import asyncio
import os

from viam.robot.client import RobotClient

async def connect():
    options = RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"],
        api_key_id=os.environ["VIAM_API_KEY_ID"]
    )

    return await RobotClient.at_address(
        "armfarm1-main.310sld03v2.viam.cloud",
        options
    )

async def main():
    async with await connect() as machine:
        print("Successfully connected to Viam!")
        print("Available resources:")
        print(machine.resource_names)

if __name__ == "__main__":
    asyncio.run(main())
