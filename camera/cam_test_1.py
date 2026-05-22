from open_gopro import WiredGoPro
import asyncio
async def main():
	gopro = WiredGoPro()

	await gopro.open()

	print("YAYAY")

if __name__ == "__main__":
	asyncio.run(main())
