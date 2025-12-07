"""
Test script to verify the fix for the RunResultStreaming error
"""
import asyncio
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

# Import the modules we need to test
from agents import Runner
from app.rag_agent import docs_agent

async def test_runner():
    print("Testing the Runner functionality...")

    try:
        # Test using Runner.run (the async method we're using in main.py)
        print("Testing Runner.run()...")
        result = await Runner.run(docs_agent, input="Hello, how are you?")
        print(f"Success! Got result: {type(result)}")
        print(f"Final output type: {type(result.final_output)}")
        print(f"Final output: {result.final_output}")

    except Exception as e:
        print(f"Error with Runner.run(): {e}")
        import traceback
        traceback.print_exc()

    try:
        # Test using Runner.run_sync (synchronous method)
        print("\nTesting Runner.run_sync()...")
        result = Runner.run_sync(docs_agent, input="Hello, how are you?")
        print(f"Success! Got result: {type(result)}")
        print(f"Final output type: {type(result.final_output)}")
        print(f"Final output: {result.final_output}")

    except Exception as e:
        print(f"Error with Runner.run_sync(): {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test_runner())