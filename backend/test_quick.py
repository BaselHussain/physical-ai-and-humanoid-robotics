"""Quick sync test to see actual errors"""
import requests
import time
import subprocess
import sys

# Start server
print("Starting server...")
proc = subprocess.Popen(
    [sys.executable, "-m", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    bufsize=1
)

# Wait for startup
time.sleep(8)

try:
    # Test health
    print("\n1. Testing health...")
    r = requests.get("http://localhost:8000/health")
    print(f"   Status: {r.status_code}")

    # Test signin with existing user
    print("\n2. Testing signin...")
    r = requests.post("http://localhost:8000/auth/signin", json={
        "email": "live_test_12345@test.com",
        "password": "TestPass123!"
    })
    print(f"   Status: {r.status_code}")
    print(f"   Response: {r.json()}")

    # Get server output
    print("\n3. Server output:")
    for line in iter(proc.stdout.readline, ''):
        if not line:
            break
        print(f"   {line.strip()}")
        if "signin" in line.lower() or "error" in line.lower():
            # Show a few more lines
            for _ in range(5):
                line = proc.stdout.readline()
                if line:
                    print(f"   {line.strip()}")

finally:
    proc.terminate()
    proc.wait(timeout=5)
    print("\nServer stopped")
