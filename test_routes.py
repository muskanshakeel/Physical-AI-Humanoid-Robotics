import sys
import os
sys.path.insert(0, os.path.join(os.getcwd(), 'backend'))

# Set the working directory to backend
os.chdir('backend')

try:
    from src.main import app
    print("Available routes:")
    for route in app.routes:
        print(f"  {route.methods} {route.path}")
except Exception as e:
    print(f"Error importing app: {e}")
    import traceback
    traceback.print_exc()