"""RoboSim server entry point.

Run with: python server.py
Or: uvicorn src.interfaces.api.main:app --reload --port 8000
"""

import uvicorn


if __name__ == "__main__":
    uvicorn.run(
        "src.interfaces.api.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info",
    )
