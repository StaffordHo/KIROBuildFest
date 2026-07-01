"""RoboSim server entry point.

Run with: python server.py
Or: uvicorn src.interfaces.api.main:app --reload --port 8000
"""

import os
import uvicorn


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 8000))
    reload = os.environ.get("DEBUG", "true").lower() == "true"
    uvicorn.run(
        "src.interfaces.api.main:app",
        host="0.0.0.0",
        port=port,
        reload=reload,
        log_level="info",
    )
