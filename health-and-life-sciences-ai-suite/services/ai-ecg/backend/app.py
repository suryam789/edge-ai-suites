from fastapi import FastAPI, UploadFile, File, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import os
import shutil
import time
from inference.engine import ECGInferenceEngine

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE_DIR, "data")
os.makedirs(DATA_DIR, exist_ok=True)

app = FastAPI(title="ECG OpenVINO API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

engine = ECGInferenceEngine()

# ---- STREAM STATE (VERY IMPORTANT) ----
stream_files = []
stream_index = 0
streaming_enabled = False


@app.get("/health")
def health():
    return {"status": "ok"}


@app.post("/start")
def start_streaming():
    """Enable streaming responses for /predict_stream_next.

    UI should call this endpoint when the user clicks the global
    "start" button so that subsequent polling begins returning data.
    """
    global streaming_enabled
    streaming_enabled = True
    return {"status": "started"}


@app.post("/stop")
def stop_streaming():
    """Disable streaming responses for /predict_stream_next.

    When disabled, /predict_stream_next will return HTTP 204 so the
    aggregator can stop broadcasting without failing.
    """
    global streaming_enabled
    streaming_enabled = False
    return {"status": "stopped"}


@app.post("/upload")
async def upload_ecg(file: UploadFile = File(...)):
    if not file.filename.endswith(".mat"):
        raise HTTPException(status_code=400, detail="Only .mat files are allowed")

    file_path = os.path.join(DATA_DIR, file.filename)
    try:
        with open(file_path, "wb") as buffer:
            shutil.copyfileobj(file.file, buffer)
    finally:
        await file.close()

    return {"status": "uploaded", "filename": file.filename}


@app.get("/predict/{filename}")
def predict_ecg(filename: str):
    file_path = os.path.join(DATA_DIR, filename)
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail="File not found")

    try:
        pred = engine.predict(filename)
        return {
            "signal": pred["signal"],
            "result": pred["result"],
            "inference_ms": pred["inference_ms"],
            "length": len(pred["signal"]),
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


# -------- NEW TRUE STREAMING ENDPOINT --------
@app.get("/predict_stream_next")
def predict_stream_next():
    """
    Returns ONE file inference per call.
    Cycles through files endlessly.
    """
    global stream_files, stream_index, streaming_enabled

    # If streaming is disabled, indicate "no content" to the caller.
    if not streaming_enabled:
        # 204 is intentionally not treated as an error by the
        # aggregator; it simply means "no new data".
        raise HTTPException(status_code=204, detail="Streaming disabled")

    if not stream_files:
        stream_files = sorted(
            [f for f in os.listdir(DATA_DIR) if f.endswith(".mat")]
        )
        stream_index = 0

    if not stream_files:
        raise HTTPException(status_code=404, detail="No .mat files found")

    filename = stream_files[stream_index]
    stream_index = (stream_index + 1) % len(stream_files)

    pred = engine.predict(filename)

    return {
        "file": filename,
        "signal": pred["signal"],
        "result": pred["result"],
        "inference_ms": pred["inference_ms"],
        "length": len(pred["signal"]),
    }
