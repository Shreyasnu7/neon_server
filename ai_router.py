# ai_router.py
import os, uuid, shutil
from fastapi import APIRouter, UploadFile, File, Form
from api_schemas import MultimodalRequest, VideoLinkRequest
from utils import save_json, new_job
from cloud_ai.plan_generator import PlanGenerator
from cloud_ai.llm import ask_ai  # your existing LLM wrapper


# -----------------------
#  Text → Job
# -----------------------
from cloud_ai.dependencies import get_orchestrator

ai_router = APIRouter(tags=["AI"])

@ai_router.post("/text")
async def ai_text(req: MultimodalRequest):
    orchestrator = get_orchestrator()
    
    # Unified Pipeline Call
    result = orchestrator.process_multimodal_request(
        text=req.text,
        user_id=req.user_id,
        drone_id=req.drone_id,
        images=req.images,
        video=req.video
    )
    return result
    return result

# -----------------------
#  Image upload (multiple)
# -----------------------
@ai_router.post("/image")
async def ai_image(
    user_id: str = Form(...),
    drone_id: str = Form(...),
    files: list[UploadFile] = File(...)
):
    paths = []
    for file in files:
        filename = f"{uuid.uuid4()}_{file.filename}"
        filepath = os.path.join(UPLOAD_DIR, filename)
        with open(filepath, "wb") as f:
            shutil.copyfileobj(file.file, f)
        paths.append(filepath)

    job_id = str(uuid.uuid4())
    new_job(job_id, {
        "type": "image",
        "user_id": user_id,
        "drone_id": drone_id,
        "files": paths
    })

    return {"job_id": job_id, "uploaded": len(paths)}

# -----------------------
#  Video upload (≤100MB)
# -----------------------
@ai_router.post("/video")
async def ai_video(
    user_id: str = Form(...),
    drone_id: str = Form(...),
    file: UploadFile = File(...)
):
    filename = f"{uuid.uuid4()}_{file.filename}"
    filepath = os.path.join(UPLOAD_DIR, filename)

    with open(filepath, "wb") as f:
        shutil.copyfileobj(file.file, f)

    job_id = str(uuid.uuid4())
    new_job(job_id, {
        "type": "video",
        "user_id": user_id,
        "drone_id": drone_id,
        "file": filepath
    })

    return {"job_id": job_id, "status": "queued"}

# -----------------------
#  Video link
# -----------------------
@ai_router.post("/video_link")
async def ai_video_link(req: VideoLinkRequest):
    job_id = str(uuid.uuid4())
    payload = {
        "type": "video_link",
        "user_id": req.user_id,
        "drone_id": req.drone_id,
        "url": req.url
    }
    new_job(job_id, payload)
    return {"job_id": job_id, "status": "queued"}