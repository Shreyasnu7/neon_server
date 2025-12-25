# utils.py
import json, os

import tempfile
JOBS_DIR = os.path.join(tempfile.gettempdir(), "drone_jobs")
os.makedirs(JOBS_DIR, exist_ok=True)

def save_json(path, data):
    with open(path, "w") as f:
        json.dump(data, f)

def new_job(job_id, payload):
    path = f"{JOBS_DIR}/job_{job_id}.json"
    save_json(path, payload)
    return path
