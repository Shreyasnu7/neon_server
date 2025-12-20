# utils.py
import json, os

JOBS_DIR = "/mnt/data/jobs"
os.makedirs(JOBS_DIR, exist_ok=True)

def save_json(path, data):
    with open(path, "w") as f:
        json.dump(data, f)

def new_job(job_id, payload):
    path = f"{JOBS_DIR}/job_{job_id}.json"
    save_json(path, payload)
    return path
