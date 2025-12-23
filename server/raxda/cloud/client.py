import requests

CLOUD_URL = "https://your-cloud-server/api/command"

def get_command(drone_id):
    r = requests.get(f"{CLOUD_URL}/{drone_id}")
    if r.status_code == 200:
        return r.json()
    return None