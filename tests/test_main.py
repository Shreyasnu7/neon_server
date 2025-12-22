
from fastapi.testclient import TestClient
from main import app

client = TestClient(app)

def test_read_root():
    response = client.get("/")
    assert response.status_code == 200
    assert response.json()["status"] == "Quantum Drone Server Online"

def test_ai_router_exists():
    # Should be 422 (Validation Error) for missing body, NOT 404
    response = client.post("/ai/text", json={}) 
    assert response.status_code in [400, 422]
